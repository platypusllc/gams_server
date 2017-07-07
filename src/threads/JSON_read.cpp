
#include "gams/loggers/GlobalLogger.h"
#include "JSON_read.h"

namespace knowledge = madara::knowledge;

// constructor
threads::JSON_read::JSON_read (std::shared_ptr<boost::asio::serial_port> port, Containers & containers, threads::localization * localization_ref)
: threads::io_thread(port, containers), end_of_line_char_(END_OF_LINE_CHAR), rejected_line_count_(0)
{
  new_sensor_callback = std::bind(& threads::localization::new_sensor_update, localization_ref, std::placeholders::_1);
}

// destructor
threads::JSON_read::~JSON_read ()
{
}

/**
 * Executes the actual thread logic. Best practice is to simply do one loop
 * iteration. If you want a long running thread that executes something
 * frequently, see the madara::threads::Threader::runHz method in your
 * controller.
 **/
void
threads::JSON_read::run (void)
{
  /**
   * the MADARA logger is thread-safe, fast, and allows for specifying
   * various options like output files and multiple output targets (
   * e.g., std::cerr, a system log, and a thread_output.txt file). You
   * can create your own custom log levels or loggers as well.
   **/
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "threads::JSON_read::run:" 
    " executing\n");

  boost::system::error_code ec;
  int bytes_read = port_->read_some(boost::asio::buffer(raw_buffer_, BUFFER_SIZE), ec);
  if (!ec && bytes_read > 0) 
  {
    for (uint i = 0; i < bytes_read; i++) 
    {
      char c = raw_buffer_[i];
      if (c == end_of_line_char_) 
      {
        if (rejected_line_count_ > INITIAL_REJECT_COUNT) 
        {
          json j;
          try
          {
            //printf("serial input: %s \n", raw_data_.c_str());
            j = json::parse(raw_data_);
            std::string primary_key(j.begin().key().data());
            
            // SENSORS
            if (primary_key.substr(0,1) == "s") // look for a leading "s"
            {
              std::string type = j.front().find("type").value();
              utility::string_tools::remove_quotes(type);
              
              if (type.compare("battery") == 0) 
              {
                std::string data = j.front().find("data").value();
                utility::string_tools::remove_quotes(data);
                std::vector<std::string> elems = utility::string_tools::split(data, ' ');
                double battery_voltage = std::stod(elems.at(0), nullptr);
                //printf("battery voltage = %.3f V\n", battery_voltage);
                containers_.battery_voltage = battery_voltage;
              }
             
              if (type.compare("gps") == 0)              
              {
                std::string nmea = j.front().find("data").value();
                utility::string_tools::remove_quotes(nmea);
                std::vector<std::string> elems = utility::string_tools::split(nmea, ',');

                //{"s1":{"type":"AdafruitGPS","data":"$GPRMC,183146.935,V,,,,,0.00,0.00,300616,,,N*49"}}
                double lat = -999.;
                double lon = -999.;
                if ( !elems.at((int)RMC_STRING::LAT_RAW).size()|| !elems.at((int)RMC_STRING::LON_RAW).size())
                {
                  printf("WARNING: Adafruit GPS does not have a fix\n");
                }
                else
                {                                 
                  lat = std::stod(elems.at((int)RMC_STRING::LAT_RAW), nullptr);
                  lon = std::stod(elems.at((int)RMC_STRING::LON_RAW), nullptr);
                  lat = GPRMC_to_degrees(lat)*( elems.at( (int)RMC_STRING::LAT_CARDINAL ).compare(NORTH) ?-1.:1.);
                  lon = GPRMC_to_degrees(lon)*(elems.at((int)RMC_STRING::LON_CARDINAL).compare(EAST)     ?-1.:1.);
                  //printf("Received Adafruit GPS lat/long = %f, %f\n", lat, lon);
                  GeographicLib::GeoCoords coord(lat, lon);
                  std::vector<double> gps_utm = {coord.Easting(), coord.Northing()};
                  containers_.gpsZone = coord.Zone();
                  if (coord.Northp())
                  {
                    containers_.northernHemisphere = 1;
                  }
                  else
                  {
                    containers_.northernHemisphere = 0;
                  }              
                  Eigen::MatrixXd covariance(2, 2);
                  covariance = Eigen::MatrixXd::Identity(2, 2);
                  Datum datum(SENSOR_TYPE::GPS, SENSOR_CATEGORY::LOCALIZATION, gps_utm, covariance);
                  new_sensor_callback(datum);                  
                }                               
              }
              else if (type.compare("AHRS") == 0)              
              {
                //printf("Received AHRS data\n");
                std::string nmea = j.front().find("data").value();
                //Seperate data by commas
                std::vector<std::string> ahrs_data;
                ahrs_data = utility::string_tools::split(nmea, '*');
                std::string data = ahrs_data[0];
                //printf("Splitting data string : %s \n", data.c_str());
                ahrs_data = utility::string_tools::split(data, ',');

                try{
                  double euler_yaw = std::stod(ahrs_data[4]);
                  double yaw = (-euler_yaw - 90.0);
                  if (yaw < -180.0)
                  {
                    yaw += 360.0;
                  }
                  yaw *= M_PI/180.0;
                  std::vector<double> compass = {yaw}; 
                  Eigen::MatrixXd covariance(1, 1);
                  covariance = 0.00001*Eigen::MatrixXd::Identity(1, 1); 
                  Datum datum(SENSOR_TYPE::COMPASS, SENSOR_CATEGORY::LOCALIZATION, compass, covariance);
                  new_sensor_callback(datum);
                }catch (const std::invalid_argument&) {
                  printf("Argument is invalid\n");
                } catch (const std::out_of_range&) {
                  printf("Argument is out of range for a double\n");
                }
              }
              else if (type.compare("imu") == 0)
              {
                std::string data = j.front().find("data").value();
                std::vector<std::string> imu_data;
                imu_data = utility::string_tools::split(data, ',');

                try{
                  double raw_yaw = std::stod(imu_data[0]);
                  raw_yaw -= 90.0; // Offset angle to account for mounting orientation
                  if (raw_yaw < 0.0)
                  {
                    raw_yaw += 360.0;
                  }
                  double yaw = 180.0 - raw_yaw; // Flip direction and map to -180 to 180
                  yaw *= M_PI / 180.0; // Convert to Radians

                  std::vector<double> compass = {yaw};
                  Eigen::MatrixXd covariance(1, 1);
                  covariance = 0.00001*Eigen::MatrixXd::Identity(1, 1); 
                  Datum datum(SENSOR_TYPE::COMPASS, SENSOR_CATEGORY::LOCALIZATION, compass, covariance);
                  new_sensor_callback(datum);
                }catch (const std::invalid_argument&) {
                  printf("Argument is invalid\n");
                } catch (const std::out_of_range&) {
                  printf("Argument is out of range for a double\n");
                }
              }
            }
          }
          catch (std::exception e) 
          {
            printf("ERROR: json parse failed: %s\n", e.what());
            printf("Serial Input: %s\n", raw_data_.c_str());
          }
        }
        else
        {
          rejected_line_count_++;
        }
        raw_data_.clear();
      }
      else
      {
        raw_data_ += c;
      }
    }        
  }
  else if (ec){
    printf("ERROR: port_->read_some() error: %s\n", ec.message().c_str());
  }
}

double threads::JSON_read::GPRMC_to_degrees(double value)
{
  double fullDegrees = floor(value/100.0);
  double minutes = value - fullDegrees*100.0;
  return (fullDegrees + minutes/60.0);
}
