
#include "gams/loggers/GlobalLogger.h"
#include "localization.h"

namespace knowledge = madara::knowledge;

// constructor
threads::localization::localization (Containers & containers)
: containers_(containers)
{
  state = StateMatrix::Zero(); // [x y th xdot ydot thdot]
  t = utility::time_tools::now();
  t0 = utility::time_tools::now();
  
  QBase = 0.1*StateSizedSquareMatrix::Identity();
  P = StateSizedSquareMatrix::Zero();
  P(0, 0) = 5.0;
  P(1, 1) = 5.0;
  P(2, 2) = 1.0;
  Phi = StateSizedSquareMatrix::Identity();
  Phi_k = StateSizedSquareMatrix::Identity();
  G = StateSizedSquareMatrix::Identity();
  eastingNorthingHeading.resize(3);
  location.resize(3);
  local_state.resize(STATE_DIMENSION);
}

// destructor
threads::localization::~localization ()
{
}

/**
 * Initialization to a knowledge base. If you don't actually need access
 * to the knowledge base, just scheduling things in madara::threads::Threader,
 * then you can decide to delete this function or simply do nothing inside of
 * the function.
 **/
void
threads::localization::init (knowledge::KnowledgeBase & knowledge)
{
  // point our data plane to the knowledge base initializing the thread
  data_ = knowledge;
}

/**
 * Executes the actual thread logic. Best practice is to simply do one loop
 * iteration. If you want a long running thread that executes something
 * frequently, see the madara::threads::Threader::runHz method in your
 * controller.
 **/
void
threads::localization::run (void)
{
  /**
   * the MADARA logger is thread-safe, fast, and allows for specifying
   * various options like output files and multiple output targets (
   * e.g., std::cerr, a system log, and a thread_output.txt file). You
   * can create your own custom log levels or loggers as well.
   **/
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "threads::localization::run:" 
    " executing\n");

  update();
}

void threads::localization::new_sensor_update(Datum datum)
{
  //printf("Datum unique_id_count = %d    @ %s\n", Datum::unique_id_count, datum.human_readable_time().c_str());

  // Deal with initial GPS and compass values
  if (containers_.localized == 0)
  {
    madara_logger_ptr_log(gams::loggers::global_logger.get(), 
      gams::loggers::LOG_MAJOR,
      "threads::localizaion::new_sensor_update:"
      " INFO: Boat waiting for intial localizaion data\n");
    if (containers_.gps_init == 0 && datum.type() == SENSOR_TYPE::GPS)
    {
      madara_logger_ptr_log(gams::loggers::global_logger.get(), 
        gams::loggers::LOG_MAJOR,
        "threads::localizaion::new_sensor_update:"
        " INFO: Received first GPS Location [%f, %f\n",
        datum.value().at(0), datum.value().at(1));
      //printf("Received first GPS: %f, %f\n", datum.value().at(0), datum.value().at(1)); 
      containers_.gps_init = 1;
      home_x = datum.value().at(0);
      home_y = datum.value().at(1);
      containers_.self.agent.home.set(0, home_x);
      containers_.self.agent.home.set(1, home_y);
      containers_.self.agent.source.set(0, home_x);
      containers_.self.agent.source.set(1, home_y);
      containers_.self.agent.dest.set(0, home_x);
      containers_.self.agent.dest.set(1, home_y);
      state(0, 0) = 0.0;
      state(1, 0) = 0.0;      
    }
    if (containers_.compass_init == 0 && (datum.type() == SENSOR_TYPE::COMPASS || datum.type() == SENSOR_TYPE::AHRS))
    {
      madara_logger_ptr_log(gams::loggers::global_logger.get(), 
        gams::loggers::LOG_MAJOR,
        "threads::localization::new_sensor_update:"
        " INFO: Received first compass reading [%f]\n",
        datum.value().at(0));
      //printf("Received first compass: %f\n", datum.value().at(0));
      containers_.compass_init = 1;
      state(2, 0) = datum.value().at(0);
      //state(2, 0) = 1.0;
      heading = datum.value().at(0);
      //std::cout << "Updated state = " << state.transpose() << std::endl;
    }
    if (containers_.compass_init == 1 && containers_.gps_init == 1)
    {
      madara_logger_ptr_log(gams::loggers::global_logger.get(), 
        gams::loggers::LOG_MAJOR,
        "threads::localizaion::new_sensor_update:"
        " INFO: Localized, Sending Arm Signal\n");
      containers_.localized = 1; 
      containers_.arm_signal = 1;
      updateKB();     
    }
    return; // don't do anything until at least 1 gps and compass measurement come through
  }

  ///// BEGIN LOCKED SECTION
  std::lock_guard<std::mutex> lock(queue_mutex);
  if (data_queue.size() > MAX_DATA_QUEUE_SIZE)
  {
    madara_logger_ptr_log(gams::loggers::global_logger.get(), 
      gams::loggers::LOG_MAJOR,
      "threads::localizaion::new_sensor_update:"
      " WARNING: localization queue is too long, throwing away oldest datum\n");
    data_queue.pop(); // throw out oldest item
  }
  data_queue.push(datum);// push Datum to queue
  //printf("pushed datum to queue, size = %d\n", data_queue.size());
  ///// END LOCKED SECTION
}

void threads::localization::predict(double dt)
{
  // Kalman filter prediction step
  //printf("prediction step: dt = %f\n", dt);
  
  Q = QBase*dt;
  
  Phi(0, 3) = dt;
  Phi(1, 4) = dt;
  Phi(2, 5) = dt;
  
  Phi_k(0, 3) = dt;
  Phi_k(1, 4) = dt;
  Phi_k(2, 5) = dt;
  
  state = Phi*state; // Phi*x
  
  // wrap theta
  state(2,0) = utility::angle_tools::wrap_to_pi(state(2,0));
  
  P = Phi_k*P*Phi_k.transpose() + G*Q*G.transpose();
  
}

void threads::localization::updateKB()
{
  madara_logger_ptr_log(gams::loggers::global_logger.get(), 
    gams::loggers::LOG_MAJOR,
    "threads::localizaion::updateKB:"
    " INFO: Updating knowledge base\n");
  // update the knowledge base. Make sure to use containers_ so that these updates are not sent out constantly
  eastingNorthingHeading.at(0) = state(0, 0) + home_x;
  eastingNorthingHeading.at(1) = state(1, 0) + home_y;


  // sets heading to last value received from sensors, bypassing the KF!!!
  //eastingNorthingHeading.at(2) = state(2, 0);
  eastingNorthingHeading.at(2) = heading;
  
  //printf("heading = %f\n", state(2,0)*180.0/M_PI);
  
  containers_.eastingNorthingHeading.set(eastingNorthingHeading); // update the knowledge base
  
  try {
    coord.Reset(containers_.gpsZone.to_integer(), containers_.northernHemisphere.to_integer(), eastingNorthingHeading.at(0), eastingNorthingHeading.at(1));        
    location.at(0) = coord.Latitude();
    location.at(1) = coord.Longitude();
    location.at(2) = 0.0;
    containers_.location.set(location);
  } catch (const GeographicLib::GeographicErr&) {
    madara_logger_ptr_log(gams::loggers::global_logger.get(), 
      gams::loggers::LOG_MAJOR,
      "threads::localizaion::updateKB:"
      " Error: Error while converting UTM to Lat, Long\n");
  }
  for (int i = 0; i < state.rows(); i++) 
  {
    containers_.local_state.set(i, state(i, 0));
  }    
}

void threads::localization::update()
{
  bool new_datum_available = false;
  ///// BEGIN LOCKED SECTION
  {
    // pop Datum from queue
    std::lock_guard<std::mutex> lock(queue_mutex);
    //printf("is queue empty? size = %d\n", data_queue.size());
    if (!data_queue.empty())
    {
      new_datum_available = true;
      current_datum = data_queue.top();
      data_queue.pop();
      //printf("popped datum from queue, size = %d\n", data_queue.size());      
    }
    else 
    {
      //printf("no datum available\n");
    }
  }
  ///// END LOCKED SECTION

  //if there is a datum to process, continue, else return
  if (!new_datum_available) return;
  
  //printf("Processing a datum of type: %s", current_datum.type_string().c_str());
  //std::cout << " value = " << current_datum.value() << std::endl;
  
  // prediction step
  // calculate dt using current time and datum time stamp    
  double dt = utility::time_tools::dt(t, current_datum.timestamp());
  if (dt < 0)
  {
    madara_logger_ptr_log(gams::loggers::global_logger.get(), 
      gams::loggers::LOG_MAJOR,
      "threads::localizaion::update:"
      " WARNING: localization timestep is negative, skipping sensor update\n");
    return;
  }
  
  predict(dt);
  t = current_datum.timestamp();
  
  // convert value std::vector into a column matrix
  std::vector<double> value = current_datum.value();
  if (current_datum.type() == SENSOR_TYPE::GPS)
  {
    madara_logger_ptr_log(gams::loggers::global_logger.get(), 
      gams::loggers::LOG_MAJOR,
      "threads::localizaion::update:"
      " INFO: Processing GPS Datum [%f, %f]\n",
      value.at(0), value.at(1));


    value.at(0) -= home_x; // use local frame
    value.at(1) -= home_y;

    containers_.heartbeat_gps = 1;

    /*Don't bother computing this since we don't currently use it

    
    // save gps information for use in gps velocity calculation
    tR = utility::time_tools::dt(t0, t);
    //printf("t = %f seconds\n", tR);
    gps_data_t.push_back(utility::time_tools::dt(t0, current_datum.timestamp()));
    gps_data_x.push_back(value.at(0));
    gps_data_y.push_back(value.at(1));
    // eliminate old gps data
    for (int i = gps_data_t.size()-1; i > -1; i--)
    {
      if (tR - gps_data_t.at(i) > GPS_HISTORY_TIME_WINDOW)
      {
        //printf("gps datum age = %f seconds, deleting it...\n", tR - gps_data_t.at(i));
        gps_data_t.erase(gps_data_t.begin() + i);
        gps_data_x.erase(gps_data_x.begin() + i);
        gps_data_y.erase(gps_data_y.begin() + i);
      }
    }

    // if there are enough gps data remaining, calculate dx/dt and dy/dt
    if (gps_data_t.size() >= GPS_HISTORY_REQUIRED_SIZE)
    {
      //printf("There are at least %d gps data available, calculating velocities...\n", GPS_HISTORY_REQUIRED_SIZE);
      double n, s_t, s_x, s_y, s_tx, s_ty, s_tt, dx_dt, dy_dt;
      n = (double)gps_data_t.size();
      s_t = std::accumulate(gps_data_t.begin(), gps_data_t.end(), 0.0);
      s_x = std::accumulate(gps_data_x.begin(), gps_data_x.end(), 0.0);
      s_y = std::accumulate(gps_data_y.begin(), gps_data_y.end(), 0.0);
      s_tt = std::inner_product(gps_data_t.begin(), gps_data_t.end(), gps_data_t.begin(), 0.0);
      s_tx = std::inner_product(gps_data_t.begin(), gps_data_t.end(), gps_data_x.begin(), 0.0);        
      s_ty = std::inner_product(gps_data_t.begin(), gps_data_t.end(), gps_data_y.begin(), 0.0);
      dx_dt = (n*s_tx - s_t*s_x)/(n*s_tt - s_t*s_t);
      dy_dt = (n*s_ty - s_t*s_y)/(n*s_tt - s_t*s_t);
      //printf("dx_dt = %f m/s,  dy_dt = %f m/s\n", dx_dt, dy_dt);        
      std::vector<double> velocities = {dx_dt, dy_dt};
      Eigen::MatrixXd covariance(2, 2);
      covariance = Eigen::MatrixXd::Identity(2, 2); 
      Datum datum(SENSOR_TYPE::GPS_VELOCITY, SENSOR_CATEGORY::LOCALIZATION, velocities, covariance);       
      // Don't use this data right now...
      //new_sensor_update(datum); // put this gps_velocity datum into the queue
    }
    */
  }
  
  Eigen::Map<Eigen::MatrixXd> z_(value.data(), value.size(), 1);
  z = z_;
  R = current_datum.covariance();
  setH();
  
  K = P*H.transpose()*(H*P*H.transpose() + R).inverse();
  
  dz = z - H*state;
  if (current_datum.type() == SENSOR_TYPE::COMPASS)
  {
    containers_.heartbeat_compass = 1;
    dz(0, 0) = utility::angle_tools::minimum_difference(dz(0, 0)); // use true angular difference, not algebraic difference
  }
          
  S = H*P*H.transpose();
  
  if (S.determinant() < pow(10.0, -12.0))
  {
    madara_logger_ptr_log(gams::loggers::global_logger.get(), 
      gams::loggers::LOG_MAJOR,
      "threads::localizaion::update:"
      " WARNING: innovation covariance is singular\n");
    //printf("WARNING: innovation covariance is singular for sensor type: %s\n", current_datum.type_string().c_str());
    std::cout << "z = " << z << std::endl;
    std::cout << "H = " << H << std::endl;
    std::cout << "R = " << R << std::endl;
    std::cout << "P = " << P << std::endl;
    std::cout << "K = " << K << std::endl;
    std::cout << "S = " << S << std::endl;
    std::cout << "det(S) = " << S.determinant() << " vs. " << pow(10.0, -6.0) << std::endl;
    return;
  }
  
  double d = sqrt((dz.transpose()*S.inverse()*dz)(0, 0));
  //printf("Mahalonobis distance = %f\n", d);
  
  state += K*dz;
  P = (StateSizedSquareMatrix::Identity() - K*H)*P;        
  //std::cout << "Updated state = " << state.transpose() << std::endl;
  
  updateKB();
}

void threads::localization::setH()
{
  //double th = state(2, 0);
  //double s = sin(th);
  //double c = cos(th);
  
  H = Eigen::MatrixXd::Zero(z.rows(), STATE_DIMENSION);
  if (current_datum.type() == SENSOR_TYPE::GPS)
  {
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;
  }
  else if (current_datum.type() == SENSOR_TYPE::COMPASS) 
  {
    H(0, 2) = 1.0;
    heading = current_datum.value().at(0);
  }
  else if (current_datum.type() == SENSOR_TYPE::GYRO) 
  {
    H(0, 5) = 1.0;
  }
  else if (current_datum.type() == SENSOR_TYPE::AHRS)
  {
    H(0, 2) = 1.0;
    H(1, 5) = 1.0;
  }
  else if (current_datum.type() == SENSOR_TYPE::GPS_VELOCITY) 
  {
    H(0, 3) = 1.0;
    H(1, 4) = 1.0;    
  }
}
