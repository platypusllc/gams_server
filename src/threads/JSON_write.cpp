
#include "gams/loggers/GlobalLogger.h"
#include "JSON_write.h"

namespace knowledge = madara::knowledge;

// constructor
threads::JSON_write::JSON_write (std::shared_ptr<boost::asio::serial_port> port, Containers & containers)
: io_thread(port, containers) 
{
}

// destructor
threads::JSON_write::~JSON_write ()
{
}

/**
 * Executes the actual thread logic. Best practice is to simply do one loop
 * iteration. If you want a long running thread that executes something
 * frequently, see the madara::threads::Threader::runHz method in your
 * controller.
 **/
void
threads::JSON_write::run (void)
{
  /**
   * the MADARA logger is thread-safe, fast, and allows for specifying
   * various options like output files and multiple output targets (
   * e.g., std::cerr, a system log, and a thread_output.txt file). You
   * can create your own custom log levels or loggers as well.
   **/
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "threads::JSON_write::run:" 
    " executing\n");

  // construct motor json
  json motor_json;
  std::vector<double> motor_signals = containers_.motor_signals.to_record().to_doubles();
  if (motor_signals.size() < 2) 
  {
      madara_logger_ptr_log(gams::loggers::global_logger.get(), 
        gams::loggers::LOG_MAJOR,
        "threads::JSON_write::run"
        " WARNING: there were not two motor signals in knowledge base\n");
      return;
  }
  //printf("motor signal 0 = %.3f   motor signal 1 = %.3f\n", motor_signals.at(0), motor_signals.at(1));

  //Send arming signal to hardware
  if ( containers_.arm_signal == 1 )
  {
    madara_logger_ptr_log(gams::loggers::global_logger.get(), 
      gams::loggers::LOG_MAJOR,
      "threads::JSON_write::run:"
      " INFO: Arm Signal Received, sending EBoard arm command\n");
    json arm_json;
    arm_json["e"] = { { "cmd", "arm" } };
    containers_.arm_signal = 0;
    write( arm_json );
  }
  
  
  // Differential Drive Support only
  motor_json["m0"] = { {"v", motor_signals.at(0)} };
  motor_json["m1"] = { {"v", motor_signals.at(1)} };

  madara_logger_ptr_log(gams::loggers::global_logger.get(), 
    gams::loggers::LOG_MAJOR,
    "threads::JSON_write::run:"
    " INFO: Writing motor commands [%f, %f]\n",
    motor_signals.at(0),
    motor_signals.at(1));

  write(motor_json);
  

  //Check for error corde and inform eboard.
  /*if ( containers_.error_signal != 0 )
  {
    json error_json;
    if ( containers_.error_signal == 1)
    { 
      error_json["o"] = { { "e", "conn" } };
      write(error_json); 
    }
    else if ( containers_.error_signal == 2 )
    { 
      error_json["o"] = { { "e", "gps"  } };
      write(error_json); 
    }
    else if ( containers_.error_signal == 3 )
    { 
      error_json["o"] = { { "e", "ahrs" } };
      write(error_json); 
    }

  }*/
}

void threads::JSON_write::write(json & json_data)
{
  raw_data_.clear();
  raw_data_ = json_data.dump();
  strncpy(raw_buffer_, raw_data_.c_str(), raw_data_.size());
  raw_buffer_[raw_data_.size()] = '\n'; // explicitly add a newline character at the end?
  //printf("JSON write buffer: %s\n", raw_buffer);
  boost::system::error_code ec;
  port_->write_some(boost::asio::buffer(raw_buffer_, raw_data_.size()+1), ec);
  if (!ec) 
  {
      return;
  }
  else 
  {
    madara_logger_ptr_log(gams::loggers::global_logger.get(), 
      gams::loggers::LOG_MAJOR,
      "treads::JSON_write::write:"
      " ERROR: writ_some() failed\n");
  }

}
