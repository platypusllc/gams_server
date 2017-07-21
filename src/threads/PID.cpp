
#include "gams/loggers/GlobalLogger.h"
#include "PID.h"

namespace knowledge = madara::knowledge;

// constructor
threads::PID::PID (Containers & containers)
: containers_(containers), dt_(0.0), prevError_(0.0), errorDerivative_(0.0), errorSum_(0.0)
{
  currentTime_ = utility::time_tools::now();
  prevTime_ = currentTime_;

  update_coefficients();
}

// destructor
threads::PID::~PID ()
{
}

/**
 * Initialization to a knowledge base. If you don't actually need access
 * to the knowledge base, just scheduling things in madara::threads::Threader,
 * then you can decide to delete this function or simply do nothing inside of
 * the function.
 **/
void
threads::PID::init (knowledge::KnowledgeBase & knowledge)
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
threads::PID::run (void)
{
  /**
   * the MADARA logger is thread-safe, fast, and allows for specifying
   * various options like output files and multiple output targets (
   * e.g., std::cerr, a system log, and a thread_output.txt file). You
   * can create your own custom log levels or loggers as well.
   **/
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "threads::PID::run:" 
    " executing\n");

  if (containers_.autonomy_enabled == 1 && containers_.teleop_status != 1 && containers_.localized == 1)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MAJOR,
          "threads::PID::run:" 
          " INFO: Autonomy enabled and Boat is localized\n");
      // goal state - determined by containers for agent.id.source, agent.id.destination, and agent.id.desired_velocity
      double x_dest, x_source, x_current, y_dest, y_source, y_current, th_full, th_current; 
      double dx_current, dx_full, dy_current, dy_full, L_current, L_full, dth;
      double projected_length, distance_from_ideal_line;
      double lookahead_distance, dx_lookahead, dy_lookahead;
      double heading_desired, heading_current, heading_error, heading_signal;
      x_dest = containers_.self.agent.dest[0] - containers_.self.agent.home[0];
      y_dest = containers_.self.agent.dest[1] - containers_.self.agent.home[1];
      x_source = containers_.self.agent.source[0] - containers_.self.agent.home[0];
      y_source = containers_.self.agent.source[1] - containers_.self.agent.home[1];
      x_current = containers_.local_state[0];
      y_current = containers_.local_state[1];
      heading_current = containers_.local_state[2];
      
      //printf("Home: %f, %f; Source: %f,%f; Current: %f, %f; Destination: %f,%f\n", 
      //  containers_.self.agent.home[0], containers_.self.agent.home[1], containers_.self.agent.source[0], containers_.self.agent.source[1],
      //  containers_.eastingNorthingHeading[0], containers_.eastingNorthingHeading[1], containers_.self.agent.dest[0], containers_.self.agent.dest[1]);
      //printf("Source: %f, %f; Current: %f, %f; Desired: %f, %f\n", x_source, y_source, x_current, y_current, x_dest, y_dest);
      //printf("%f     %f\n", x_current, y_current);
      
      // Compute current distance to destination
      containers_.dist_to_dest = sqrt(pow(x_dest - x_current, 2.) + pow(y_dest - y_current, 2.));
      //printf("Distance to destination: %f\n", containers_.dist_to_dest.to_double());

      // If you are not yet at the destination (within the suffcient proximity)
      if (containers_.dist_to_dest.to_double() > containers_.sufficientProximity.to_double())
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MAJOR,
          "threads::PID::run:" 
          " INFO: Moving to desired destination [%f, %f]\n",
          containers_.self.agent.dest[0],
          containers_.self.agent.dest[1]);
        dx_full = x_dest - x_source; // x distance from starting location to destination
        dx_current = x_current - x_source; // x distance from current boat position to destination
        dy_full = y_dest - y_source; // y distance from starting location to destination
        dy_current = y_current - y_source; // y distance from current boat position to destination
        L_full = sqrt(pow(dx_full, 2.) + pow(dy_full, 2.)); // length of line between starting location and destination
        L_current = sqrt(pow(dx_current, 2.) + pow(dy_current, 2.)); // length of line between current boat position and destination
        
        //printf("dx_current = %f   dy_current = %f   dx_full = %f   dy_full = %f\n", dx_current, dy_current, dx_full, dy_full);
        //printf("L_full = %f   L_current = %f\n", L_full, L_current);
        
        th_full = atan2(dy_full, dx_full); // angle of desired line of travel
        th_current = atan2(dy_current, dx_current); // angle of line from boat to destination
        
        //printf("th_full = %f   heading_current = %f\n", th_full, heading_current);
        
        // Project boat position to desired line of travel, compute point along line to travel to
        dth = std::abs(utility::angle_tools::minimum_difference(th_full - th_current));
        projected_length = L_current*cos(dth);
        distance_from_ideal_line = L_current*sin(dth);
        lookahead_distance = containers_.LOS_lookahead.to_double(); // TODO - use a dynamic lookahead?     
        std::vector<double> projected_state = {x_source + L_current*cos(th_full), y_source + L_current*sin(th_full)};
        std::vector<double> lookahead_state = {projected_state.at(0) + lookahead_distance*cos(th_full), projected_state.at(1) + lookahead_distance*sin(th_full)};
        if (  L_current + lookahead_distance > L_full )
        {
            lookahead_state[0] = x_dest;
            lookahead_state[1] = y_dest;
        }
        
        // IMPORTANT NOTE: the ideal state (the lookahead) is allowed to go past the actual destination because it is just a tool to get the boat on top of the goal
        dx_lookahead = lookahead_state.at(0) - x_current;
        dy_lookahead = lookahead_state.at(1) - y_current;
        heading_desired = atan2(dy_lookahead, dx_lookahead);
        //std::cout<< "Headings " << heading_current << " ,  "<< atan2(dy_current, dx_current) <<std::endl;
        heading_error = utility::angle_tools::minimum_difference(heading_current - heading_desired); // fed into a 1 DOF PID for heading                     
        containers_.heading_error = heading_error; 
        containers_.heading_ahrs = heading_current; 
        containers_.heading_desired = heading_desired; 
        currentTime_ = utility::time_tools::now();
        heading_signal = compute_signal(heading_error);                
        if (std::abs(heading_signal) > 1.0)
        {
          //std::cout << "clipping heading signal from " << heading_signal << " to ";
          heading_signal = copysign(1.0, heading_signal);
          //std::cout << heading_signal << std::endl;
        }
        //printf("heading error = %f   heading signal = %f\n", heading_error, heading_signal);
        
        // potentially reduce thrust signal due to too much heading error
        double base_surge_effort_fraction = containers_.LOS_surge_effort_fraction.to_double();
        double surge_effort_fraction_coefficient = 0.5; // [0, 1], reduces thrust
        double angle_from_projected_to_boat = atan2(projected_state.at(1) - y_current, projected_state.at(0) - x_current);
        double cross_product = cos(th_full)*sin(angle_from_projected_to_boat) - cos(angle_from_projected_to_boat)*sin(th_full);
        
        //printf("CONTROL: signed distance to line = %f\n", copysign(distance_from_ideal_line, cross_product));
        
        // if cross_product is positive, do not thrust if you have positive (th_full - heading_current)
        // if negative, do not thrust if you have negative (th_full - heading_current)
        if (distance_from_ideal_line > containers_.sufficientProximity.to_double())
        {
          if (cross_product < 0. && utility::angle_tools::minimum_difference(th_full - heading_current) < 0.)
          {
            surge_effort_fraction_coefficient = 0.; // do not thrust if you aren't at least parallel with the line          
          }
          if (cross_product > 0. && utility::angle_tools::minimum_difference(th_full - heading_current) > 0.)
          {
            surge_effort_fraction_coefficient = 0.;
          }
        }   
    
        double surge_effort_fraction = base_surge_effort_fraction*surge_effort_fraction_coefficient;
        std::pair<double, double> motor_signals = compute_motor_commands(surge_effort_fraction, heading_signal);        
        containers_.motor_signals.set(0, motor_signals.first);
        containers_.motor_signals.set(1, motor_signals.second);
      }
      else
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MAJOR,
          "threads::PID::run:" 
          " INFO: Boat within sufficient distance of destination, zeroing motors\n");
        //printf("Boat within %f meters of destination, dist: %f\n", containers_.sufficientProximity.to_double(),  containers_.dist_to_dest.to_double());
        //printf("Home: %f, %f; Source: %f,%f; Current: %f, %f; Destination: %f,%f\n", 
        //  containers_.self.agent.home[0], containers_.self.agent.home[1], containers_.self.agent.source[0], containers_.self.agent.source[1],
        //  containers_.eastingNorthingHeading[0], containers_.eastingNorthingHeading[1], containers_.self.agent.dest[0], containers_.self.agent.dest[1]);
        //printf("Source: %f, %f; Current: %f, %f; Desired: %f, %f\n", x_source, y_source, x_current, y_current, x_dest, y_dest);
        //containers.self.agent.source.set(0, containers.local_state[0]);
        //containers.self.agent.source.set(1, containers.local_state[1]);
        containers_.motor_signals.set(0, 0.0);
        containers_.motor_signals.set(1, 0.0);
        reset_error_integral();
        update_coefficients();     
      }
      
      // TODO - set up velocity profile along the path that lets the desired thrust be modulated for slow start up and drift down. Independent of time, only depends on location along line.
      
    }
    //Should we drive home if we are localised?
    /* else if (containers.connection_status == 0)
    {
        containers.motor_signals.set(0, 0.0);
        containers.motor_signals.set(1, 0.0);
        heading_PID.reset();        
    }*/
    //In teleop mode, get motor signals from thrust and heading fractions
    else if (containers_.teleop_status == 1)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "threads::PID::run:" 
        " INFO: Telop engaged\n");
      std::pair<double, double> motor_signals = compute_motor_commands(
       containers_.thrustFraction.to_double(), 
       containers_.headingFraction.to_double());

      containers_.motor_signals.set(0, motor_signals.first);
      containers_.motor_signals.set(1, motor_signals.second);
      //printf("In Teleop Mode. Motor signals are: %f, %f\n", motor_signals.first, motor_signals.second);

      reset_error_integral();
      update_coefficients();
    }
    else
    {
      //printf("Not moving because conditions are not met. Zeroing motors.\n");
      //printf("Autonomy: %d, localized: %d, teleop_status: %d\n", containers_.autonomy_enabled.to_integer(), containers_.localized.to_integer(), containers_.teleop_status.to_integer());
      containers_.motor_signals.set(0, 0.0);
      containers_.motor_signals.set(1, 0.0);

      reset_error_integral();
    }
}


double threads::PID::compute_signal(double error)
{
  dt_ = utility::time_tools::dt(currentTime_, prevTime_);

  errorDerivative_ = 0.0;

  if (dt_ > 0.0)
  {
    errorDerivative_ = (error - prevError_) / dt_;

    if (std::abs(kI_) > 0.0)
    {
      errorSum_ += error;
    }
  }

  prevError_ = error;

  return kP_ * error + kI_ * errorSum_ + kD_ * errorDerivative_;
}

void threads::PID::reset_error_integral()
{
  errorSum_ = 0.0;
}

void threads::PID::update_coefficients()
{
  kP_ = containers_.LOS_heading_PID[0];
  kI_ = containers_.LOS_heading_PID[1];
  kD_ = containers_.LOS_heading_PID[2];

}

std::pair<double, double> threads::PID::compute_motor_commands(double effort, double signal)
{
  double m0 = 0.0;
  double m1 = 0.0;
  double motor_overage0 = 0.0;
  double motor_overage1 = 0.0;
  m0 = effort - signal;
  m1 = effort + signal;
  
  //printf("Motor signals BEFORE saturation correction:  m0 = %f   m1 = %f\n", m0, m1);
  
  if (std::abs(m0) > 1.0)
  {
    motor_overage0 = copysign(std::abs(m0) - 1.0, m0);
  }
  if (std::abs(m1) > 1.0)
  {
    motor_overage1 = copysign(std::abs(m1) - 1.0, m1);
  }


  if (std::abs(motor_overage0) < std::abs(motor_overage1))
  {
    motor_overage0 = copysign(motor_overage1, m0);
  }
  else
  {
    motor_overage1 = copysign(motor_overage0, m1);
  }


  m0 -= motor_overage0;
  m1 -= motor_overage1;
  
  //printf("Motor signals AFTER saturation correction:  m0 = %f   m1 = %f\n", m0, m1);
  //printf("Design: equivalent effort fractions: thrust = %f   heading = %f\n", corrected_thrust_fraction, heading_fraction);
  
  std::pair<double, double> result = std::make_pair(m0, m1);
  return result;
}
