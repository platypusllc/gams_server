#include "boat_containers.h"

Containers::Containers()
{
}

Containers::Containers(madara::knowledge::KnowledgeBase &kb_, int id_) 
: kb(kb_)
{ 
    std::stringstream prefix_;
    prefix_ << "agent." << id_ << ".";
    std::string prefix(prefix_.str());

    // Localization and control stuff
    motor_signals.set_name(prefix + "motorCommands", kb);
    std::vector<double> motor_signal_start = {0.0, 0.0};
    motor_signals.set(motor_signal_start);
    
    eastingNorthingHeading.set_name(prefix + "eastingNorthingBearing", kb);
    std::vector<double> eastingNorthingHeading_start = {0.0, 0.0, 0.0};
    eastingNorthingHeading.set(eastingNorthingHeading_start);
    
    location.set_name(prefix + "location", kb);
    std::vector<double> location_start = {0.0, 0.0, 0.0};
    location.set(location_start);
    
    local_state.set_name(prefix + "localState", kb);
    std::vector<double> local_state_start = {0., 0., 0., 0., 0., 0.};
    local_state.set(local_state_start);
    
    thrustFraction.set_name(prefix + "thrustFraction", kb);
    thrustFraction = 0.0;
    
    headingFraction.set_name(prefix + "bearingFraction", kb);
    headingFraction = 0.0;
    
    sufficientProximity.set_name(prefix + "sufficientProximity", kb);
    sufficientProximity = 3.0;
    
    dist_to_dest.set_name(prefix + "distToDest", kb);
    dist_to_dest = 0.0;
    
    gpsZone.set_name(prefix + "gps_zone", kb);

    northernHemisphere.set_name(prefix + "northern_hemisphere", kb);
    
    design_type.set_name(prefix + "design_type", kb);   
    
    LOS_lookahead.set_name(prefix + "LOS_lookahead", kb);
    LOS_lookahead = 3.0;
    
    LOS_surge_effort_fraction.set_name(prefix + "LOS_surge_effort_fraction", kb);
    LOS_surge_effort_fraction = 0.75;
    
    LOS_heading_PID.set_name(prefix + "LOS_heading_PID", kb);
    std::vector<double> LOS_heading_PID_start = {2.0, 0.0, 1.0};
    LOS_heading_PID.set(LOS_heading_PID_start);

    heading_error.set_name(prefix + "headingError", kb);
    heading_error = 0;
    heading_ahrs.set_name(prefix + "headingAHRS", kb);
    heading_ahrs = 0;
    heading_desired.set_name(prefix + "headingDesired", kb);
    heading_desired = 0;

    // Integer status stuff
    gps_init.set_name(prefix + "gpsInitialized", kb);
    gps_init = 0;

    compass_init.set_name(prefix + "compassInitialized", kb);
    compass_init = 0;

    localized.set_name(prefix + "localized", kb);
    localized = 0;
    
    heartbeat_gps.set_name(prefix + "gpsWatchdog", kb);
    heartbeat_gps = 1;
    
    heartbeat_compass.set_name(prefix + "compassWatchdog", kb);
    heartbeat_compass = 1;
    
    heartbeat_connectivity.set_name(prefix + "connectivityWatchdog", kb);
    heartbeat_connectivity = 1;
    
    heartbeat_operator.set_name(prefix + "operatorHeartbeat", kb);
    heartbeat_operator = 0;
    
    teleop_status.set_name(prefix + "teleopStatus", kb);
    teleop_status = 1; // = 1 --> start in teleop mode
    
    arm_signal.set_name(prefix + "armSignal", kb);
    arm_signal = 0;
    
    error_signal.set_name(prefix + "errorSignal", kb);
    error_signal = 0;

    autonomy_enabled.set_name(prefix + "autonomyEnabled", kb);
    autonomy_enabled = 0;
    
    
    // misc stuff
    battery_voltage.set_name(prefix + "batteryVoltage", kb);
    battery_voltage = 0.0;
    
    id.set_name(".id", kb);
    id = id_;
    
    self.init_vars(kb, id.to_integer());
    std::vector<double> dest_start = {0., 0.};
    self.agent.dest.set(dest_start);
    self.agent.home.set(dest_start);
    self.agent.source.set(dest_start);

    prefix_.str(std::string()); // clear the string stream, step 1
    prefix_.clear(); // step 2
    platform_status.init_vars(kb, "");
    platform_status.movement_available = 1;
}

Containers::~Containers() {  }
