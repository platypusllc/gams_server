#ifndef CONTAINERS_H
#define CONTAINERS_H

#include <string>
#include <sstream>

#include "madara/knowledge/KnowledgeBase.h"
#include "madara/knowledge/containers/NativeDoubleVector.h"
#include "madara/knowledge/containers/Double.h"
#include "madara/knowledge/containers/Integer.h"
#include "madara/knowledge/containers/String.h"
#include "gams/variables/Self.h"
#include "gams/variables/PlatformStatus.h"

namespace containers = madara::knowledge::containers;

class Containers {
public:
    Containers();
    Containers(madara::knowledge::KnowledgeBase &kb_, int id_);
    ~Containers();
    
    // Localization and control stuff
    containers::NativeDoubleVector motor_signals;
    containers::NativeDoubleVector eastingNorthingHeading;
    containers::NativeDoubleVector location; // lat, long, elevation
    containers::NativeDoubleVector local_state; // local x, local y, theta, xdot, ydot, thetadot
    containers::Double thrustFraction;
    containers::Double headingFraction;
    containers::Double sufficientProximity;
    containers::Double dist_to_dest;
    containers::Integer gpsZone;
    containers::Integer northernHemisphere;
    containers::Integer design_type;
    containers::Double LOS_lookahead;
    containers::Double LOS_surge_effort_fraction; // desired maximum surge effort = [0., 1.]
    containers::NativeDoubleVector LOS_heading_PID;
    containers::Double heading_error;
    containers::Double heading_ahrs;
    containers::Double heading_desired;
    
    // Integer status stuff
    containers::Integer gps_init;
    containers::Integer compass_init;
    containers::Integer localized;
    containers::Integer heartbeat_gps;
    containers::Integer heartbeat_compass;
    containers::Integer heartbeat_connectivity;
    containers::Integer heartbeat_operator;
    containers::Integer teleop_status;
    containers::Integer arm_signal;
    containers::Integer error_signal;
    containers::Integer autonomy_enabled;
    containers::Integer test_flag;
    
    // misc. stuff
    containers::Integer id;
    containers::Double battery_voltage;
    gams::variables::Self self;
    gams::variables::PlatformStatus platform_status;

private:
    madara::knowledge::KnowledgeBase kb;
};

#endif  // CONTAINERS_H