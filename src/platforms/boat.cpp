
#include "madara/knowledge/containers/NativeDoubleVector.h"
#include "boat.h"

gams::pose::CartesianFrame  platforms::boat::cartesian_frame;   
gams::pose::GPSFrame  platforms::boat::gps_frame;         
        
 
// factory class for creating a boat 
gams::platforms::BasePlatform *
platforms::boatFactory::create (
        const madara::knowledge::KnowledgeMap & args,
        madara::knowledge::KnowledgeBase * knowledge,
        gams::variables::Sensors * sensors,
        gams::variables::Platforms * platforms,
        gams::variables::Self * self)
{
  return new boat (knowledge, sensors, self);
}
        
// Constructor
platforms::boat::boat (
  madara::knowledge::KnowledgeBase * knowledge,
  gams::variables::Sensors * sensors,
  gams::variables::Self * self)
: gams::platforms::BasePlatform (knowledge, sensors, self)
{
  // as an example of what to do here, create a coverage sensor
  if (knowledge && sensors)
  {
    // set the data plane for the threader
    threader_.set_data_plane (*knowledge);
  
    // create a coverage sensor
    gams::variables::Sensors::iterator it = sensors->find ("coverage");
    if (it == sensors->end ()) // create coverage sensor
    {
      // get origin
      gams::utility::GPSPosition origin;
      madara::knowledge::containers::NativeDoubleArray origin_container;
      origin_container.set_name ("sensor.coverage.origin", *knowledge, 3);
      origin.from_container (origin_container);

      // establish sensor
      gams::variables::Sensor* coverage_sensor =
        new gams::variables::Sensor ("coverage", knowledge, 2.5, origin);
      (*sensors)["coverage"] = coverage_sensor;
    }
    (*sensors_)["coverage"] = (*sensors)["coverage"];
    status_.init_vars (*knowledge, "");
    
    // create threads
    // end create threads
    
    
    /**
    * the following should be set when movement is available in your
    * platform. If on construction, movement should be possible, then
    * feel free to keep this uncommented. Otherwise, set it somewhere else
    * in analyze or somewhere else when appropriate to enable movement.
    * If you never enable movement_available, movement based algorithms are
    * unlikely to ever move with your platform.
    **/
    status_.movement_available = 1;
  }
}


// Destructor
platforms::boat::~boat ()
{
  threader_.terminate ();
  threader_.wait ();
}


// Polls the sensor environment for useful information. Required.
int platforms::boat::sense (void)
{
  return gams::platforms::PLATFORM_OK;
}


// Analyzes platform information. Required.
int
platforms::boat::analyze (void)
{
  containers_.heartbeat_connectivity = 1;
  printf("setting connectivity heartbeat\n");
  return gams::platforms::PLATFORM_OK;
}


// Gets the name of the platform. Required.
std::string
platforms::boat::get_name () const
{
  return "boat";
}


// Gets the unique identifier of the platform.
std::string
platforms::boat::get_id () const
{
  return "boat";
}


// Gets the position accuracy in meters. Optional.
double
platforms::boat::get_accuracy (void) const
{
  // will depend on your localization capabilities for robotics
  return containers_.sufficientProximity.to_double();
}

// Gets Location of platform, within its parent frame. Optional.
gams::pose::Position
platforms::boat::get_location () const
{
  gams::pose::Position result;
  
  return result;
}


// Gets Rotation of platform, within its parent frame. Optional.
gams::pose::Orientation
platforms::boat::get_orientation () const
{
  gams::pose::Orientation result(0.0, 0.0, containers_.eastingNorthingHeading[2]);
  
  return result;
}


// Gets sensor radius. Optional.
double
platforms::boat::get_min_sensor_range () const
{
  // should be in square meters
  return 0.0;
}

// Gets move speed. Optional.
double
platforms::boat::get_move_speed () const
{
  // should be in meters/s
  // not really m/s in this case but roughly indicates speed
  return containers_.thrustFraction.to_double();
}

// Instructs the agent to return home. Optional.
int
platforms::boat::home (void)
{
  /**
   * Movement functions work in a polling manner. In a real implementation,
   * we would want to check a finite state machine, external thread or
   * platform status to determine what to return. For now, we will simply
   * return that we are in the process of moving to the final pose.
   **/

  self_->agent.dest.set(0, self_->agent.dest[0]);
  self_->agent.dest.set(1, self_->agent.dest[1]);
  self_->agent.source.set(0, containers_.eastingNorthingHeading[0]);
  self_->agent.source.set(1, containers_.eastingNorthingHeading[1]);

  return gams::platforms::PLATFORM_IN_PROGRESS;
}


// Instructs the agent to land. Optional.
int
platforms::boat::land (void)
{
  /**
   * Movement functions work in a polling manner. In a real implementation,
   * we would want to check a finite state machine, external thread or
   * platform status to determine what to return. For now, we will simply
   * return that we are in the process of moving to the final pose.
   **/
  return gams::platforms::PLATFORM_IN_PROGRESS;
}


// Moves the platform to a location. Optional.
int
platforms::boat::move (
  const gams::pose::Position & location,
  double epsilon)
{
  /**
   * Movement functions work in a polling manner. In a real implementation,
   * we would want to check a finite state machine, external thread or
   * platform status to determine what to return. For now, we will simply
   * return that we are in the process of moving to the final pose.
   **/

  double lat = location.lat();
  double lng = location.lng();

  printf("platform.move():  lat = %f, lng = %f\n", lat, lng);

  GeographicLib::GeoCoords coord(lat, lng, containers_.gpsZone.to_integer());
  double easting = coord.Easting();
  double northing = coord.Northing();   

  if (easting != self_->agent.dest[0] || northing != self_->agent.dest[1])
  {
   // update source to prior destination
   self_->agent.source.set(0, self_->agent.dest[0]);
   self_->agent.source.set(1, self_->agent.dest[1]);
   
   // new destination
   self_->agent.dest.set(0, easting);
   self_->agent.dest.set(1, northing);    
  }

  return gams::platforms::PLATFORM_MOVING;
}


// Rotates the platform to match a given angle. Optional.
int
platforms::boat::rotate (
  const gams::pose::Orientation & target,
  double epsilon)
{
  /**
   * Movement functions work in a polling manner. In a real implementation,
   * we would want to check a finite state machine, external thread or
   * platform status to determine what to return. For now, we will simply
   * return that we are in the process of moving to the final pose.
   **/
  return gams::platforms::PLATFORM_MOVING;
}


// Moves the platform to a pose (location and rotation). Optional.
int
platforms::boat::pose (const gams::pose::Pose & target,
  double loc_epsilon, double rot_epsilon)
{
  /**
   * Movement functions work in a polling manner. In a real implementation,
   * we would want to check a finite state machine, external thread or
   * platform status to determine what to return. For now, we will simply
   * return that we are in the process of moving to the final pose.
   **/
  return gams::platforms::PLATFORM_MOVING;
}

// Pauses movement, keeps source and dest at current values. Optional.
void
platforms::boat::pause_move (void)
{
}


// Set move speed. Optional.
void
platforms::boat::set_move_speed (const double& speed)
{
}


// Stops movement, resetting source and dest to current location. Optional.
void
platforms::boat::stop_move (void)
{
}

// Instructs the agent to take off. Optional.
int
platforms::boat::takeoff (void)
{
  return gams::platforms::PLATFORM_OK;
}

const gams::pose::ReferenceFrame &
platforms::boat::get_frame (void) const
{
  return gps_frame;
}

// Sets platform containers
void platforms::boat::set_containers(Containers & containers)
{
  containers_ = containers;
}
