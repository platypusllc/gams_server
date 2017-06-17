#include "datum.h"

Sensor::Sensor()
{
}

Sensor::~Sensor()
{
}

Datum::Datum()
{
}

Datum::Datum(SENSOR_TYPE type, SENSOR_CATEGORY category, std::vector<double> value, Eigen::MatrixXd covariance)
: type_(type), category_(category), value_(value), covariance_(covariance)
{
  unique_id_ = unique_id_count++;
  timestamp_ = std::chrono::high_resolution_clock::now();
  std::time_t now = std::chrono::high_resolution_clock::to_time_t(timestamp_);
  human_readable_time_ = std::ctime(&now);
  dimension = value.size();
  if (type == SENSOR_TYPE::GPS)
  {
    type_string_ = "GPS";
  }
  else if (type == SENSOR_TYPE::COMPASS)
  {
    type_string_ = "COMPASS";
  }
  else if (type == SENSOR_TYPE::GYRO)
  {
    type_string_ = "GYRO";
  }
  else if (type == SENSOR_TYPE::AHRS)
  {
    type_string_ = "AHRS";
  }
  else if (type == SENSOR_TYPE::ACCELEROMETER)
  {
    type_string_ = "ACCELEROMETER";
  }
  else if (type == SENSOR_TYPE::GPS_VELOCITY)
  {
    type_string_ = "GPS_VELOCITY";
  }
}

Datum::~Datum()
{
}

void Datum::set_location(GeographicLib::GeoCoords location) 
{
  location_ = location;
}

std::vector<double> Datum::value()
{
  return value_;
}

Eigen::MatrixXd Datum::covariance()
{
  return covariance_;
}

SENSOR_TYPE Datum::type()
{
  return type_;
}

long Datum::unique_id()
{
  return unique_id_;
}

std::chrono::time_point<std::chrono::high_resolution_clock> Datum::timestamp()
{
  return timestamp_;
}

std::string Datum::human_readable_time()
{
  return human_readable_time_;
}

std::string Datum::type_string()
{
  return type_string_;
}

long Datum::unique_id_count = -1;
