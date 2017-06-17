#ifndef _SENSOR_H_
#define _SENSOR_H_

#include <string>
#include <vector>
#include <chrono>
#include <ctime>
#include <stdio.h>
#include <GeographicLib/GeoCoords.hpp>
#include <eigen3/Eigen/Core>

enum class SENSOR_CATEGORY
{
  LOCALIZATION,
  ENVIRONMENTAL
};

enum class SENSOR_TYPE
{
  GPS,
  COMPASS,
  GYRO,
  AHRS, // combined compass and gyro in one datum
  ACCELEROMETER,
  GPS_VELOCITY,
  EC,
  TEMP,
  DO,
  PH,
  DEPTH,
  FLOW,
  WIFI_STRENGTH  
};


class Sensor
{
public:
  Sensor();
  ~Sensor();

protected:
  std::string name;
  SENSOR_TYPE type;
  SENSOR_CATEGORY category;
  double Hz;

private:
};


class Datum
{
public:
  Datum();
  Datum(SENSOR_TYPE type, SENSOR_CATEGORY category, std::vector<double> value, Eigen::MatrixXd covariance);
  ~Datum();
  void set_location(GeographicLib::GeoCoords location);
  std::vector<double> value();
  Eigen::MatrixXd covariance();
  SENSOR_TYPE type();
  long unique_id();
  std::chrono::time_point<std::chrono::high_resolution_clock> timestamp();
  std::string human_readable_time();
  std::string type_string();

  static long unique_id_count;

private:
  SENSOR_TYPE type_;
  SENSOR_CATEGORY category_;
  long unique_id_;
  std::chrono::time_point<std::chrono::high_resolution_clock> timestamp_;
  std::string type_string_;
  std::string human_readable_time_;
  int dimension;
  std::vector<double> value_;
  Eigen::MatrixXd covariance_;
  GeographicLib::GeoCoords location_;
};

class DatumComparison // used for the localization sensor update priority queue
{
public:
  bool operator() (Datum & a, Datum & b)
  {
    return a.timestamp() > b.timestamp();
  }
};



#endif // _SENSOR_H_
