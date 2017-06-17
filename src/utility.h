#ifndef UTILITY_H
#define UTILITY_H

#include <string>
#include <vector>
#include <sstream>
#include <cmath>
#include <chrono>
#include <eigen3/Eigen/Core>
#include <random>
#include <functional>

#include "datum.h"

namespace utility 
{

  namespace string_tools
  {
    inline std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
        std::stringstream ss(s);
        std::string item;
        while (std::getline(ss, item, delim)) {
            elems.push_back(item);
        }
        return elems;
    }

    inline std::vector<std::string> split(const std::string &s, char delim) {
        std::vector<std::string> elems;
        split(s, delim, elems);
        return elems;
    }

    inline void remove_quotes(std::string &s) {
        if (s.substr(s.length()-1, 1).compare("\"") == 0) {
            s.erase(s.length()-1, 1);
        }
        if (s.substr(0, 1).compare("\"") == 0) {
            s.erase(0, 1);
        }    
    }
  }

  inline double round_to_decimal(double input, int decimal_places)
  {
    return round(input*pow(10.0, decimal_places))/pow(10.0, decimal_places);
  }
  
  namespace angle_tools
  {
    inline double wrap_to_pi(double angle)
    {    
      while (std::abs(angle) > M_PI)
      {
        angle -= copysign(2.0*M_PI, angle);
      }
      return angle;
    }
    
    inline double minimum_difference(double algebraic_difference) // return actual angular distance between two angles
    {
      if (std::abs(algebraic_difference - 2*M_PI) < std::abs(algebraic_difference))
      {
        return algebraic_difference - 2*M_PI;
      }
      if (std::abs(algebraic_difference + 2*M_PI) < std::abs(algebraic_difference))
      {
        return algebraic_difference + 2*M_PI;
      }      
      return algebraic_difference;
    }
  }
 

  namespace time_tools 
  {
    inline std::chrono::time_point<std::chrono::high_resolution_clock> now()
    {
      return std::chrono::high_resolution_clock::now();
    }
    
    inline double dt(std::chrono::time_point<std::chrono::high_resolution_clock> t0, 
                     std::chrono::time_point<std::chrono::high_resolution_clock> t1)
    {
      std::chrono::duration<double> dt_ = std::chrono::duration_cast< std::chrono::duration<double> >(t1-t0);
      return dt_.count();
    }
  }


  namespace type_conversion
  {
    inline Eigen::MatrixXd std_vector_to_MatrixXd(std::vector<double> input)
    {
      int dim = input.size();
      Eigen::MatrixXd output(dim, 1);
      for (int i; i < dim; i++)
      {
        output(i, 0) = input.at(i);
      }
    }
  }

  namespace units_conversion
  {
  }

  namespace random_numbers 
  {
    static std::random_device rd;
    static std::mt19937 generator(rd());
    static std::uniform_real_distribution<> distribution(0., 1.0);

    inline double rand()
    {
      return distribution(generator);
    }

    inline double rand(double min, double max)
    {
      return distribution(generator)*(max - min) + min;
    }

    inline std::vector<double> rand(int size)
    {
      std::vector<double> result;
      for (int i = 0; i < size; i++)
      {
        result.push_back(distribution(generator));
      }
      return result;
    }

    inline std::vector<double> rand(int size, double min, double max)
    {
      std::vector<double> result;
      for (int i = 0; i < size; i++)
      {
        result.push_back(distribution(generator)*(max - min) + min);
      }
      return result;    
    }
  }

}

#endif // UTILITY_H
