
#ifndef   _THREAD_PID_H_
#define   _THREAD_PID_H_

#include <string>
#include <stdio.h>
#include <iostream>
#include <memory>
#include <cmath>
#include <utility>
#include <algorithm>

#include "madara/threads/BaseThread.h"

#include "../boat_containers.h"
#include "../utility.h"

typedef std::chrono::time_point<std::chrono::high_resolution_clock> precise_time_t;

namespace threads
{
  /**
  * A custom thread generated by gpc.pl
  **/
  class PID : public madara::threads::BaseThread
  {
  public:
    /**
     * Default constructor
     **/
    PID (Containers & containers);
    
    /**
     * Destructor
     **/
    virtual ~PID ();
    
    /**
      * Initializes thread with MADARA context
      * @param   context   context for querying current program state
      **/
    virtual void init (madara::knowledge::KnowledgeBase & knowledge);

    /**
      * Executes the main thread logic
      **/
    virtual void run (void);

  private:
    /// data plane if we want to access the knowledge base
    madara::knowledge::KnowledgeBase data_;

    Containers containers_;
    
    // time variables
    precise_time_t currentTime_;
    precise_time_t prevTime_;
    double dt_;

    // error terms
    double prevError_;
    double errorDerivative_;
    double errorSum_;

    // pid coefficients
    double kP_, kI_, kD_;

    /**
      * Computes the control signal from the current error using PID
      **/
    double compute_signal (double error);

    /**
      * Clears the integral of the error
      **/
    void reset_error_integral (void);

    /**
      * Update pid coefficients from knowledge base
      **/
    void update_coefficients (void);

    /**
      * Computes motor commands from effort fraction and signal
      **/
    std::pair<double, double> compute_motor_commands (double effort, double signal);

  };
} // end namespace threads

#endif // _THREAD_PID_H_
