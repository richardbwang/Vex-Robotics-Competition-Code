#include "pid.h"
#include "vex.h"

#include <cmath>

// Constructor
PID::PID(float new_kp, float new_ki, float new_kd)
  : arrived(false),
    stable_time_duration(50), 
    check_time(0.0),
    first_time(true),
    proportional_range(50),
    integral_max(20) {
  // Set up the Coefficient.
  kp = new_kp;
  ki = new_ki;
  kd = new_kd;
  // Not arrived initially.
  arrived = false;

  index = 1;
}

void PID::SetCoefficient(float new_kp, float new_ki, float new_kd) {
  kp = new_kp;
  ki = new_ki;
  kd = new_kd;
}

void PID::SetTarget(float new_target) { 
  target = new_target;
}

void PID::SetErrorTolerance(float new_error_tolerance) { 
  error_tolerance = new_error_tolerance;
}

void PID::SetIntegralMax(float new_integral_max) { 
  integral_max = new_integral_max;
}

void PID::SetProportionalRange(float new_proportional_range) { 
  proportional_range = new_proportional_range;
}

void PID::ClearSumError() { 
  sum_error = 0;
}

void PID::SetDerivativeTolerance(float new_derivative_tolerance) { 
  derivative_tolerance = new_derivative_tolerance;
}

void PID::SetStableTimeDuration(
    float new_stable_time_duration_msec) { 
  stable_time_duration = new_stable_time_duration_msec;
}

bool PID::TargetArrived() { 
  return arrived;
}

float PID::GetI() { 
  return ki;
}

float PID::GetOutput() { 
  return output;
}

int PID::Sign(float number) {
  if (number > 0) {
    return 1;
  } else if (number < 0) {
    return -1;
  }
  return 0;
}

float PID::Update(float input) {
  // Calculate current error
  current_error = target - input; 
  if (first_time) {
    // First time is tricky.
    first_time = false;

    // Need to skip derivative. 
    previous_error = current_error;
    sum_error = 0;
    check_time = Brain.timer(msec);
  }

  // Calculate proportional
  proportional = kp * current_error;
  
  // Calculate derivative
  derivative = kd * (current_error - previous_error); 
  
  // Record current error
  previous_error = current_error; 
  
  if (fabs(proportional) >= proportional_range) { 
    // integral = 0 if proportinal > proportional_range
    sum_error = 0;
  } else { 
    sum_error += current_error;
    if (fabs(sum_error) * ki > integral_max) {
      // Limit integral to integral_max
      sum_error = Sign(sum_error) * integral_max / ki;
    }
  }

  if (Sign(sum_error) != Sign(current_error) || 
      (fabs(current_error) <= error_tolerance)) {
    // Clear integral if overshoot or current_error is very small. 
    // This is to stablize the movement.
    sum_error = 0;
  }



  // Calculate integral
  integral = ki * sum_error;
  
  if (fabs(current_error) <= error_tolerance && 
      fabs(derivative) <= derivative_tolerance) { 
    // Exit when staying in tolerated region and 
    // maintaining a low enough speed for enough time
    if (Brain.timer(msec) - check_time >= stable_time_duration) {
      arrived = true;
    }
  } else {
    check_time = Brain.timer(msec);
  }

  output = proportional + integral + derivative;
  if (fabs(current_error) < 2) {
    if (arrived) {
      //Brain.Screen.printAt(15, index * 18, "Mid: %.2f, %.2f, %.2f, %.2f, %.2f TURE", current_error, proportional, integral, derivative, output); 
    } else {
      //Brain.Screen.printAt(15, index * 18, "Mid: %.2f, %.2f, %.2f, %.2f, %.2f FALSE", current_error, proportional, integral, derivative, output);      
    }
    index++;
  }

  return output;
}

