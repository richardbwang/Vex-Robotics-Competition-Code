#ifndef __PID__
#define __PID__

class PID {
 public:
  PID(float new_kp, float new_ki, float new_kd);
  
  // Adjust the kp, ki and kd if needed.
  void SetCoefficient(float new_kp, float new_ki, float new_kd);

  // Set the desired target value.
  void SetTarget(float new_target);

  // Set maximum allowed integral.
  void SetIntegralMax(float new_integral_max);

  // Set the allowed proportional range, ignore the integal
  // value if the proportional is out of this range.
  void SetProportionalRange(float new_proportional_range);
  
  // Set error and derivative tolerance.
  void SetErrorTolerance(float new_error_tolerance);
  void SetDerivativeTolerance(float new_derivative_tolerance);

  // Reset the sum_error for integral.
  void ClearSumError();
  
  // Set the stable time duration as the exit condition check.
  void SetStableTimeDuration(
      float new_stable_time_duration_msec);
  
  // Whether arrived at the desired target within the tolerance
  // range.
  bool TargetArrived();
  
  float GetI();

  // Calculate the output value.
  float Update(float input);

  // Get the calculated output value.
  float GetOutput();
  
  // Util function to design sign (-1, 0, 1) of the number.
  int Sign(float number);
 
 protected:
  // Desired target value. 
  float target;

  // Exit condition arrived.
  bool arrived;

  // Minimum time duration to hold a stable state so 
  // we can exit the pid loop.
  float stable_time_duration;

  // Time to check the exit the condition, used to make sure
  // the robot stay in tolerated region and 
  // maintain a low enough speed for enough time.
  float check_time;

  // Wether this is the first time to run PID.
  bool first_time;

  // PID Coefficient. 
  float kp, ki, kd;

  // Do not consider integral if the proportional value
  // is out of this range. 
  float proportional_range;

  // Max integral value allowed.
  float integral_max;

  // Small tolerance to check both error and derivative.
  float derivative_tolerance, error_tolerance;
  // Temporary value for calculating proportional, 
  // integral, derivative.
  float current_error, previous_error, sum_error;

  // Calculated proportional, integral, derivative.
  float proportional, integral, derivative;

  // Calculated PID output. 
  float output;

  int index;
};

#endif