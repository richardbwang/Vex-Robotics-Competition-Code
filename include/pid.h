#ifndef __PID__
#define __PID__

class PID {
 public:
  PID(double new_kp, double new_ki, double new_kd);
  
  // Adjust the kp, ki and kd if needed.
  void SetCoefficient(double new_kp, double new_ki, double new_kd);

  // Set the desired target value.
  void SetTarget(double new_target);

  // Set maximum allowed integral.
  void SetIntegralMax(double new_integral_max);

  // Set the allowed proportional range, ignore the integal
  // value if the proportional is out of this range.
  void SetIntegralRange(double new_integral_range);
  
  // Set error and derivative tolerance.
  void SetSmallBigErrorTolerance(double new_small_error_tolerance, double new_big_error_tolerance);
  void SetDerivativeTolerance(double new_derivative_tolerance);

  // Reset the sum_error for integral.
  void ClearSumError();
  
  // Set the stable time duration as the exit condition check.
  void SetSmallBigErrorDuration(double new_small_error_duration, double new_big_error_duration);
  
  // Whether arrived at the desired target within the tolerance
  // range.
  bool TargetArrived();
  
  double GetI();

  // Calculate the output value.
  double Update(double input);

  // Get the calculated output value.
  double GetOutput();
  
  // Util function to design sign (-1, 0, 1) of the number.
  int Sign(double number);
 
 protected:
  // Desired target value. 
  double target;

  // Exit condition arrived.
  bool arrived;

  // Minimum time duration to hold a stable state so 
  // we can exit the pid loop.
  double small_error_tolerance;
  double big_error_tolerance;
  double small_error_duration;
  double big_error_duration;

  // Time to check the exit the condition, used to make sure
  // the robot stay in tolerated region and 
  // maintain a low enough speed for enough time.
  double small_check_time, big_check_time;

  // Wether this is the first time to run PID.
  bool first_time;

  // PID Coefficient. 
  double kp, ki, kd;

  // Do not consider integral if the proportional value
  // is out of this range. 
  double integral_range;

  // Max integral value allowed.
  double integral_max;

  // Small tolerance to check both error and derivative.
  double derivative_tolerance, error_tolerance;
  // Temporary value for calculating proportional, 
  // integral, derivative.
  double current_error, previous_error, sum_error;

  // Calculated proportional, integral, derivative.
  double proportional, integral, derivative;

  // Calculated PID output. 
  double output;

};

#endif