#include "vex.h"
#include "motor-control.h"
#include <ctime>

//================================Chassis
//Control=========================================//
void ChassisControl(float lPower, float rPower) {
  left_chassis1.spin(fwd, 0.128 * lPower, voltageUnits::volt);
  left_chassis2.spin(fwd, 0.128 * lPower, voltageUnits::volt);

  right_chassis1.spin(fwd, 0.128 * rPower, voltageUnits::volt);
  right_chassis2.spin(fwd, 0.128 * rPower, voltageUnits::volt);
}

void intake(float lPower, float rPower) {
  leftintake_motor.spin(fwd, 0.128 * lPower, voltageUnits::volt);
  righttintake_motor.spin(fwd, 0.128 * rPower, voltageUnits::volt);
}
void intake_rpm(float lPower, float rPower) {

  leftintake_motor.spin(fwd, 6 * lPower, rpm);
  righttintake_motor.spin(fwd, 6 * rPower, rpm);
}

void fw_adjustable_rpm(float lRpm, float rRpm) { 
  double total_wait_time = 0;
  double error_l = 0;
  double error_r = 0;
  
  // Find the difference.
  leftintake_motor.spin(fwd, 6 * lRpm, velocityUnits::rpm);
  righttintake_motor.spin(fwd, 6 * rRpm, velocityUnits::rpm);
    
  error_l = leftintake_motor.velocity(rpm) - 6 * lRpm;
  error_r = righttintake_motor.velocity(rpm) - 6 * rRpm;

  // Adjust the rmp if either side is slow and wait for at most 0.2 second.
  while ((error_l > 10 || error_r > 10) && total_wait_time < 200) {
    // Tune the difference with 1.5 multiplier.
    leftintake_motor.spin(fwd, 6 * lRpm - 1.5 * error_l, velocityUnits::rpm);
    righttintake_motor.spin(fwd, 6 * rRpm - 1.5 * error_r, velocityUnits::rpm);
    
    // Stablize the motor a bit.
    wait(50, msec);

    // Increase the total wait time.
    total_wait_time += 50;
    
    // Check the difference again.
    error_l = leftintake_motor.velocity(rpm) - 6 * lRpm;
    error_r = righttintake_motor.velocity(rpm) - 6 * rRpm;    
  }
}

void intake_pid(float lPower, float rPower) {
  double kp = 0.6;
  double ki = 0.6;
  double kd = 0.7;
  //===============================
  double error_l = 0.0;
  double error_r = 0.0;
  double error_sum_l = 0.0;
  double error_sum_r = 0.0;
  double last_error_l = 0.0;
  double last_error_r = 0.0;
  //===============================
  double d_l = 0.0;
  double d_r = 0.0;
  error_l = leftintake_motor.velocity(rpm) - 6 * lPower;
  error_r = righttintake_motor.velocity(rpm) - 6 * rPower;

  error_sum_l = error_sum_l + error_l;
  error_sum_r = error_sum_r + error_r;
  d_l = kp * error_l + ki * error_sum_l + kd * (error_l - last_error_l);
  d_r = kp * error_r + ki * error_sum_r + kd * (error_r - last_error_r);
  last_error_l = error_l;
  last_error_r = error_r;

  leftintake_motor.spin(fwd, 6 * lPower - d_l, rpm);
  righttintake_motor.spin(fwd, 6 * rPower - d_r, rpm);
}

void Stop(brakeType type) {
  left_chassis1.stop(type);
  left_chassis2.stop(type);

  right_chassis1.stop(type);
  right_chassis2.stop(type);
}
void liftDown(float power) {
  lift_down.spin(fwd, 0.128 * power, voltageUnits::volt);
}

void xi(float power) { xi_motor.spin(fwd, 0.128 * power, voltageUnits::volt); }