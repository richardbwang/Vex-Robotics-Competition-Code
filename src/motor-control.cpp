#include "vex.h"
#include "utils.h"
#include <ctime>
#include <cmath>

#include "motor-control.h"
  
//================================Chassis Control=========================================//
void ChassisControl(float lPower, float rPower) {
  left_chassis1.spin(fwd, 0.128 * lPower, voltageUnits::volt); // 12.8v battery
  left_chassis2.spin(fwd, 0.128 * lPower, voltageUnits::volt);  
  right_chassis1.spin(fwd, 0.128 * rPower, voltageUnits::volt);
  right_chassis2.spin(fwd, 0.128 * rPower, voltageUnits::volt); 
}

void ChassisControlPercent(float lVelocity, float rVelocity) {
  left_chassis1.spin(forward, lVelocity, percent);
  left_chassis2.spin(forward, lVelocity, percent);
  right_chassis1.spin(forward, rVelocity, percent);
  right_chassis2.spin(forward, rVelocity, percent);
}

void ResetChassis() {
  left_chassis1.setPosition(0, degrees);
  left_chassis2.setPosition(0, degrees);
  right_chassis1.setPosition(0, degrees);
  right_chassis2.setPosition(0, degrees);
}

double GetLeftRotationDegree() {
  return (left_chassis1.position(degrees) + left_chassis2.position(degrees)) / 2;
}

double GetRightRotationDegree() {
  return (right_chassis1.position(degrees) + right_chassis2.position(degrees)) / 2;
}



//================================Chassis Movement=========================================//
double TurnForAngle(float turnAngle, float timeLimit_msec) {
  // Predefined paramaters, DO NOT CHANGE!!!
  std::string result;
  const float kP = 0.65, kI = 0.0004, kD = 45;

  float prevTime = 0, deltaTime = 0, currentTime = 0;
  float integral = 0, error = 0, derivative = 0, prevError = 0;
  float currentAngle = 0;

  // Condition variable to control the loop.
  bool notDone = true;

  // Reset heading and timer.
  InertialA.setHeading(0, degrees);
  float startAngle = InertialA.heading(degrees);
  float startTime = Brain.timer(msec);
  
  Graph graph = Graph(currentAngle, 250, turnAngle);

  // Start to turn.
  int i = 0;
  int j = 0;
  do {
    currentTime = Brain.timer(msec);
    deltaTime = prevTime == 0 ? 0 : currentTime - prevTime;
    prevTime = currentTime;

    currentAngle = InertialA.heading(degrees) - startAngle;

    // Try to turn the minimum degree, and also tolerate some 
    // noise at the beginning.
    if (currentAngle > 180) {
      currentAngle = currentAngle - 360;
    } else if (currentAngle < -180) {
      currentAngle = 360 + currentAngle;
    }

    error = turnAngle - currentAngle;
    
    // graph.updateData(currentAngle);
    // graph.drawGraph();

    // Always choose the minimum degree to turn.
    if (error > 180) {
      error = error - 360;
    } else if (error < -180) {
      error = 360 + error;
    }
    
    derivative = deltaTime == 0 ? 0 : (error - prevError) / deltaTime;
    
    prevError = error;
    if (error * turnAngle < 0) {
      integral /= 2;
    } else {
      integral += error * deltaTime;
    }

    float driveSpeed = kP * error + kI * integral + kD * derivative;
    if ((fabs(error) < fabs(turnAngle * 0.005)) || (Brain.timer(msec) - startTime >= timeLimit_msec)) {
      notDone = false;
      Stop(hold);
      //Brain.Screen.printAt(10, 10, "FC:%.2f E:%.2f I:%.2f D:%.2f S:%.2f", 
      //                              currentAngle, error, integral, derivative, driveSpeed);
    } else {
      ChassisControlPercent(-driveSpeed, driveSpeed);
    }

    /*
    char temp_result[200];
    sprintf(temp_result, "Round %d, ERR:%.3f PERR:%.3f INT:%.3f DER:%.3f SPD:%.3f \n", 
            i++, error, prevError, integral, derivative, driveSpeed);
    result.append(temp_result);
    */
    // Wait a bit before checking again.
    // Brain.SDcard.appendtextfile("test.txt", "");
    if (i > 13 && i % 2 == 0) {
      // Brain.Screen.printAt(15, 15 + j * 15, "C:%.2f E:%.2f I:%.2f D:%.2f S:%.2f", 
      //                                      currentAngle, error, integral, derivative, driveSpeed);
      j += 1;                                  
    }
    i += 1;
    wait(15, msec);
  } while (notDone);

  // return Brain.timer(msec) - startTime;
  return InertialA.heading(degrees);
}

void DriveFor(float distance_in, float time_limit_msec) {
  // Current robot setup, 3.25 inch wheel, 60/36 gear ratio.
  const float wheel_distance_in = (60.0 / 36.0) * 3.25 * 3.14159;
  // Tuned parameters, DO NOT CHANGE!!!
  const float kp = 3, ki = 0.0007, kd = 500;

  double drive_direction = distance_in > 0 ? 1 : -1;
  distance_in = distance_in * drive_direction;

  // Reset the chassis.
  ResetChassis();
  float start_time = Brain.timer(msec);
  float prev_time = start_time, delta_time, current_time;
  float l_integral = 0, r_integral = 0;
  float l_prev_error = 0, r_prev_error = 0;
  float l_error = 0, r_error = 0;
  float l_derivative = 0, r_derivative = 0;

  bool not_done = true;

  float current_left = 0, current_right = 0;
  //= (GetLeftRotationDegree() / 360.0) * wheel_distance_in;
  //float currentRight = (GetRightRotationDegree() / 360.0) * wheel_distance_in;
  // Graph graph = Graph(currentLeft, 250, goalDistance);
  int j = 0;
  do {
    current_time = Brain.timer(msec);
    delta_time = prev_time == 0 ? 0 : current_time - prev_time;
    prev_time = current_time;

    current_left = fabs((GetLeftRotationDegree() / 360.0) * wheel_distance_in);
    current_right = fabs((GetRightRotationDegree() / 360.0) * wheel_distance_in);

    // graph.updateData(currentLeft);
    // graph.drawGraph();

    l_error = distance_in - current_left;
    r_error = distance_in - current_right;

    //std::cout << LError << std::endl;

    l_derivative = delta_time == 0 ? 0: (l_error - l_prev_error) / delta_time;
    r_derivative = delta_time == 0 ? 0: (r_error - r_prev_error) / delta_time;
    l_prev_error = l_error;
    r_prev_error = r_error;

    // Ignore the integral if the error is too large.
    if (fabs(l_error) < 10 || fabs(l_error) < 10) {
      l_integral += l_error * delta_time;
      r_integral += r_error * delta_time;
    } else {
      l_integral = 0;
      r_integral = 0;
    }

    float left_speed = kp * l_error + ki * l_integral + kd * l_derivative;
    float right_speed = kp * r_error + ki * r_integral + kd * r_derivative;
    /*
    if (maxSpeed != 100) {
      if (fabs(leftDriveSpeed) > maxSpeed) {
        leftDriveSpeed = maxSpeed * (int(0) < leftDriveSpeed) - (leftDriveSpeed < int(0));
      }
      if (fabs(rightDriveSpeed) > maxSpeed) {
        rightDriveSpeed = maxSpeed * (int(0) < rightDriveSpeed) - (rightDriveSpeed < int(0));
      }
    }*/
    //std::cout << leftDriveSpeed << std::endl;
    
    if (l_error < 1) {
      left_speed = 0;
      right_speed = 0;
      Brain.Screen.printAt(15, j * 15, "L:%.2f E:%.2f I:%.2f D:%.2f S:%.2f", 
                        current_left, l_error, l_integral, l_derivative, left_speed);
      Brain.Screen.printAt(15, 15 + j * 15, "R:%.2f E:%.2f I:%.2f D:%.2f S:%.2f", 
                        current_right, r_error, r_integral, r_derivative, right_speed);
      j += 2;
      
    }
    ChassisControlPercent(left_speed * drive_direction, right_speed * drive_direction);

    if (((fabs(l_error) < .6) && (fabs(r_error) < .6)) || 
        (Brain.timer(msec) - start_time > time_limit_msec)) {
      not_done = false;
      Brain.Screen.printAt(10, 200,  "D1:%.2f, %.2f\n", current_left, current_right);
      Stop(vex::brakeType::brake);
      ChassisControlPercent(0, 0);
    }
    wait(20, msec);
  } while (not_done);
}


//=================================Flying Wheels==========================================//
void fw(float lPower, float rPower) {
  // Control using power.
  left_fw_motor.spin(fwd, 0.128 * lPower, voltageUnits::volt);
  right_fw_motor.spin(fwd, 0.128 * rPower, voltageUnits::volt);
}

void fw_rpm(double lRpm, double rRpm) { 
  // Control using rpm, multiply by 6.
  left_fw_motor.spin(fwd, 6 * lRpm,  velocityUnits::rpm);
  right_fw_motor.spin(fwd, 6* rRpm, velocityUnits::rpm);
}

int xxx = 1;
void fw_pid_rpm(double lRpm, double rRpm) { 
  /*
  double kp= 0.6;
  double ki= 0.6;
  double kd= 0.7;
  */
  double kp= 0.4;
  double ki= 0.4;
  double kd= 0.5;

  //===============================
  double error_l = 0;
  double error_r = 0;
  double error_sum_l = 0;
  double error_sum_r = 0;
  double last_error_l = 0;
  double last_error_r = 0;
  //===============================
  double d_l;
  double d_r;

  error_l = left_fw_motor.velocity(rpm) - lRpm;
  error_r = right_fw_motor.velocity(rpm) - rRpm;

  error_sum_l = error_sum_l + error_l;
  error_sum_r = error_sum_r + error_r;

  d_l = kp * error_l + ki * error_sum_l + kd * (error_l - last_error_l);
  d_r = kp * error_r + ki * error_sum_r + kd * (error_r - last_error_r);
  
  last_error_l = error_l;
  last_error_r = error_r;

  left_fw_motor.spin(fwd, lRpm - d_l, rpm);
  right_fw_motor.spin(fwd, rRpm - d_r, rpm);
  /*
  if (xxx > 15) {
    Brain.Screen.clearScreen(black);
    xxx = 1;
  }
  double c_l_rpm = left_fw_motor.velocity(rpm);
  double c_r_rpm = right_fw_motor.velocity(rpm);
  Brain.Screen.printAt(15, 15 * (xxx++), "L:%.2f %.2f %.2f| R:%.2f %.2f %.2f", 
                      error_l, lRpm - d_l,  c_l_rpm, error_r, rRpm - d_r, c_r_rpm);
  */
}


void FwSpinFor(double l_rpm, double r_rpm, int time_limt_msec) {
  // Fixed PID parameters for team 99116X. DO NOT CHANGE!!!!
  const double l_kp = 1.9, l_ki = 0.0, l_kd = 0;
  const double r_kp = 1.9, r_ki = 0.0, r_kd = 0;

  float start_time = Brain.timer(msec);

  // PID variables for left and right flying wheels.
  double error_l = 0;
  double last_error_l = 0;
  double error_sum_l = 0;
  double d_l;
  
  double error_r = 0;
  double last_error_r = 0;
  double error_sum_r = 0;
  double d_r;  
  //==============================================================
  // Set up the motor speed for both left and righ flying wheels.
  // int i = 0;
  bool not_done = true;
  double max_rmp = l_rpm > r_rpm ? l_rpm : r_rpm;
  Graph graph = Graph(0, 250, max_rmp / 2);

  do {
      double c_l_rpm = left_fw_motor.velocity(rpm);
      double c_r_rpm = right_fw_motor.velocity(rpm);
      graph.updateData(c_l_rpm / 2, 0);
      graph.updateData(c_r_rpm / 2, 1);
     
      error_l = l_rpm - c_l_rpm;
      error_r = r_rpm - c_r_rpm;
      if (Brain.timer(msec) - start_time < time_limt_msec &&
          (fabs(error_l) > l_rpm * 0.03 || fabs(error_r) > l_rpm * 0.03)) {

        error_sum_l += error_l;
        error_sum_r += error_r;
        /*
        if (last_error_l * error_l < 0) {
          error_sum_l = 0;
        } else {
           error_sum_l += error_l;
        }
        if (last_error_r * error_r < 0) {
          error_sum_r = 0;
        } else {
           error_sum_r += error_r;
        }*/
        
        d_l = l_kp * error_l + l_ki * error_sum_l + l_kd * (error_l - last_error_l);
        d_r = r_kp * error_r + r_ki * error_sum_r + r_kd * (error_r - last_error_r);

        // Use percent instead of rpm since rpm requies longer time to tune.
        left_fw_motor.spin(fwd, l_rpm + d_l, rpm);
        right_fw_motor.spin(fwd, r_rpm + d_r, rpm);

        last_error_l = error_l;
        last_error_r = error_r;
        wait(20, msec);
      } else {
        not_done = false;
      }
      /*
      Brain.Screen.setPenColor(red);
      Brain.Screen.drawLine(i + 20, 220-left_fw_motor.velocity(rpm) / 3,i + 20, 220 - last_s_l / 3);

      Brain.Screen.setPenColor(green);
      Brain.Screen.drawLine(i + 20, 30 + right_fw_motor.velocity(rpm) / 3, i + 20, 30 + last_s_r / 3);
      */
  } while (not_done); 

  if (xxx > 15) {
    Brain.Screen.clearScreen(black);  
    xxx = 1;
  }
  double c_l_rpm = left_fw_motor.velocity(rpm);
  double c_r_rpm = right_fw_motor.velocity(rpm);
  Brain.Screen.printAt(15, 15 * (xxx++), "L:%.2f %.2f %.2f| R:%.2f %.2f %.2f", 
                        error_l, l_rpm + d_l, c_l_rpm, error_r, r_rpm - d_r, c_r_rpm);

  // graph.drawGraph();
  // Brain.Screen.printAt(40, 210, "Total time : %.3f", Brain.timer(msec) - start_time);
}


void fw_pid_rpm_with_time_limit(double l_rpm, double r_rpm, int time_limt_msec) { 
  //==============================================================
  // Fixed Pid parameters tuned with experiments for team 99116X. 
  // DO NOT CHANGE!!!!
  //==============================================================
  double kp = 0.3;
  double ki = 0.01;
  double kd = 0.2;
  float start_time = Brain.timer(msec);

  //==============================================================
  // PID variables for left and right flying wheels.
  //==============================================================
  double error_l = 0;
  double last_error_l = 0;
  double error_sum_l = 0;
  double d_l;
  
  double error_r = 0;
  double last_error_r = 0;
  double error_sum_r = 0;
  double d_r;
  
  bool not_done = true;
  //==============================================================
  // Set up the motor speed for both left and righ flying wheels.
  int i = 0;
  double last_s_r = 0;
  double last_s_l = 0;
  do {
      double c_l_rpm = left_fw_motor.velocity(rpm);
      double c_r_rpm = right_fw_motor.velocity(rpm);
      error_l = l_rpm - c_l_rpm;
      error_r = r_rpm - c_r_rpm;


      if ((fabs(error_l) < 10 && fabs(error_r) < 10) || 
          Brain.timer(msec) - start_time > time_limt_msec) {
        not_done = false;    
      } else {
        error_sum_l += error_l;
        error_sum_r += error_r;
        
        d_l = kp * error_l + ki * error_sum_l + kd * (error_l - last_error_l);
        d_r = kp * error_r + ki * error_sum_r + kd * (error_r - last_error_r);
        
        left_fw_motor.spin(fwd, l_rpm + d_l, velocityUnits::rpm);
        right_fw_motor.spin(fwd, r_rpm + d_r, velocityUnits::rpm);

        last_error_l = error_l;
        last_error_r = error_r;
        
        wait(10, msec);
        Brain.Screen.setPenColor(red);
        Brain.Screen.drawLine(i + 20, 220-left_fw_motor.velocity(rpm) / 3,i + 20, 220- last_s_l / 3);

        Brain.Screen.setPenColor(green);
        Brain.Screen.drawLine(i + 20, 30 + right_fw_motor.velocity(rpm) / 3, i + 20, 30 + last_s_r / 3);
        last_s_l = l_rpm;
        last_s_r = r_rpm;
        i ++;
      }
    } while (not_done); 
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

void grab(float power) {
  grab_motor.spin(fwd, 0.128 * power, voltageUnits::volt);
}