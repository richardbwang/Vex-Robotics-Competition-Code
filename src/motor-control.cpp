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
  const float kP = (0.1*(turnAngle/180*0.5))+0.63, kI = 0.00001, kD = 0.43*turnAngle/180+0.5;

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

   
    ChassisControlPercent(-driveSpeed, driveSpeed);

    if ((fabs(error) < fabs(turnAngle * 0.005)) || (Brain.timer(msec) - startTime >= timeLimit_msec)) {
      notDone = false;
      Stop(hold);
      //Brain.Screen.printAt(10, 10, "FC:%.2f E:%.2f I:%.2f D:%.2f S:%.2f", 
      //                              currentAngle, error, integral, derivative, driveSpeed);
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

void DriveForTest(float distance_in)
{
  ResetChassis();
  float wheel_distance_in = 60.0 / 36.0 * 3.25 * 3.14159;
  bool notdone = true;
  //Brain.Screen.print(GetLeftRotationDegree());
  //Brain.Screen.print(GetLeftRotationDegree());
  //Brain.Screen.newLine();
  do {
    float currentleft1=GetLeftRotationDegree() / 360.0;
    float currentright1=GetLeftRotationDegree() / 360.0;
    float current_left = currentleft1 * wheel_distance_in;
    float current_right = currentleft1 * wheel_distance_in;
    if(current_left < distance_in && current_right < distance_in) {
      ChassisControl(75, 75);
      wait(1, msec);
    } else {
    Stop(vex::brakeType::brake);
    /*Brain.Screen.print(GetLeftRotationDegree());
    Brain.Screen.print(GetLeftRotationDegree());
    Brain.Screen.newLine();
    Brain.Screen.print(current_left);
    Brain.Screen.print(current_right);
    Brain.Screen.newLine();
    Brain.Screen.print(currentleft1);
    Brain.Screen.print(currentright1);
    Brain.Screen.newLine();
    Brain.Screen.print(wheel_distance_in);
    Brain.Screen.newLine();*/
    notdone = false;
    }
  } while(notdone == true);
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
  right_fw_motor.spin(fwd, 6 * rRpm, velocityUnits::rpm);
}


void fw_pid_rpm_with_time_limit(double lRpm, double rRpm, int time_limt_msec) { 
  //==============================================================
  // Fixed Pid parameters tuned with experiments for team 99116X. 
  // DO NOT CHANGE!!!!
  //==============================================================
  double kp = -0.3;
  double ki = -0.01;
  double kd = -0.2;
  int total_wait_time_msec = 0;

  // Set the target rRpm based on motor ratio;
  lRpm = 6 * lRpm; 
  rRpm = 6 * rRpm;

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
  //==============================================================
  // Set up the motor speed for both left and righ flying wheels.
  int i = 0;
  double last_s_r = 0;
  double last_s_l = 0;
  do {
      double l_rpm = left_fw_motor.velocity(rpm);
      double r_rpm = right_fw_motor.velocity(rpm);
      error_l = l_rpm - lRpm;
      error_r = r_rpm - rRpm;

      error_sum_l += error_l;
      error_sum_r += error_r;
      
      d_l = kp * error_l + ki * error_sum_l + kd * (error_l - last_error_l);
      d_r = kp * error_r + ki * error_sum_r + kd * (error_r - last_error_r);

      left_fw_motor.spin(fwd, lRpm - d_l, velocityUnits::rpm);
      right_fw_motor.spin(fwd, rRpm - d_r, velocityUnits::rpm);

      last_error_l = error_l;
      last_error_r = error_r;
      
      wait(10, msec);
      total_wait_time_msec += 10;

      Brain.Screen.setPenColor(red);
      Brain.Screen.drawLine(i + 20, 220-left_fw_motor.velocity(rpm) / 3,i + 20, 220- last_s_l / 3);

      Brain.Screen.setPenColor(green);
      Brain.Screen.drawLine(i + 20, 30 + right_fw_motor.velocity(rpm) / 3, i + 20, 30 + last_s_r / 3);
      last_s_l = l_rpm;
      last_s_r = r_rpm;
      i ++;
      // Print
      // Brain.Screen.print("L:%4.2f,R:%4.2f\n", l_rpm, r_rpm);
  } while ((total_wait_time_msec < time_limt_msec) &&
           fabs(error_l) < 5 &&
           fabs(error_r) < 5); 
  //Brain.Screen.print("I:%d\n", i);
}

bool AimingGoal() {
  int found_num = VisionA.takeSnapshot(VisionA__HIGH_GOAL_BLUE);
  if (found_num > 1) {
    // The following makes sure auto aiming is not turned on if the basket is
    // not correctly identified via some basic checks.
    //
    // The largest two objects captured should be the basket bottom and top.
    VexVisionObject high_goal_bottom = VisionA.objects[0];
    VexVisionObject high_goal_top = VisionA.objects[1];
    // Make sure top is above bottom.
    if (high_goal_top.centerY < high_goal_bottom.centerY)
      return false;
    // Make sure top and bottom are verticall aligned.
    if (abs(high_goal_top.centerX - high_goal_bottom.centerX) > 30)
      return false;
    // Make sure bottom is wider than top.
    if (high_goal_top.width > high_goal_bottom.width)
      return false;
    // Make sure bottom is larger than top in height.
    //if (high_goal_top.height > high_goal_bottom.height)
      //return false;

    int x = VisionA.largestObject.centerX;
    // int y = VisionA.largestObject.centerY;
    // int h = VisionA.largestObject.height;
    // int w = VisionA.largestObject.width;
    // int a = VisionA.largestObject.angle;
    // int ox = VisionA.largestObject.originX;
    // int oy = VisionA.largestObject.originY;
    if ((x <= 150) && (x >= 145)) {
      // Aiming is good, shoot
    } else {
      double degree = (double(x) - 150.00) * 30 / 80;
      TurnForAngle(degree, 200);
    }
    return true;
  }

  return false;
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