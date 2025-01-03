#include "vex.h"
#include "utils.h"
#include "pid.h"
#include <ctime>
#include <cmath>

#include "motor-control.h"

void ChassisControl(float left_power, float right_power) {
  left_chassis1.spin(fwd, 0.128 * left_power, voltageUnits::volt);
  left_chassis2.spin(fwd, 0.128 * left_power, voltageUnits::volt);
  left_chassis3.spin(fwd, 0.128 * left_power, voltageUnits::volt);
  
  right_chassis1.spin(fwd, 0.128 * right_power, voltageUnits::volt);
  right_chassis2.spin(fwd, 0.128 * right_power, voltageUnits::volt);
  right_chassis3.spin(fwd, 0.128 * right_power, voltageUnits::volt);
}

void ChassisControlPercent(float left_velocity, float right_velocity) {
  left_chassis1.spin(forward, left_velocity, percent);
  left_chassis2.spin(forward, left_velocity, percent);
  left_chassis3.spin(forward, left_velocity, percent);

  right_chassis1.spin(forward, right_velocity, percent);
  right_chassis2.spin(forward, right_velocity, percent);
  right_chassis3.spin(forward, right_velocity, percent);
}

void ResetChassis() {
  left_chassis1.setPosition(0, degrees);
  left_chassis2.setPosition(0, degrees);
  left_chassis3.setPosition(0, degrees);
  
  right_chassis1.setPosition(0, degrees);
  right_chassis2.setPosition(0, degrees);
  right_chassis3.setPosition(0, degrees);
}

double GetLeftRotationDegree() {
  return (left_chassis1.position(degrees) + left_chassis2.position(degrees) + left_chassis3.position(degrees)) / 3;
}

double GetRightRotationDegree() {
  return (right_chassis1.position(degrees) + right_chassis2.position(degrees) + right_chassis3.position(degrees)) / 3;
}


//================================Chassis Movement=========================================//
double NormalizeAngle(double angle) {
  if (angle > 180) {
    angle = angle - 360;
  } else if (angle < -180) {
    angle = angle + 360;
  }

  return angle;
}

double GetInertialHeading() {
  double result = InertialA.heading(degrees);
  return NormalizeAngle(result);
}

double TurnToAngle(float turn_angle, float time_limit_msec) {
  double threshold = 0.3;
  PID pid = PID(0.65, 0.00035, 1.5);
  
  pid.SetTarget(turn_angle);
  pid.SetIntegralMax(200);  
  pid.SetProportionalRange(fabs(turn_angle) / 2);
  
  pid.SetErrorTolerance(threshold);
  pid.SetDerivativeTolerance(threshold * 4.5);
  // 5 Iterations
  pid.SetStableTimeDuration(20);
  
  // Draw the baseline.
  double draw_amplifier = 230 / fabs(turn_angle);
  Brain.Screen.clearScreen(black);
  Brain.Screen.setPenColor(green);
  Brain.Screen.drawLine(0, fabs(turn_angle) * draw_amplifier, 
                        600, fabs(turn_angle) * draw_amplifier);
  Brain.Screen.setPenColor(red);
  
  // Start the PID loop.
  InertialA.setHeading(0, degrees);
  float start_time = Brain.timer(msec);
  float output;
  float current_heading;
  float previous_heading = 0;
  int index = 1;
  while (!pid.TargetArrived() &&
         Brain.timer(msec) - start_time <= time_limit_msec) {
    current_heading = GetInertialHeading();
    output = pid.Update(GetInertialHeading());
    
    // Draw line
    Brain.Screen.drawLine(
        index * 3, fabs(previous_heading) * draw_amplifier, 
        (index + 1) * 3, fabs(current_heading * draw_amplifier));
    index++;
    previous_heading = current_heading;
    // End

    ChassisControlPercent(-output, output);
    wait(10, msec);
  }  
  Stop(vex::brake);
  return GetInertialHeading();
}

double TurnForAngle(float turn_angle, float time_limit_msec) {
  /*
  Brain.Screen.clearScreen(black);
  Brain.Screen.setPenColor(green);
  double draw_amplifier = 3;
  Brain.Screen.drawLine(0, fabs(turn_angle) * draw_amplifier, 
                        600, fabs(turn_angle) * draw_amplifier);
  Brain.Screen.setPenColor(red);
  */

  // Decide the error threshhold.
  double threshold = fabs(turn_angle) * 0.003 > 0.35 ? fabs(turn_angle) * 0.003 : 0.35;

  
  // Predefined paramaters, DO NOT CHANGE!!!
  std::string result;
  //const float kp = 0.65, ki = 0.0004, kd = 45;
  //const float kp = 0.66, ki = 0.0003, kd = 53;
  //const float kp = 0.65, ki = 0.00015,  kd = 75;
  float kp = 0.59, ki = 0 * 0.66, kd = 13.5;

  float prev_time = 0, delta_time = 0, current_time = 0;
  float integral = 0, error = 0, derivative = 0, prev_error = 0;
  // float prev_angle = 0; 
  float current_angle = 0;

  // Condition variable to control the loop.
  bool not_done = true;

  // Reset heading and timer.
  InertialA.setHeading(0, degrees);
  float start_time = Brain.timer(msec);
  
  // Start to turn.
  // int index = 0, xx = 0;
  do {
    current_time = Brain.timer(msec);
    delta_time = prev_time == 0 ? 0 : current_time - prev_time;
    prev_time = current_time;
    current_angle = GetInertialHeading();
    /*
    Brain.Screen.drawLine(
      index * 3, fabs(prev_angle) * draw_amplifier, 
      (index + 1) * 3, fabs(current_angle * draw_amplifier));
    index++;
    prev_angle = current_angle;
    */

    // Compute KP, KI, KD.
    error = turn_angle - current_angle;
    if ((fabs(error) <= threshold) || (Brain.timer(msec) - start_time >= time_limit_msec)) {
      not_done = false;
      Stop(vex::brake);
      // Brain.Screen.printAt(15, xx * 20, "Final result: %.2f %.2f %.2f %.2f", current_angle, kp * error, ki * integral, kd * derivative); 
    } else {
      integral += error * delta_time;      
      if (error * integral < 0) {
        integral /= 2;
      } 
      integral += error * delta_time;
      derivative = delta_time == 0 ? 0 : (error - prev_error) / delta_time;      
      prev_error = error;

      float drive_speed = kp * error + ki * integral + kd * derivative;
      ChassisControlPercent(-drive_speed, drive_speed);
      /*
      if (fabs(error) < 2) {
        Brain.Screen.printAt(15, xx * 20, "Mid result: %.2f %.2f %.2f %.2f %.2f", current_angle, kp * error, ki * integral, kd * derivative, drive_speed); 
        xx++;
      }*/
    }

    wait(10, msec);
  } while (not_done);

  return InertialA.heading(degrees);
}

void DriveFor(float distance_in, float time_limit_msec) {
  // Current robot setup, 3.25 inch wheel, 60/36 gear ratio.
  const float wheel_distance_in = (60.0 / 36.0) * 3.25 * 3.14159;
  // Tuned parameters, DO NOT CHANGE!!!
  const float kp = 3, ki = 0.0001, kd = 50;

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
      Stop(vex::brakeType::coast);
      ChassisControlPercent(0, 0);
    } else {
      ChassisControlPercent(left_speed * drive_direction, right_speed * drive_direction);
    }
    wait(20, msec);
  } while (not_done);
}


void DriveFor(float distance_in, float time_limit_msec, bool catapult) {
  // Current robot setup, 3.25 inch wheel, 60/36 gear ratio.
  const float wheel_distance_in = (60.0 / 36.0) * 3.25 * 3.14159;
  // Tuned parameters, DO NOT CHANGE!!!
  const float kp = 3, ki = 0.0001, kd = 50;

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
  do {
    current_time = Brain.timer(msec);
    delta_time = prev_time == 0 ? 0 : current_time - prev_time;
    prev_time = current_time;

    current_left = fabs((GetLeftRotationDegree() / 360.0) * wheel_distance_in);
    current_right = fabs((GetRightRotationDegree() / 360.0) * wheel_distance_in);
    l_error = distance_in - current_left;
    r_error = distance_in - current_right;

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

    if (catapult) {
      if (LimitSwitchA.pressing() == 0) {
        catapult_motor.spin(fwd,100*0.128,volt);
      } else {
        catapult_motor.stop(hold);
      }
    }

    float left_speed = kp * l_error + ki * l_integral + kd * l_derivative;
    float right_speed = kp * r_error + ki * r_integral + kd * r_derivative;
    if (((fabs(l_error) < .5) && (fabs(r_error) < .5)) || 
        (Brain.timer(msec) - start_time > time_limit_msec)) {
      not_done = false;
      Stop(vex::brakeType::coast);
      ChassisControlPercent(0, 0);
    } else {
      ChassisControlPercent(left_speed * drive_direction * 0.85, right_speed * drive_direction * 0.85);
    }

    wait(10, msec);
  } while (not_done);
}

void Grab(float power) {
  intake_motor.spin(fwd, 0.128 * power, voltageUnits::volt);
}

void Stop(brakeType type) {
  left_chassis1.stop(type);
  left_chassis2.stop(type);
  left_chassis3.stop(type);
  
  right_chassis1.stop(type);
  right_chassis2.stop(type);
  right_chassis3.stop(type);
}