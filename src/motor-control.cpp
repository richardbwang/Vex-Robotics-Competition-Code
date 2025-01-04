#include "vex.h"
#include "utils.h"
#include "pid.h"
#include <ctime>
#include <cmath>

#include "motor-control.h"
#include "autonomous.h"

bool isturning = false;
bool usevelocity = false;
double correct_angle = 0;

void ChassisControl(double left_power, double right_power) {
  left_chassis1.spin(fwd, 0.128 * right_power, voltageUnits::volt);
  left_chassis2.spin(fwd, 0.128 * right_power, voltageUnits::volt);
  left_chassis3.spin(fwd, 0.128 * right_power, voltageUnits::volt);
  
  right_chassis1.spin(fwd, 0.128 * left_power, voltageUnits::volt);
  right_chassis2.spin(fwd, 0.128 * left_power, voltageUnits::volt);
  right_chassis3.spin(fwd, 0.128 * left_power, voltageUnits::volt);
}

void ChassisControlReverse(double left_power, double right_power) {
  left_chassis1.spin(fwd, 0.128 * left_power, voltageUnits::volt);
  left_chassis2.spin(fwd, 0.128 * left_power, voltageUnits::volt);
  left_chassis3.spin(fwd, 0.128 * left_power, voltageUnits::volt);
  
  right_chassis1.spin(fwd, 0.128 * right_power, voltageUnits::volt);
  right_chassis2.spin(fwd, 0.128 * right_power, voltageUnits::volt);
  right_chassis3.spin(fwd, 0.128 * right_power, voltageUnits::volt);
}

void ChassisControl1(double left_power, double right_power, double left_only, double right_only) {
  if (right_only == true || left_only == false) {
    left_chassis1.spin(fwd, right_power, voltageUnits::volt);
    left_chassis2.spin(fwd, right_power, voltageUnits::volt);
    left_chassis3.spin(fwd, right_power, voltageUnits::volt);
  }
  
  if (left_only == true || right_only == false) {
  right_chassis1.spin(fwd, left_power, voltageUnits::volt);
  right_chassis2.spin(fwd, left_power, voltageUnits::volt);
  right_chassis3.spin(fwd, left_power, voltageUnits::volt);
  }
}

void ResetChassis() {
  left_chassis1.setPosition(0, degrees);
  left_chassis2.setPosition(0, degrees);
  left_chassis3.setPosition(0, degrees);
  
  right_chassis1.setPosition(0, degrees);
  right_chassis2.setPosition(0, degrees);
  right_chassis3.setPosition(0, degrees);
}

double GetRightRotationDegree() {
  return (left_chassis1.position(degrees) + left_chassis2.position(degrees) + left_chassis3.position(degrees)) / 3;
}

double GetLeftRotationDegree() {
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

double GetInertialHeading(bool normalize) {
  double result = InertialA.rotation(degrees);
  if(normalize == false) {
    return(result);
  } else {
    return NormalizeAngle(result);
  }
}

//1-60 degrees
double TurnSmall(double turn_angle, double time_limit_msec) {
  Stop(vex::brakeType::coast);
  isturning = true;
  double threshold = 0.8;
  const double kp = 0.72;
  const double ki = 0.00015;
  const double kd = 4.15;
  PID pid = PID(kp, ki, kd);
  
  pid.SetTarget(turn_angle);
  //pid.SetIntegralMax(300);  
  pid.SetProportionalRange(5);
  
  pid.SetSmallBigErrorTolerance(threshold, 0);
  pid.SetSmallBigErrorDuration(250, 0);
  pid.SetDerivativeTolerance(threshold * 4.5);
  // 5 Iterations
  
  // Draw the baseline.
  double draw_amplifier = 230 / fabs(turn_angle);
  Brain.Screen.clearScreen(black);
  Brain.Screen.setPenColor(green);
  Brain.Screen.drawLine(0, fabs(turn_angle) * draw_amplifier, 
                        600, fabs(turn_angle) * draw_amplifier);
  Brain.Screen.setPenColor(red);
  
  // Start the PID loop.
  double start_time = Brain.timer(msec);
  double output;
  double current_heading;
  double previous_heading = 0;
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
    ChassisControl1(output, -output);
    wait(10, msec);
  }  
  Stop(vex::hold);
  correct_angle = turn_angle;
  Controller1.Screen.print(GetInertialHeading());
  return GetInertialHeading();
  isturning = false;
}

//61-120 degrees
double TurnMedium(double turn_angle, double time_limit_msec) {
  Stop(vex::brakeType::coast);
  isturning = true;
  double threshold = 0.8;
  const double kp = 0.71;
  const double ki = 0.00016;
  const double kd = 4.12;
  PID pid = PID(kp, ki, kd);
  
  pid.SetTarget(turn_angle);
  //pid.SetIntegralMax(300);  
  pid.SetProportionalRange(5);
  
  pid.SetSmallBigErrorTolerance(threshold, 0);
  pid.SetSmallBigErrorDuration(250, 0);
  pid.SetDerivativeTolerance(threshold * 4.5);
  // 5 Iterations
  
  // Draw the baseline.
  double draw_amplifier = 230 / fabs(turn_angle);
  Brain.Screen.clearScreen(black);
  Brain.Screen.setPenColor(green);
  Brain.Screen.drawLine(0, fabs(turn_angle) * draw_amplifier, 
                        600, fabs(turn_angle) * draw_amplifier);
  Brain.Screen.setPenColor(red);
  
  // Start the PID loop.
  double start_time = Brain.timer(msec);
  double output;
  double current_heading;
  double previous_heading = 0;
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
    ChassisControl1(output, -output);
    wait(10, msec);
  }  
  Stop(vex::hold);
  correct_angle = turn_angle;
  Controller1.Screen.print(GetInertialHeading());
  return GetInertialHeading();
  isturning = false;
}

//121-180 degrees
double TurnBig(double turn_angle, double time_limit_msec) {
  Stop(vex::brakeType::coast);
  isturning = true;
  double threshold = 0.8;
  const double kp = 0.6;
  const double ki = 0.00014;
  const double kd = 4.16;
  PID pid = PID(kp, ki, kd);
  
  pid.SetTarget(turn_angle);
  //pid.SetIntegralMax(300);  
  pid.SetProportionalRange(5);
  
  pid.SetSmallBigErrorTolerance(threshold, 0);
  pid.SetSmallBigErrorDuration(250, 0);
  pid.SetDerivativeTolerance(threshold * 4.5);
  // 5 Iterations
  
  // Draw the baseline.
  double draw_amplifier = 230 / fabs(turn_angle);
  Brain.Screen.clearScreen(black);
  Brain.Screen.setPenColor(green);
  Brain.Screen.drawLine(0, fabs(turn_angle) * draw_amplifier, 
                        600, fabs(turn_angle) * draw_amplifier);
  Brain.Screen.setPenColor(red);
  
  // Start the PID loop.
  double start_time = Brain.timer(msec);
  double output;
  double current_heading;
  double previous_heading = 0;
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
    ChassisControl1(output, -output);
    wait(10, msec);
  }  
  Stop(vex::hold);
  correct_angle = turn_angle;
  Controller1.Screen.print(GetInertialHeading());
  return GetInertialHeading();
  isturning = false;
}

void TurnToAngle(double turn_angle, double time_limit_msec) {
  Stop(vex::brakeType::coast);
  isturning = true;
  //REMINDER: EXIT CONDITION REMOVED FOR TUNING
  double threshold = 1;
  const double kp = 0.9;
  const double ki = 0.0433-(((fabs(turn_angle - GetInertialHeading()) - 90)/30))*0.005;
  const double kd = 6;
  PID pid = PID(kp, ki, kd);
  
  pid.SetTarget(turn_angle);
  //pid.SetIntegralMax(300);  
  pid.SetProportionalRange(5);
  
  pid.SetSmallBigErrorTolerance(threshold, threshold * 3);
  pid.SetSmallBigErrorDuration(100, 500);
  pid.SetDerivativeTolerance(threshold * 4.5);
  // 5 Iterations
  
  // Draw the baseline.
  double draw_amplifier = 230 / fabs(turn_angle);
  Brain.Screen.clearScreen(black);
  Brain.Screen.setPenColor(green);
  Brain.Screen.drawLine(0, fabs(turn_angle) * draw_amplifier, 
                        600, fabs(turn_angle) * draw_amplifier);
  Brain.Screen.setPenColor(red);
  
  // Start the PID loop.
  double start_time = Brain.timer(msec);
  double output;
  double current_heading;
  double previous_heading = 0;
  int index = 1;
  //REMINDER: EXIT CONDITION IS REMOVED FOR TUNING
  while (!pid.TargetArrived() && Brain.timer(msec) - start_time <= time_limit_msec) {
    current_heading = GetInertialHeading();
    output = pid.Update(current_heading);
    
    // Draw line
    Brain.Screen.drawLine(
        index * 3, fabs(previous_heading) * draw_amplifier, 
        (index + 1) * 3, fabs(current_heading * draw_amplifier));
    index++;
    previous_heading = current_heading;
    // End
    ChassisControl1(output, -output);
    wait(10, msec);
  }  
  Stop(vex::hold);
  correct_angle = turn_angle;
  Controller1.Screen.print(GetInertialHeading());
  isturning = false;
}

void DriveTo(double distance_in, double time_limit_msec, double max_output) {
  ResetChassis();
  Stop(vex::brakeType::coast);
  isturning = true;
  // Current robot setup, 3.25 inch wheel, 60/36 gear ratio.
  // Tuned parameters, DO NOT CHANGE!!!
  double threshold = 0.5;
  const double kp = 1.7, ki = 0.1, kd = 10;
  const double kph = 0.72, kih = 0.00015, kdh = 4.15;

  int drive_direction = distance_in > 0 ? 1 : -1;
  distance_in = distance_in * drive_direction;
  PID pidleft = PID(kp, ki, kd);
  PID pidright = PID(kp, ki, kd);
  PID pidh = PID(kph, kih, kdh);

  pidleft.SetTarget(distance_in);
  pidleft.SetIntegralMax(300);  
  pidleft.SetProportionalRange(3);
  pidleft.SetSmallBigErrorTolerance(threshold, 0);
  pidleft.SetSmallBigErrorDuration(50, 0);
  pidleft.SetDerivativeTolerance(threshold * 4.5);
  // 5 Iterations

  pidright.SetTarget(distance_in);
  pidright.SetIntegralMax(300);  
  pidright.SetProportionalRange(3);
  pidright.SetSmallBigErrorTolerance(threshold, 0);
  pidright.SetSmallBigErrorDuration(50, 0);
  pidright.SetDerivativeTolerance(threshold * 4.5);
  // 5 Iterations
  
  pidh.SetTarget(correct_angle);
  //pid.SetIntegralMax(300);  
  pidh.SetProportionalRange(3);
  
  pidh.SetSmallBigErrorTolerance(0, 0);
  pidh.SetSmallBigErrorDuration(0, 0);
  pidh.SetDerivativeTolerance(0);
  // 5 Iterations

  // Reset the chassis.
  ResetChassis();
  double start_time = Brain.timer(msec);
  double leftoutput = 0, rightoutput = 0, correction_output = 0;

  double current_left = 0, current_right = 0, current_angle = 0;

  while ((!pidleft.TargetArrived() || !pidright.TargetArrived()) && Brain.timer(msec) - start_time <= time_limit_msec) {
    current_left = fabs((GetLeftRotationDegree() / 360.0) * wheel_distance_in);
    current_right = fabs((GetRightRotationDegree() / 360.0) * wheel_distance_in);
    current_angle = GetInertialHeading();
    leftoutput = pidleft.Update(fabs((GetLeftRotationDegree() / 360.0) * wheel_distance_in));
    rightoutput = pidright.Update(fabs((GetRightRotationDegree() / 360.0) * wheel_distance_in));
    correction_output = pidh.Update(current_angle);
    if(leftoutput/* + correction_output */> max_output && max_output != 0/* && correction_output >= 0*/) {
      leftoutput = max_output/* - correction_output*/;
    } else if(leftoutput/* + correction_output */< -max_output && max_output != 0) {
      leftoutput = -max_output/* - correction_output*/;
    }
    if(rightoutput/* - correction_output */< -max_output && max_output != 0/* && correction_output >= 0*/) {
      rightoutput = -max_output/* + correction_output*/;
    } else if(rightoutput/* - correction_output */> max_output && max_output != 0) {
      rightoutput = max_output/* + correction_output*/;
    }
    /*
    leftoutput = leftoutput + correction_output;
    rightoutput = rightoutput - correction_output;
    */
    ChassisControl1(leftoutput * drive_direction * pow((distance_in-current_left)/distance_in, 5) + leftoutput*1/2*drive_direction, 
    rightoutput * drive_direction * pow(((distance_in-current_right)/distance_in), 5) + rightoutput*1/2*drive_direction);
    wait(10, msec);
  }
  Stop(vex::hold);
  isturning = false;
}

void DriveToNew(double distance_in, double time_limit_msec, double max_output) {
  ResetChassis();
  Stop(vex::brakeType::coast);
  isturning = true;
  // Current robot setup, 3.25 inch wheel, 60/36 gear ratio.
  // Tuned parameters, DO NOT CHANGE!!!
  double threshold = 0.25;
  const double kp = 1.04, ki = 0.007, kd = 12;
  const double kph = 0.3, kih = 0, kdh = 2;

  int drive_direction = distance_in > 0 ? 1 : -1;
  double max_deceleration = 0.3;
  distance_in = distance_in * drive_direction;
  PID piddistance = PID(kp, ki, kd);
  PID pidh = PID(kph, kih, kdh);

  piddistance.SetTarget(distance_in);
  piddistance.SetIntegralMax(300);  
  piddistance.SetProportionalRange(5);
  piddistance.SetSmallBigErrorTolerance(threshold, threshold * 3);
  piddistance.SetSmallBigErrorDuration(50, 250);
  piddistance.SetDerivativeTolerance(5);
  // 5 Iterations
  
  pidh.SetTarget(correct_angle);
  //pid.SetIntegralMax(300);  
  pidh.SetProportionalRange(0);
  
  pidh.SetSmallBigErrorTolerance(0, 0);
  pidh.SetSmallBigErrorDuration(0, 0);
  pidh.SetDerivativeTolerance(0);
  // 5 Iterations

  // Reset the chassis.
  double start_time = Brain.timer(msec);
  double leftoutput = 0, rightoutput = 0, correction_output = 0, previous_left_output = 0, previous_right_output = 0;

  double current_distance = 0, current_angle = 0;

  while ((!piddistance.TargetArrived()) && Brain.timer(msec) - start_time <= time_limit_msec) {
    current_distance = (fabs((GetLeftRotationDegree() / 360.0) * wheel_distance_in) + fabs((GetRightRotationDegree() / 360.0) * wheel_distance_in)) / 2;
    current_angle = GetInertialHeading();
    leftoutput = piddistance.Update(current_distance);
    rightoutput = leftoutput;
    correction_output = pidh.Update(current_angle);
    if(drive_direction == 1) {
      leftoutput = leftoutput + correction_output;
      rightoutput = rightoutput - correction_output;
    } else {
      leftoutput = leftoutput - correction_output;
      rightoutput = rightoutput + correction_output;
    }

    //Max Output Check
    if(fabs(leftoutput) > fabs(rightoutput) && leftoutput > max_output) {
      rightoutput = rightoutput / leftoutput * max_output;
      leftoutput = max_output;
    } else if(fabs(rightoutput) > fabs(leftoutput) && rightoutput > max_output) {
      leftoutput = leftoutput / rightoutput * max_output;
      rightoutput = max_output;
    } else if(fabs(leftoutput) > fabs(rightoutput) && leftoutput < -max_output) {
      rightoutput = rightoutput / leftoutput * -max_output;
      leftoutput = -max_output;
    } else if(fabs(rightoutput) > fabs(leftoutput) && rightoutput < -max_output) {
      leftoutput = leftoutput / rightoutput * -max_output;
      rightoutput = -max_output;
    }

    //Max Acceleration/Deceleration Check
    if(previous_left_output - leftoutput > max_deceleration) {
      leftoutput = previous_left_output - max_deceleration;
    }
    if(previous_right_output - rightoutput > max_deceleration) {
      rightoutput = previous_right_output - max_deceleration;
    }
    previous_left_output = leftoutput;
    previous_right_output = rightoutput;
    ChassisControl1(leftoutput * drive_direction, rightoutput * drive_direction);
    wait(10, msec);
  }
  Brain.Screen.print(GetLeftRotationDegree()/360 * wheel_distance_in);
  Brain.Screen.newLine();
  Brain.Screen.print(GetRightRotationDegree()/360 * wheel_distance_in);
  Brain.Screen.newLine();
  Stop(vex::hold);
  isturning = false;
}

void Grab(double power) {
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

void PullCatapult() {
  int start_time = Brain.timer(msec);
  while(LimitSwitchA.pressing() == 0 && Brain.timer(msec) - start_time < 5000) {
    catapult_motor.spin(fwd, 100 * 0.128, volt);
    wait(10, msec);
  }
  if(Brain.timer(msec) - start_time > 5000) {
    catapult_motor.stop(coast);
  } else {
    catapult_motor.stop(hold);
  }
}

void FireCatapult() {
  while(LimitSwitchA.pressing() == 1) {
    catapult_motor.spin(fwd, 100 * 0.128, volt);
    wait(10, msec);
  }
  
  catapult_motor.stop(hold);
  
  // Wait for the limited switch to be fully released.
  while(LimitSwitchA.pressing() == 1) {
    wait(10, msec);
  }
  wait(100, msec);
}

void AngleStabilization() {
  double initialheading = 0, newheading = 0;
  while(true)  {
    initialheading = GetInertialHeading();
    wait(50, msec);
    newheading = GetInertialHeading();
    if(isturning == false && fabs(initialheading - newheading) > 3) {
      TurnSmallNoBool(initialheading, 1000);
    }
  }
}

void GrabCheck() {
  Grab(40);
  do {
    wait(10, msec);
  } while(LimitSwitchA.pressing() == 0);
  Grab(70);
}

void GrabCheckFast() {
  Grab(45);
  do {
    wait(10, msec);
  } while(LimitSwitchA.pressing() == 0);
  Grab(100);
}

void CurveCircle(double result_angle_deg, double center_radius, double time_limit_msec, double max_output) {
  double in_arc, out_arc;
  double real_angle = 0, current_angle = 0;
  double ratio, result_angle;
  const double start_angle = GetInertialHeading();
  result_angle = result_angle_deg * 3.14159265359 / 180;
  in_arc = fabs((fabs(center_radius) - 5.25) * result_angle);
  out_arc = fabs((fabs(center_radius) + 5.25) * result_angle);
  ratio = in_arc / out_arc;
  ResetChassis();
  Stop(vex::brakeType::coast);
  isturning = true;
  // Tuned parameters, DO NOT CHANGE!!!
  double threshold = 0.5;
  const double kp = 1.6, ki = 0, kd = 10;
  const double kpt = 0.6, kit = 0.0002, kdt = 4.2;

  int curve_direction = center_radius > 0 ? 1 : -1;
  int drive_direction = 0;
  if ((curve_direction == 1 && result_angle_deg > 0) || (curve_direction == -1 && result_angle_deg < 0)) {
    drive_direction = 1;
  } else {
    drive_direction = -1;
  }

  PID pidout = PID(kp, ki, kd);
  PID pidturn = PID(kpt, kit, kdt);
  PID pidturnarrived = PID(kpt, kit, kdt);

  pidout.SetTarget(out_arc);
  pidout.SetIntegralMax(300);  
  pidout.SetProportionalRange(3);
  pidout.SetSmallBigErrorTolerance(threshold, 0);
  pidout.SetSmallBigErrorDuration(100, 0);
  pidout.SetDerivativeTolerance(threshold * 4.5);
  // 5 Iterations
  
  pidturn.SetTarget(0);
  pidturn.SetIntegralMax(100);  
  pidturn.SetProportionalRange(0.3);
  
  pidturn.SetSmallBigErrorTolerance(0, 0);
  pidturn.SetSmallBigErrorDuration(0, 0);
  pidturn.SetDerivativeTolerance(0);
  // 5 Iterations

  pidturnarrived.SetTarget(result_angle_deg);
  pidturnarrived.SetSmallBigErrorTolerance(1, 0);
  pidturnarrived.SetSmallBigErrorDuration(400, 0);

  // Reset the chassis.
  ResetChassis();
  double start_time = Brain.timer(msec);
  double left_output = 0, right_output = 0, correction_output = 0;
  double current_right = 0, current_left = 0;

  if (curve_direction == -1) {
    while (!pidout.TargetArrived() && Brain.timer(msec) - start_time <= time_limit_msec && !pidturnarrived.TargetArrived()) {
      current_angle = fabs(GetInertialHeading() - start_angle);
      current_right = fabs((GetRightRotationDegree() / 360.0) * wheel_distance_in);
      real_angle = fabs(current_right/out_arc * result_angle_deg);
      pidturn.SetTarget(real_angle);
      right_output = pidout.Update(current_right);
      left_output = right_output * ratio;
      correction_output = pidturn.Update(fabs(current_angle));
      right_output = right_output + correction_output;
      left_output = left_output - correction_output;

      //Max Output Check
      if(fabs(left_output) > fabs(right_output) && left_output > max_output) {
        right_output = right_output / left_output * max_output;
        left_output = max_output;
      } else if(fabs(right_output) > fabs(left_output) && right_output > max_output) {
        left_output = left_output / right_output * max_output;
        right_output = max_output;
      } else if(fabs(left_output) > fabs(right_output) && left_output < -max_output) {
        right_output = right_output / left_output * -max_output;
        left_output = -max_output;
      } else if(fabs(right_output) > fabs(left_output) && right_output < -max_output) {
        left_output = left_output / right_output * -max_output;
        right_output = -max_output;
      }

      ChassisControl1(left_output * drive_direction, right_output * drive_direction);
      wait(10, msec);
    }
  } else {
    while (!pidout.TargetArrived() && Brain.timer(msec) - start_time <= time_limit_msec && !pidturnarrived.TargetArrived()) {
      current_angle = fabs(GetInertialHeading() - start_angle);
      current_left = fabs((GetLeftRotationDegree() / 360.0) * wheel_distance_in);
      real_angle = fabs(current_left/out_arc * result_angle_deg);
      pidturn.SetTarget(real_angle);
      left_output = pidout.Update(current_left);
      right_output = left_output * ratio;
      correction_output = pidturn.Update(fabs(current_angle));
      left_output = left_output + correction_output;
      right_output = right_output - correction_output;

      //Max Output Check
      if(fabs(left_output) > fabs(right_output) && left_output > max_output) {
        right_output = right_output / left_output * max_output;
        left_output = max_output;
      } else if(fabs(right_output) > fabs(left_output) && right_output > max_output) {
        left_output = left_output / right_output * max_output;
        right_output = max_output;
      } else if(fabs(left_output) > fabs(right_output) && left_output < -max_output) {
        right_output = right_output / left_output * -max_output;
        left_output = -max_output;
      } else if(fabs(right_output) > fabs(left_output) && right_output < -max_output) {
        left_output = left_output / right_output * -max_output;
        right_output = -max_output;
      }

      ChassisControl1(left_output * drive_direction, right_output * drive_direction);
      wait(10, msec);
    }
  }
  Stop(vex::brakeType::hold);
  correct_angle = correct_angle + result_angle_deg;
  isturning = false;
}

void DriveToSeperate(double l_distance_in, double r_distance_in, double time_limit_msec, double max_output) {
  ResetChassis();
  Stop(vex::brakeType::coast);
  isturning = true;
  // Current robot setup, 3.25 inch wheel, 60/36 gear ratio.
  // Tuned parameters, DO NOT CHANGE!!!
  double threshold = 0.5;
  const double kp = 1.7, ki = 0.002, kd = 10;

  double l_drive_direction = l_distance_in > 0 ? 1 : -1;
  double r_drive_direction = r_distance_in > 0 ? 1 : -1;
  l_distance_in = l_distance_in * l_drive_direction;
  r_distance_in = r_distance_in * l_drive_direction;
  PID pidleft = PID(kp, ki, kd);
  PID pidright = PID(kp, ki, kd);

  pidleft.SetTarget(l_distance_in);
  pidleft.SetIntegralMax(300);  
  pidleft.SetProportionalRange(fabs(l_distance_in) / 2);
  pidleft.SetSmallBigErrorTolerance(threshold, 0);
  pidleft.SetSmallBigErrorDuration(50, 0);
  pidleft.SetDerivativeTolerance(threshold * 4.5);
  // 5 Iterations

  pidright.SetTarget(r_distance_in);
  pidright.SetIntegralMax(300);  
  pidright.SetProportionalRange(fabs(r_distance_in) / 2);
  pidright.SetSmallBigErrorTolerance(threshold, 0);
  pidright.SetSmallBigErrorDuration(50, 0);
  pidright.SetDerivativeTolerance(threshold * 4.5);
  // 5 Iterations

  // Reset the chassis.
  ResetChassis();
  double start_time = Brain.timer(msec);
  double leftoutput = 0;
  double rightoutput = 0;

  double current_left = 0, current_right = 0;

  while ((!pidleft.TargetArrived() || !pidright.TargetArrived()) && Brain.timer(msec) - start_time <= time_limit_msec) {
    current_left = fabs((GetLeftRotationDegree() / 360.0) * wheel_distance_in);
    current_right = fabs((GetRightRotationDegree() / 360.0) * wheel_distance_in);
    leftoutput = pidleft.Update(fabs((GetLeftRotationDegree() / 360.0) * wheel_distance_in));
    rightoutput = pidright.Update(fabs((GetRightRotationDegree() / 360.0) * wheel_distance_in));
    ChassisControl1(leftoutput * l_drive_direction * pow((l_distance_in-current_left)/l_distance_in, 5) + leftoutput*1/2*l_drive_direction, 
    rightoutput * r_drive_direction * pow(((r_distance_in-current_right)/r_distance_in), 5) + rightoutput*1/2*r_drive_direction);
    wait(10, msec);
  }
  Stop(vex::brakeType::hold);
  ChassisControl1(0, 0);
  isturning = false;
}

void WiggleForward(double distance_in, double wiggle_amplifier, double wiggle_frequency, double forward_speed, double wiggle_time, double drive_time_limit_msec) {
  ResetChassis();
  Stop(vex::brakeType::coast);
  isturning = true;
  const double wheel_distance_in = (36.0 / 60.0) * 3.25 * 3.14159;
  double start_time = Brain.timer(msec);
  double current_time = 0;
  while(Brain.timer(msec) - start_time < wiggle_time) {
    ChassisControl1(forward_speed + wiggle_amplifier * sin(current_time * wiggle_frequency), forward_speed - wiggle_amplifier * sin(current_time * wiggle_frequency));
    current_time = Brain.timer(msec) - start_time;
    wait(10, msec);
  }
  DriveToSeperate(distance_in - (GetLeftRotationDegree() / 360.0) * wheel_distance_in, distance_in - (GetRightRotationDegree() / 360.0) * wheel_distance_in, 0, drive_time_limit_msec);
  isturning = false;
}

void distance_check() {
  Grab(100);
  while (DistanceA.objectDistance(inches) > 3) {
    wait(10, msec);
  }
  wait(100, msec);
  Grab(0);
}

void grab_wait() {
  while((GetLeftRotationDegree() + GetRightRotationDegree())/720 * wheel_distance_in < 90) {
    wait(10, msec);
  }
  Grab(0);
}

void heading_correction() {
  double output = 0;
  const double kp = 0.72;
  const double ki = 0.00015;
  const double kd = 4.15;
  PID pid = PID(kp, ki, kd);
  
  pid.SetTarget(correct_angle);
  //pid.SetIntegralMax(300);  
  pid.SetProportionalRange(correct_angle);
  
  pid.SetSmallBigErrorTolerance(0, 0);
  pid.SetSmallBigErrorDuration(0, 0);
  pid.SetDerivativeTolerance(0);
  // 5 Iterations
  
  // Start the PID loop.
  while(true) {
    pid.SetTarget(correct_angle);
    if(isturning == false) {
      output = pid.Update(GetInertialHeading());
      ChassisControl1(output, -output);
    }
    wait(10, msec);
  }
}