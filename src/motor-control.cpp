#include "vex.h"
#include "utils.h"
#include "pid.h"
#include <ctime>
#include <cmath>

#include "motor-control.h"
#include "autonomous.h"

bool isturning = false;
bool usevelocity = false;
bool headingcorrection = true;
double xpos = 0, ypos = 0;
double correct_angle = 0, cx = 0, cy = 0;
double distance_value = 35;
double distancebetweenwheels = 11.2;

void DriveControl(double left_power, double right_power) {
  left_chassis1.spin(fwd, 0.128 * left_power, voltageUnits::volt);
  left_chassis2.spin(fwd, 0.128 * left_power, voltageUnits::volt);
  left_chassis3.spin(fwd, 0.128 * left_power, voltageUnits::volt);
  
  right_chassis1.spin(fwd, 0.128 * right_power, voltageUnits::volt);
  right_chassis2.spin(fwd, 0.128 * right_power, voltageUnits::volt);
  right_chassis3.spin(fwd, 0.128 * right_power, voltageUnits::volt);
}

void ChassisControl(double left_power, double right_power) {
  left_chassis1.spin(fwd, left_power, voltageUnits::volt);
  left_chassis2.spin(fwd, left_power, voltageUnits::volt);
  left_chassis3.spin(fwd, left_power, voltageUnits::volt);
  
  right_chassis1.spin(fwd, right_power, voltageUnits::volt);
  right_chassis2.spin(fwd, right_power, voltageUnits::volt);
  right_chassis3.spin(fwd, right_power, voltageUnits::volt);
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

double NormalizeTarget(double angle) {
  if (angle - InertialA.rotation() > 180) {
    angle = angle - 360;
  } else if (angle - InertialA.rotation() < -180) {
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

void TurnToAngle(double turn_angle, double time_limit_msec) {
  Stop(vex::brakeType::coast);
  isturning = true;
  double threshold = 1;
  double kp = 0.6 - fabs(turn_angle - GetInertialHeading())/90 * 0.05;
  if(kp > 0.6) {
    kp = 0.6;
  } else if(kp < 0.5) {
    kp = 0.5;
  }
  const double ki = 0;
  double kd = 4.1;
  PID pid = PID(kp, ki, kd);
  
  pid.SetTarget(turn_angle);
  pid.SetIntegralMax(0);  
  pid.SetIntegralRange(5);
  
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
    ChassisControl(output, -output);
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
  int drive_direction = distance_in > 0 ? 1 : -1;
  double kp = 0.68, ki = 0, kd = 6;
  const double kph = 0.3, kih = 0, kdh = 3;

  double max_acceleration = 0.9;
  double max_deceleration = 0.3;
  distance_in = distance_in * drive_direction;
  PID piddistance = PID(kp, ki, kd);
  PID pidh = PID(kph, kih, kdh);

  piddistance.SetTarget(distance_in);
  piddistance.SetIntegralMax(5);  
  piddistance.SetSmallBigErrorTolerance(threshold, threshold * 3);
  piddistance.SetSmallBigErrorDuration(50, 250);
  piddistance.SetDerivativeTolerance(5);
  // 5 Iterations
  
  pidh.SetTarget(correct_angle);
  pidh.SetIntegralMax(0);  
  pidh.SetIntegralRange(0);
  
  pidh.SetSmallBigErrorTolerance(0, 0);
  pidh.SetSmallBigErrorDuration(0, 0);
  pidh.SetDerivativeTolerance(0);
  pidh.SetArrive(false);
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
    if(leftoutput - previous_left_output > max_acceleration) {
      leftoutput = previous_left_output + max_acceleration;
    }
    if(rightoutput - previous_right_output > max_acceleration) {
      rightoutput = previous_right_output + max_acceleration;
    }
    previous_left_output = leftoutput;
    previous_right_output = rightoutput;
    ChassisControl(leftoutput * drive_direction, rightoutput * drive_direction);
    wait(10, msec);
  }
  Stop(vex::hold);
  isturning = false;
}

void Grab(double power) {
  intake_motor.spin(fwd, 0.128 * power, voltageUnits::volt);
}

void Arm(double power) {
  arm_motor.spin(fwd, 0.128 * power, voltageUnits::volt);
}

void Stop(brakeType type) {
  left_chassis1.stop(type);
  left_chassis2.stop(type);
  left_chassis3.stop(type);
  
  right_chassis1.stop(type);
  right_chassis2.stop(type);
  right_chassis3.stop(type);
}

void CurveCircle(double result_angle_deg, double center_radius, double time_limit_msec, bool exit, double max_output) {
  double in_arc, out_arc;
  double real_angle = 0, current_angle = 0;
  double ratio, result_angle;
  result_angle = (result_angle_deg - correct_angle) * 3.14159265359 / 180;
  in_arc = fabs((fabs(center_radius) - (distancebetweenwheels / 2)) * result_angle);
  out_arc = fabs((fabs(center_radius) + (distancebetweenwheels / 2)) * result_angle);
  ratio = in_arc / out_arc;
  ResetChassis();
  Stop(vex::brakeType::coast);
  isturning = true;
  // Tuned parameters, DO NOT CHANGE!!!
  double threshold = 0.5;
  double kp = 0.7, ki = 0.1, kd = 2;
  if(exit == false) {
    if(fabs(center_radius) > 20) {
      kp = 1.8, ki = 0.1, kd = 0;
    } else {
      kp = 1, ki = 0.1, kd = 0;
    }
  }
  const double kpt = 0.7, kit = 0, kdt = 4.1;

  int curve_direction = center_radius > 0 ? 1 : -1;
  int drive_direction = 0;
  if ((curve_direction == 1 && (result_angle_deg - correct_angle) > 0) || (curve_direction == -1 && (result_angle_deg - correct_angle) < 0)) {
    drive_direction = 1;
  } else {
    drive_direction = -1;
  }

  PID pidout = PID(kp, ki, kd);
  PID pidturn = PID(kpt, kit, kdt);

  pidout.SetTarget(out_arc);
  pidout.SetIntegralMax(300);  
  pidout.SetIntegralRange(5);
  pidout.SetSmallBigErrorTolerance(0.3, 0.9);
  pidout.SetSmallBigErrorDuration(50, 250);
  pidout.SetDerivativeTolerance(threshold * 4.5);
  // 5 Iterations
  
  pidturn.SetTarget(0);
  pidturn.SetIntegralMax(100);  
  pidturn.SetIntegralRange(0);
  
  pidturn.SetSmallBigErrorTolerance(0, 0);
  pidturn.SetSmallBigErrorDuration(0, 0);
  pidturn.SetDerivativeTolerance(0);
  pidturn.SetArrive(false);
  // 5 Iterations

  // Reset the chassis.
  ResetChassis();
  double start_time = Brain.timer(msec);
  double left_output = 0, right_output = 0, correction_output = 0;
  double current_right = 0, current_left = 0;
  
  if (curve_direction == -1 && exit == true) {
    while (!pidout.TargetArrived() && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_angle = GetInertialHeading();
      current_right = fabs((GetRightRotationDegree() / 360.0) * wheel_distance_in);
      real_angle = current_right/out_arc * (result_angle_deg - correct_angle) + correct_angle;
      pidturn.SetTarget(real_angle);
      right_output = pidout.Update(current_right);
      left_output = right_output * ratio;
      correction_output = pidturn.Update(current_angle);
      if(drive_direction == 1) {
        right_output = right_output - correction_output;
        left_output = left_output + correction_output;
      } else {
        right_output = right_output + correction_output;
        left_output = left_output - correction_output;
      }

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

      ChassisControl(left_output * drive_direction, right_output * drive_direction);
      wait(10, msec);
    }
  } else if (curve_direction == 1 && exit == true) {
    while (!pidout.TargetArrived() && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_angle = GetInertialHeading();
      current_left = fabs((GetLeftRotationDegree() / 360.0) * wheel_distance_in);
      real_angle = current_left/out_arc * (result_angle_deg - correct_angle) + correct_angle;
      pidturn.SetTarget(real_angle);
      left_output = pidout.Update(current_left);
      right_output = left_output * ratio;
      correction_output = pidturn.Update(current_angle);
      if(drive_direction == 1) {
        left_output = left_output + correction_output;
        right_output = right_output - correction_output;
      } else {
        left_output = left_output - correction_output;
        right_output = right_output + correction_output;
      }

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

      ChassisControl(left_output * drive_direction, right_output * drive_direction);
      wait(10, msec);
    }
  } else if (curve_direction == -1 && exit == false) {
    while (current_right < out_arc && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_angle = GetInertialHeading();
      current_right = fabs((GetRightRotationDegree() / 360.0) * wheel_distance_in);
      real_angle = current_right/out_arc * (result_angle_deg - correct_angle) + correct_angle;
      pidturn.SetTarget(real_angle);
      right_output = pidout.Update(current_right);
      left_output = right_output * ratio;
      correction_output = pidturn.Update(current_angle);
      if(drive_direction == 1) {
        right_output = right_output - correction_output;
        left_output = left_output + correction_output;
      } else {
        right_output = right_output + correction_output;
        left_output = left_output - correction_output;
      }

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

      ChassisControl(left_output * drive_direction, right_output * drive_direction);
      wait(10, msec);
    }
  } else {
    while (current_left < out_arc && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_angle = GetInertialHeading();
      current_left = fabs((GetLeftRotationDegree() / 360.0) * wheel_distance_in);
      real_angle = current_left/out_arc * (result_angle_deg - correct_angle) + correct_angle;
      pidturn.SetTarget(real_angle);
      left_output = pidout.Update(current_left);
      right_output = left_output * ratio;
      correction_output = pidturn.Update(current_angle);
      if(drive_direction == 1) {
        left_output = left_output + correction_output;
        right_output = right_output - correction_output;
      } else {
        left_output = left_output - correction_output;
        right_output = right_output + correction_output;
      }

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

      ChassisControl(left_output * drive_direction, right_output * drive_direction);
      wait(10, msec);
    }
  }
  if(exit == true) {
    Stop(vex::brakeType::hold);
  }
  correct_angle = result_angle_deg;
  isturning = false;
}

void Swing(double swing_angle, double drive_direction, double time_limit_msec, double max_output) {
  Stop(vex::brakeType::coast);
  isturning = true;
  double threshold = 1;
  double kp = 0.6;
  const double ki = 0;
  double kd = 4.1;
  PID pid = PID(kp, ki, kd);
  
  pid.SetTarget(swing_angle);
  pid.SetIntegralMax(1000);  
  pid.SetIntegralRange(5);
  
  pid.SetSmallBigErrorTolerance(threshold, threshold * 3);
  pid.SetSmallBigErrorDuration(100, 600);
  pid.SetDerivativeTolerance(threshold * 4.5);
  // 5 Iterations
  
  // Draw the baseline.
  double draw_amplifier = 230 / fabs(swing_angle);
  Brain.Screen.clearScreen(black);
  Brain.Screen.setPenColor(green);
  Brain.Screen.drawLine(0, fabs(swing_angle) * draw_amplifier, 
                        600, fabs(swing_angle) * draw_amplifier);
  Brain.Screen.setPenColor(red);
  
  // Start the PID loop.
  double start_time = Brain.timer(msec);
  double output;
  double current_heading;
  double previous_heading = 0;
  int index = 1;
  int choice = 1;
  if(swing_angle - correct_angle < 0 && drive_direction == 1) {
    choice = 1;
  } else if(swing_angle - correct_angle > 0 && drive_direction == 1) {
    choice = 2;
  } else if(swing_angle - correct_angle < 0 && drive_direction == -1) {
    choice = 3;
  } else {
    choice = 4;
  }
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

    if(output > max_output) {
      output = max_output;
    } else if(output < -max_output) {
      output = -max_output;
    }

    switch(choice) {
    case 1:
      left_chassis1.stop(hold);
      left_chassis2.stop(hold);
      left_chassis3.stop(hold);
      right_chassis1.spin(fwd, -output * drive_direction, volt);
      right_chassis2.spin(fwd, -output * drive_direction, volt);
      right_chassis3.spin(fwd, -output * drive_direction, volt);
      break;
    case 2:
      left_chassis1.spin(fwd, output * drive_direction, volt);
      left_chassis2.spin(fwd, output * drive_direction, volt);
      left_chassis3.spin(fwd, output * drive_direction, volt);
      right_chassis1.stop(hold);
      right_chassis2.stop(hold);
      right_chassis3.stop(hold);
      break;
    case 3:
      left_chassis1.spin(fwd, -output * drive_direction, volt);
      left_chassis2.spin(fwd, -output * drive_direction, volt);
      left_chassis3.spin(fwd, -output * drive_direction, volt);
      right_chassis1.stop(hold);
      right_chassis2.stop(hold);
      right_chassis3.stop(hold);
      break;
    case 4:
      left_chassis1.stop(hold);
      left_chassis2.stop(hold);
      left_chassis3.stop(hold);
      right_chassis1.spin(fwd, output * drive_direction, volt);
      right_chassis2.spin(fwd, output * drive_direction, volt);
      right_chassis3.spin(fwd, output * drive_direction, volt);
      break;
    }
    wait(10, msec);
  }  
  Stop(vex::hold);
  correct_angle = swing_angle;
  Controller1.Screen.print(GetInertialHeading());
  isturning = false;
}

void pullcatapult() {
  int start_time = Brain.timer(msec);
  catapult_motor.spin(fwd, -100 * 0.128, volt);
  while(Distance5.value() > distance_value) {
    wait(10, msec);
  }
  if(Brain.timer(msec) - start_time > 5000) {
    catapult_motor.stop(coast);
  } else {
    catapult_motor.stop(hold);
  }
}

void catapultlaunch(int times, int interval_msec) {
  int start_time = 0, extra_time = 100;
  for(int i = 0; i < times; i++) {
    start_time = Brain.timer(msec);
    catapult_motor.spin(fwd, -100 * 0.128, volt);
    while(Distance5.value() > distance_value) {
      wait(10, msec);
    }
    if(Brain.timer(msec) - start_time < interval_msec) {
      catapult_motor.stop(hold);
      wait(interval_msec - (Brain.timer(msec) - start_time) - extra_time, msec);
      catapult_motor.spin(fwd, -100 * 0.128, volt);
      extra_time = 0;
      while(Distance5.value() <= distance_value + 2) {
        extra_time += 10;
        wait(10, msec);
      }
    } else {
      catapult_motor.spin(fwd, -100 * 0.128, volt);
      extra_time = 0;
      while(Distance5.value() <= distance_value + 2) {
        extra_time += 10;
        wait(10, msec);
      }
    }
  }
}

void heading_correction() {
  double output = 0;
  const double kp = 0.3;
  const double ki = 0;
  const double kd = 3;
  PID pid = PID(kp, ki, kd);
  
  pid.SetTarget(correct_angle);
  //pid.SetIntegralMax(300);  
  pid.SetIntegralRange(fabs(correct_angle) / 2.5);
  
  pid.SetSmallBigErrorTolerance(0, 0);
  pid.SetSmallBigErrorDuration(0, 0);
  pid.SetDerivativeTolerance(0);
  pid.SetArrive(false);
  // 5 Iterations
  
  // Start the PID loop.
  while(headingcorrection == true) {
    pid.SetTarget(correct_angle);
    if(isturning == false) {
      output = pid.Update(GetInertialHeading());
      ChassisControl(output, -output);
    }
    wait(10, msec);
  }
}

void trackodom() {
  ResetChassis();
  double l = 0, r = 0, dl = 0, dr = 0, a = 0, da = 0, d = 0;
  while(true) {
    da = to_rad(GetInertialHeading()) - a;
    dl = (GetLeftRotationDegree() - l) * wheel_distance_in / 360;
    dr = (GetRightRotationDegree() - r) * wheel_distance_in / 360;
    if(da == 0) {
      d = (dl + dr) / 2;
    } else {
      d = 2 * sin(da / 2) * (dr / da + distancebetweenwheels / 2);
    }
    xpos += d * sin(-da / 2 + a);
    ypos += d * cos(-da / 2 + a);
    a = to_rad(GetInertialHeading());
    l = GetLeftRotationDegree();
    r = GetRightRotationDegree();
    wait(10, msec);
  }
}

void TurnToPoint(double x, double y, int d, double time_limit_msec) {
  Stop(vex::brakeType::coast);
  isturning = true;
  double threshold = 1, add = 0;
  if(d == -1) {
    add = 180;
  }
  double turn_angle = NormalizeTarget(to_deg(atan2(x - xpos, y - ypos)) + add);
  double kp = 0.6 - fabs(turn_angle - GetInertialHeading())/90 * 0.05;
  if(kp > 0.6) {
    kp = 0.6;
  } else if(kp < 0.5) {
    kp = 0.5;
  }
  const double ki = 0;
  double kd = 4.1;
  PID pid = PID(kp, ki, kd);
  
  pid.SetTarget(turn_angle);
  pid.SetIntegralMax(0);  
  pid.SetIntegralRange(5);
  
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
    pid.SetTarget(to_deg(NormalizeTarget(atan2(x - xpos, y - ypos)) + add));
    current_heading = GetInertialHeading();
    output = pid.Update(current_heading);
    
    // Draw line
    Brain.Screen.drawLine(
        index * 3, fabs(previous_heading) * draw_amplifier, 
        (index + 1) * 3, fabs(current_heading * draw_amplifier));
    index++;
    previous_heading = current_heading;
    // End
    ChassisControl(output, -output);
    wait(10, msec);
  }  
  Stop(vex::hold);
  correct_angle = turn_angle;
  cx = x;
  cy = y;
  isturning = false;
}

void MoveToPoint(double x, double y, int dir, double time_limit_msec, bool exit, double max_output) {
  Stop(vex::brakeType::coast);
  isturning = true;
  // Current robot setup, 3.25 inch wheel, 60/36 gear ratio.
  // Tuned parameters, DO NOT CHANGE!!!
  double threshold = 0.5;
  int add = dir > 0 ? 0 : 180;
  double kp = 0.68, ki = 0, kd = 6;
  double kph = 0.3, kih = 0, kdh = 5;
  if(exit == false) {
    kp = 1;
    kph = 0.6;
  }

  double max_acceleration = 0.9;
  double max_deceleration = 0.3;
  PID piddistance = PID(kp, ki, kd);
  PID pidh = PID(kph, kih, kdh);

  piddistance.SetTarget(hypot(x - xpos, y - ypos));
  piddistance.SetIntegralMax(5);  
  piddistance.SetSmallBigErrorTolerance(threshold, threshold * 3);
  piddistance.SetSmallBigErrorDuration(50, 250);
  piddistance.SetDerivativeTolerance(5);
  // 5 Iterations
  
  pidh.SetTarget(NormalizeTarget(to_deg(atan2(x - xpos, y - ypos)) + add));
  pidh.SetIntegralMax(0);  
  pidh.SetIntegralRange(0);
  
  pidh.SetSmallBigErrorTolerance(0, 0);
  pidh.SetSmallBigErrorDuration(0, 0);
  pidh.SetDerivativeTolerance(0);
  pidh.SetArrive(false);
  // 5 Iterations

  // Reset the chassis.
  double start_time = Brain.timer(msec);
  double leftoutput = 0, rightoutput = 0, correction_output = 0, previous_left_output = 0, previous_right_output = 0;

  double current_angle = 0;
  bool ch = true;

  while ((!piddistance.TargetArrived()) && Brain.timer(msec) - start_time <= time_limit_msec) {
    pidh.SetTarget(NormalizeTarget(to_deg(atan2(x - xpos, y - ypos)) + add));
    piddistance.SetTarget(hypot(x - xpos, y - ypos));
    current_angle = GetInertialHeading();
    leftoutput = piddistance.Update(0) * cos(to_rad(atan2(x - xpos, y - ypos) * 180 / M_PI + add - current_angle));
    rightoutput = leftoutput;
    if(hypot(x - xpos, y - ypos) > 4 && ch == true) {
      correction_output = pidh.Update(current_angle);
    } else {
      correction_output = 0;
      ch = false;
      if(exit == false) {
        break;
      }
    }
    if(dir == 1) {
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
    if(leftoutput - previous_left_output > max_acceleration) {
      leftoutput = previous_left_output + max_acceleration;
    }
    if(rightoutput - previous_right_output > max_acceleration) {
      rightoutput = previous_right_output + max_acceleration;
    }
    previous_left_output = leftoutput;
    previous_right_output = rightoutput;
    ChassisControl(leftoutput * dir, rightoutput * dir);
    wait(10, msec);
  }
  if(exit == true) {
    Stop(vex::hold);
  }
  isturning = false;
}

void boomerang(double x, double y, double a, double dlead, double time_limit_msec, bool exit, double max_output) {
  Stop(vex::brakeType::coast);
  isturning = true;
  // Current robot setup, 3.25 inch wheel, 60/36 gear ratio.
  // Tuned parameters, DO NOT CHANGE!!!
  double threshold = 0.5;
  double kp = 1.5, ki = 0, kd = 6;
  double kph = 0.6, kih = 0, kdh = 7;
  if(exit == false) {
    kp = 2;
    kph = 1;
  }

  double max_acceleration = 0.9;
  double max_deceleration = 0.3;
  PID piddistance = PID(kp, ki, kd);
  PID pidh = PID(kph, kih, kdh);

  piddistance.SetTarget(0);
  piddistance.SetIntegralMax(5);  
  piddistance.SetSmallBigErrorTolerance(threshold, threshold * 3);
  piddistance.SetSmallBigErrorDuration(50, 250);
  piddistance.SetDerivativeTolerance(5);
  // 5 Iterations
  
  pidh.SetTarget(GetInertialHeading());
  pidh.SetIntegralMax(0);  
  pidh.SetIntegralRange(0);
  
  pidh.SetSmallBigErrorTolerance(0, 0);
  pidh.SetSmallBigErrorDuration(0, 0);
  pidh.SetDerivativeTolerance(0);
  pidh.SetArrive(false);
  // 5 Iterations

  // Reset the chassis.
  double start_time = Brain.timer(msec);
  double leftoutput = 0, rightoutput = 0, correction_output = 0, previous_left_output = 0, previous_right_output = 0;

  double current_angle = 0, h = 0, cax = 0, cay = 0;
  bool ch = true;

  while ((!piddistance.TargetArrived()) && Brain.timer(msec) - start_time <= time_limit_msec) {
    h = hypot(xpos - x, ypos - y);
    cax = x - h * sin(to_rad(a)) * dlead;
    cay = y - h * cos(to_rad(a)) * dlead;
    pidh.SetTarget(NormalizeTarget(to_deg(atan2(cax - xpos, cay - ypos))));
    piddistance.SetTarget(hypot(cax - xpos, cay - ypos));
    current_angle = GetInertialHeading();
    leftoutput = piddistance.Update(0) * cos(to_rad(atan2(cax - xpos, cay - ypos) * 180 / M_PI - current_angle));
    rightoutput = leftoutput;
    if(hypot(x - xpos, y - ypos) > 3 && ch == true) {
      correction_output = pidh.Update(current_angle);
    } else {
      correction_output = 0;
      ch = false;
      if(exit == false) {
        break;
      }
    }
    leftoutput = leftoutput + correction_output;
    rightoutput = rightoutput - correction_output;

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
    if(leftoutput - previous_left_output > max_acceleration) {
      leftoutput = previous_left_output + max_acceleration;
    }
    if(rightoutput - previous_right_output > max_acceleration) {
      rightoutput = previous_right_output + max_acceleration;
    }
    previous_left_output = leftoutput;
    previous_right_output = rightoutput;
    ChassisControl(leftoutput, rightoutput);
    wait(10, msec);
  }
  if(exit == true) {
    Stop(vex::hold);
  }
  isturning = false;
}