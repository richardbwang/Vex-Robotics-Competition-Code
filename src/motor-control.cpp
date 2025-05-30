#include "vex.h"
#include "utils.h"
#include "pid.h"
#include <ctime>
#include <cmath>

#include "motor-control.h"
#include "autonomous.h"

bool isRed = true;
bool isturning = false;
bool usevelocity = false;
bool headingcorrection = true;
bool spinfw = false;
bool dirchangestart = true;
bool dirchangeend = true;
bool sorting = false;
double targetIntakeVolts = 0;
double min_output = 10;
double dkp = 1.1, dki = 0.1, dkd = 7;
double tkp = 0.3, tki = 0, tkd = 2.5;
double ckp = 0.6, cki = 0, ckd = 4;
double maxslewaccelfwd = 24;
double maxslewdecelfwd = 24;
double maxslewaccelrev = 24;
double maxslewdecelrev = 24;
double prevleftoutput = 0, prevrightoutput = 0;
double xpos = 0, ypos = 0;
double correct_angle = 0, cx = 0, cy = 0;
double distance_value = 35;
double distancebetweenwheels = 12.3;
double cp = 2;
double arm_angle_target = 0, arm_pid_target = 0, arm_load_target = 60, arm_store_target = 250, arm_score_target = 470;
double rushsetupangle = -23;

void ChassisControl(double left_power, double right_power) {
  left_chassis1.spin(fwd, left_power, voltageUnits::volt);
  left_chassis2.spin(fwd, left_power, voltageUnits::volt);
  left_chassis3.spin(fwd, left_power, voltageUnits::volt);
  
  right_chassis1.spin(fwd, right_power, voltageUnits::volt);
  right_chassis2.spin(fwd, right_power, voltageUnits::volt);
  right_chassis3.spin(fwd, right_power, voltageUnits::volt);
}

void intake(double inpower) {
  targetIntakeVolts = inpower;
  intake_motor.spin(fwd, inpower, voltageUnits::volt);
}

void intakeStuck() {
  int i = 0;
  while(true) {
    if(targetIntakeVolts > 0 && intake_motor.velocity(rpm) <= 5) {
      i++;
    } else {
      i = 0;
    }
    if(i > 40) {
      int prevTarget = targetIntakeVolts;
      intake(-12);
      targetIntakeVolts = prevTarget;
      wait(200, msec);
      intake(targetIntakeVolts);
      wait(100, msec);
    }
    wait(10, msec);
  }
}

void arm(double armpower) {
  arm_motor.spin(fwd, armpower, volt);
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
  return (left_chassis1.position(degrees) + left_chassis2.position(degrees) + left_chassis3.position(degrees)) / 3.0;
}

double GetRightRotationDegree() {
  return (right_chassis1.position(degrees) + right_chassis2.position(degrees) + right_chassis3.position(degrees)) / 3.0;
}

//================================Chassis Movement=========================================//
double NormalizeAngle(double angle) {
  if (angle > 180) {
    while (angle > 180) {
      angle = angle - 360;
    }
  } else if (angle < -180) {
    while (angle < -180) {
      angle = angle + 360;
    }
  }

  return angle;
}

double NormalizeTarget(double angle) {
  if (angle - GetInertialHeading() > 180) {
    while (angle - GetInertialHeading() > 180) {
      angle = angle - 360;
    }
  } else if (angle - GetInertialHeading() < -180) {
    while (angle - GetInertialHeading() < -180) {
      angle = angle + 360;
    }
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

void TurnToAngle(double turn_angle, double time_limit_msec, bool exit, double max_output) {
  turn_angle *= (isRed ? 1 : -1);
  Stop(vex::brakeType::coast);
  isturning = true;
  double threshold = 1;
  PID pid = PID(tkp, tki, tkd);
  
  turn_angle = NormalizeTarget(turn_angle);
  pid.SetTarget(turn_angle);
  pid.SetIntegralMax(0);  
  pid.SetIntegralRange(3);
  
  pid.SetSmallBigErrorTolerance(threshold, threshold * 3);
  pid.SetSmallBigErrorDuration(50, 250);
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
  double current_heading = GetInertialHeading();
  double previous_heading = 0;
  int index = 1;
  if(exit == false && correct_angle < turn_angle) {
    while (GetInertialHeading() < turn_angle && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_heading = GetInertialHeading();
      output = pid.Update(current_heading);
      
      // Draw line
      Brain.Screen.drawLine(
          index * 3, fabs(previous_heading) * draw_amplifier, 
          (index + 1) * 3, fabs(current_heading * draw_amplifier));
      index++;
      previous_heading = current_heading;
      if(output < min_output) {
        output = min_output;
      }
      if(output > max_output) {
        output = max_output;
      } else if(output < -max_output) {
        output = -max_output;
      }
      // End
      ChassisControl(output, -output);
      wait(10, msec);
    }
  } else if(exit == false && correct_angle > turn_angle) {
    while (GetInertialHeading() > turn_angle && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_heading = GetInertialHeading();
      output = pid.Update(current_heading);
      
      // Draw line
      Brain.Screen.drawLine(
          index * 3, fabs(previous_heading) * draw_amplifier, 
          (index + 1) * 3, fabs(current_heading * draw_amplifier));
      index++;
      previous_heading = current_heading;
      if(output < min_output) {
        output = min_output;
      }
      if(output > max_output) {
        output = max_output;
      } else if(output < -max_output) {
        output = -max_output;
      }
      // End
      ChassisControl(-output, output);
      wait(10, msec);
    }
  } else {
    while (!pid.TargetArrived() && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_heading = GetInertialHeading();
      output = pid.Update(current_heading);
      
      // Draw line
      Brain.Screen.drawLine(
          index * 3, fabs(previous_heading) * draw_amplifier, 
          (index + 1) * 3, fabs(current_heading * draw_amplifier));
      index++;
      previous_heading = current_heading;
      if(output > max_output) {
        output = max_output;
      } else if(output < -max_output) {
        output = -max_output;
      }
      // End
      ChassisControl(output, -output);
      wait(10, msec);
    }
  }
  if(exit) {
    Stop(vex::hold);
  }
  correct_angle = turn_angle * (isRed ? 1 : -1);
  isturning = false;
}


void TurnToAngleSetup(double turn_angle, double max_output) {
  Stop(vex::brakeType::coast);
  isturning = true;
  double threshold = 1;
  PID pid = PID(tkp, tki, tkd);
  
  turn_angle = NormalizeTarget(turn_angle);
  pid.SetTarget(turn_angle);
  pid.SetIntegralMax(0);  
  pid.SetIntegralRange(3);
  
  pid.SetSmallBigErrorTolerance(threshold, threshold * 3);
  pid.SetSmallBigErrorDuration(50, 250);
  pid.SetDerivativeTolerance(threshold * 4.5);
  pid.SetArrive(false);
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
  double current_heading = GetInertialHeading();
  double previous_heading = 0;
  int index = 1;
    while (!pid.TargetArrived()) {
      current_heading = GetInertialHeading();
      output = pid.Update(current_heading);
      
      // Draw line
      Brain.Screen.drawLine(
          index * 3, fabs(previous_heading) * draw_amplifier, 
          (index + 1) * 3, fabs(current_heading * draw_amplifier));
      index++;
      previous_heading = current_heading;
      if(output > max_output) {
        output = max_output;
      } else if(output < -max_output) {
        output = -max_output;
      }
      // End
      ChassisControl(output, -output);
      wait(10, msec);
    }
  isturning = false;
}

void DriveTo(double distance_in, double time_limit_msec, bool exit, double max_output) {
  correct_angle *= (isRed ? 1 : -1);
  double startl = GetLeftRotationDegree(), startr = GetRightRotationDegree();
  Stop(vex::brakeType::coast);
  isturning = true;
  // Current robot setup, 3.25 inch wheel, 60/36 gear ratio.
  // Tuned parameters, DO NOT CHANGE!!!
  double threshold = 0.5;
  int drive_direction = distance_in > 0 ? 1 : -1;
  double maxslewfwd = drive_direction > 0 ? maxslewaccelfwd : maxslewdecelrev;
  double maxslewrev = drive_direction > 0 ? maxslewdecelfwd : maxslewaccelrev;
  bool minspeed = false;;
  if(!exit) {
    if(!dirchangestart && dirchangeend) {
      maxslewfwd = drive_direction > 0 ? 24 : maxslewdecelrev;
      maxslewrev = drive_direction > 0 ? maxslewdecelfwd : 24;
    }
    if(dirchangestart && !dirchangeend) {
      maxslewfwd = drive_direction > 0 ? maxslewaccelfwd : 24;
      maxslewrev = drive_direction > 0 ? 24 : maxslewaccelrev;
      minspeed = true;
    }
    if(!dirchangestart && !dirchangeend) {
      maxslewfwd = 24;
      maxslewrev = 24;
      minspeed = true;
    }
  }

  distance_in = distance_in * drive_direction;
  PID piddistance = PID(dkp, dki, dkd);
  PID pidh = PID(ckp, cki, ckd);

  piddistance.SetTarget(distance_in);
  piddistance.SetIntegralMax(3);  
  piddistance.SetSmallBigErrorTolerance(threshold, threshold * 3);
  piddistance.SetSmallBigErrorDuration(50, 250);
  piddistance.SetDerivativeTolerance(5);
  // 5 Iterations
  
  pidh.SetTarget(NormalizeTarget(correct_angle));
  pidh.SetIntegralMax(0);  
  pidh.SetIntegralRange(1);
  
  pidh.SetSmallBigErrorTolerance(0, 0);
  pidh.SetSmallBigErrorDuration(0, 0);
  pidh.SetDerivativeTolerance(0);
  pidh.SetArrive(false);
  // 5 Iterations

  // Reset the chassis.
  double start_time = Brain.timer(msec);
  double leftoutput = 0, rightoutput = 0, correction_output = 0;

  double current_distance = 0, current_angle = 0;

  while (((!piddistance.TargetArrived()) && Brain.timer(msec) - start_time <= time_limit_msec && exit) || (exit == false && current_distance < distance_in && Brain.timer(msec) - start_time <= time_limit_msec)) {
    current_distance = (fabs(((GetLeftRotationDegree() - startl) / 360.0) * wheel_distance_in) + fabs(((GetRightRotationDegree() - startr) / 360.0) * wheel_distance_in)) / 2;
    current_angle = GetInertialHeading();
    leftoutput = piddistance.Update(current_distance) * drive_direction;
    rightoutput = leftoutput;
    correction_output = pidh.Update(current_angle);

    //Minimum Output Check
    if(minspeed) {
      if(fabs(leftoutput) <= fabs(rightoutput) && leftoutput < min_output && leftoutput > 0) {
        rightoutput = rightoutput / leftoutput * min_output;
        leftoutput = min_output;
      } else if(fabs(rightoutput) < fabs(leftoutput) && rightoutput < min_output && rightoutput > 0) {
        leftoutput = leftoutput / rightoutput * min_output;
        rightoutput = min_output;
      }
    }
    if(!exit) {
      leftoutput = 24 * drive_direction;
      rightoutput = 24 * drive_direction;
    }

    leftoutput = leftoutput + correction_output;
    rightoutput = rightoutput - correction_output;

    //Max Output Check
    if(fabs(leftoutput) >= fabs(rightoutput) && leftoutput > max_output) {
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
    if(prevleftoutput - leftoutput > maxslewrev) {
      leftoutput = prevleftoutput - maxslewrev;
    }
    if(prevrightoutput - rightoutput > maxslewrev) {
      rightoutput = prevrightoutput - maxslewrev;
    }
    if(leftoutput - prevleftoutput > maxslewfwd) {
      leftoutput = prevleftoutput + maxslewfwd;
    }
    if(rightoutput - prevrightoutput > maxslewfwd) {
      rightoutput = prevrightoutput + maxslewfwd;
    }
    prevleftoutput = leftoutput;
    prevrightoutput = rightoutput;
    ChassisControl(leftoutput, rightoutput);
    wait(10, msec);
  }
  if(exit) {
    prevleftoutput = 0;
    prevrightoutput = 0;
    Stop(vex::hold);
  }
  correct_angle *= (isRed ? 1 : -1);
  isturning = false;
}

void DriveToRush(double distance_in, double time_limit_msec, bool exit, double max_output) {
  double startl = GetLeftRotationDegree(), startr = GetRightRotationDegree();
  Stop(vex::brakeType::coast);
  isturning = true;
  // Current robot setup, 3.25 inch wheel, 60/36 gear ratio.
  // Tuned parameters, DO NOT CHANGE!!!
  double threshold = 0.5;
  int drive_direction = distance_in > 0 ? 1 : -1;
  double maxslewfwd = drive_direction > 0 ? maxslewaccelfwd : maxslewdecelrev;
  double maxslewrev = drive_direction > 0 ? maxslewdecelfwd : maxslewaccelrev;
  bool minspeed = false;;
  if(!exit) {
    if(!dirchangestart && dirchangeend) {
      maxslewfwd = drive_direction > 0 ? 24 : maxslewdecelrev;
      maxslewrev = drive_direction > 0 ? maxslewdecelfwd : 24;
    }
    if(dirchangestart && !dirchangeend) {
      maxslewfwd = drive_direction > 0 ? maxslewaccelfwd : 24;
      maxslewrev = drive_direction > 0 ? 24 : maxslewaccelrev;
      minspeed = true;
    }
    if(!dirchangestart && !dirchangeend) {
      maxslewfwd = 24;
      maxslewrev = 24;
      minspeed = true;
    }
  }

  distance_in = distance_in * drive_direction;
  PID piddistance = PID(dkp, dki, dkd);
  PID pidh = PID(ckp, cki, ckd);

  piddistance.SetTarget(distance_in);
  piddistance.SetIntegralMax(3);  
  piddistance.SetSmallBigErrorTolerance(threshold, threshold * 3);
  piddistance.SetSmallBigErrorDuration(50, 250);
  piddistance.SetDerivativeTolerance(5);
  // 5 Iterations
  
  pidh.SetTarget(NormalizeTarget(correct_angle));
  pidh.SetIntegralMax(0);  
  pidh.SetIntegralRange(1);
  
  pidh.SetSmallBigErrorTolerance(0, 0);
  pidh.SetSmallBigErrorDuration(0, 0);
  pidh.SetDerivativeTolerance(0);
  pidh.SetArrive(false);
  // 5 Iterations

  // Reset the chassis.
  double start_time = Brain.timer(msec);
  double leftoutput = 0, rightoutput = 0, correction_output = 0;

  double current_distance = 0, current_angle = 0;
  double temp_min_output = 2;

  while (((!piddistance.TargetArrived()) && Brain.timer(msec) - start_time <= time_limit_msec && exit) || (exit == false && current_distance < distance_in && Brain.timer(msec) - start_time <= time_limit_msec)) {
    current_distance = (fabs(((GetLeftRotationDegree() - startl) / 360.0) * wheel_distance_in) + fabs(((GetRightRotationDegree() - startr) / 360.0) * wheel_distance_in)) / 2;
    current_angle = GetInertialHeading();
    leftoutput = piddistance.Update(current_distance) * drive_direction;
    rightoutput = leftoutput;
    correction_output = pidh.Update(current_angle);

    //Minimum Output Check
    if(minspeed) {
      if(fabs(leftoutput) <= fabs(rightoutput) && leftoutput < temp_min_output && leftoutput > 0) {
        rightoutput = rightoutput / leftoutput * temp_min_output;
        leftoutput = temp_min_output;
      } else if(fabs(rightoutput) < fabs(leftoutput) && rightoutput < temp_min_output && rightoutput > 0) {
        leftoutput = leftoutput / rightoutput * temp_min_output;
        rightoutput = temp_min_output;
      }
    }

    leftoutput = leftoutput + correction_output;
    rightoutput = rightoutput - correction_output;

    //Max Output Check
    if(fabs(leftoutput) >= fabs(rightoutput) && leftoutput > max_output) {
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
    if(prevleftoutput - leftoutput > maxslewrev) {
      leftoutput = prevleftoutput - maxslewrev;
    }
    if(prevrightoutput - rightoutput > maxslewrev) {
      rightoutput = prevrightoutput - maxslewrev;
    }
    if(leftoutput - prevleftoutput > maxslewfwd) {
      leftoutput = prevleftoutput + maxslewfwd;
    }
    if(rightoutput - prevrightoutput > maxslewfwd) {
      rightoutput = prevrightoutput + maxslewfwd;
    }
    prevleftoutput = leftoutput;
    prevrightoutput = rightoutput;
    ChassisControl(leftoutput, rightoutput);
    wait(10, msec);
  }
  if(exit) {
    prevleftoutput = 0;
    prevrightoutput = 0;
    Stop(vex::hold);
  }
  isturning = false;
}

void DriveToPitch(double speed, double pitch, double time_limit_msec) {
  Stop(vex::brakeType::coast);
  isturning = true;
  PID pidh = PID(ckp, cki, ckd);
  // 5 Iterations
  
  pidh.SetTarget(GetInertialHeading());
  pidh.SetIntegralMax(0);  
  pidh.SetIntegralRange(1);
  
  pidh.SetSmallBigErrorTolerance(0, 0);
  pidh.SetSmallBigErrorDuration(0, 0);
  pidh.SetDerivativeTolerance(0);
  pidh.SetArrive(false);
  // 5 Iterations

  // Reset the chassis.
  double start_time = Brain.timer(msec);
  double leftoutput = 0, rightoutput = 0, correction_output = 0;

  while (InertialA.pitch() < pitch && Brain.timer(msec) - start_time <= time_limit_msec) {
    leftoutput = speed;
    rightoutput = leftoutput;
    correction_output = pidh.Update(GetInertialHeading());

    leftoutput = leftoutput + correction_output;
    rightoutput = rightoutput - correction_output;
    ChassisControl(leftoutput, rightoutput);
    wait(10, msec);
  }
  isturning = false;
}

void DriveToGoal(double distance_in, int dir, double time_limit_msec) {
  double startl = GetLeftRotationDegree(), startr = GetRightRotationDegree();
  Stop(vex::brakeType::coast);
  isturning = true;
  // Current robot setup, 3.25 inch wheel, 60/36 gear ratio.

  PID pidh = PID(ckp, cki, ckd);
  
  pidh.SetTarget(NormalizeTarget(correct_angle));
  pidh.SetIntegralMax(0);  
  pidh.SetIntegralRange(1);
  
  pidh.SetSmallBigErrorTolerance(0, 0);
  pidh.SetSmallBigErrorDuration(0, 0);
  pidh.SetDerivativeTolerance(0);
  pidh.SetArrive(false);

  double start_time = Brain.timer(msec);
  double leftoutput = 0, rightoutput = 0, correction_output = 0, current_distance = 0;

  double current_angle = 0;
  double max_output = 12;
  int i = 0;
  int atgoal = 0;

  while ((atgoal < 3 && Brain.timer(msec) - start_time <= time_limit_msec) || i < 6 || current_distance < distance_in) {
    current_distance = (fabs(((GetLeftRotationDegree() - startl) / 360.0) * wheel_distance_in) + fabs(((GetRightRotationDegree() - startr) / 360.0) * wheel_distance_in)) / 2;
    current_angle = GetInertialHeading();
    leftoutput = 50 * dir;
    rightoutput = 50 * dir;
    double avgvelocity = (fabs(left_chassis1.velocity(rpm)) + fabs(left_chassis2.velocity(rpm)) + fabs(left_chassis3.velocity(rpm)) )
    + fabs(right_chassis1.velocity(rpm)) + fabs(right_chassis2.velocity(rpm)) + fabs(right_chassis3.velocity(rpm)) / 6.0;
    correction_output = pidh.Update(current_angle);

    if(avgvelocity < 50) {
      atgoal++;
    } else {
      atgoal = 0;
    }

    leftoutput = leftoutput + correction_output;
    rightoutput = rightoutput - correction_output;

    //Max Output Check
    if(fabs(leftoutput) >= fabs(rightoutput) && leftoutput > max_output) {
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
    ChassisControl(leftoutput, rightoutput);
    wait(10, msec);
    i++;
  }
  Stop(vex::hold);
  isturning = false;
}


void Stop(brakeType type) {
  left_chassis1.stop(type);
  left_chassis2.stop(type);
  left_chassis3.stop(type);
  
  right_chassis1.stop(type);
  right_chassis2.stop(type);
  right_chassis3.stop(type);
}

void intake_stop(brakeType type) {
  targetIntakeVolts = 0;
  intake_motor.stop(type);
}

void CurveCircle(double result_angle_deg, double center_radius, double time_limit_msec, bool exit, double max_output) {
  double startr = GetRightRotationDegree(), startl = GetLeftRotationDegree();
  double in_arc, out_arc;
  double real_angle = 0, current_angle = 0;
  double ratio, result_angle;
  result_angle_deg = NormalizeAngle(result_angle_deg);
  result_angle = (result_angle_deg - correct_angle) * 3.14159265359 / 180;
  in_arc = fabs((fabs(center_radius) - (distancebetweenwheels / 2)) * result_angle);
  out_arc = fabs((fabs(center_radius) + (distancebetweenwheels / 2)) * result_angle);
  ratio = in_arc / out_arc;
  Stop(vex::brakeType::coast);
  isturning = true;
  // Tuned parameters, DO NOT CHANGE!!!
  double threshold = 0.5;

  int curve_direction = center_radius > 0 ? 1 : -1;
  int drive_direction = 0;
  if ((curve_direction == 1 && (result_angle_deg - correct_angle) > 0) || (curve_direction == -1 && (result_angle_deg - correct_angle) < 0)) {
    drive_direction = 1;
  } else {
    drive_direction = -1;
  }
  double maxslewfwd = drive_direction > 0 ? maxslewaccelfwd : maxslewdecelrev;
  double maxslewrev = drive_direction > 0 ? maxslewdecelfwd : maxslewaccelrev;
  bool minspeed = false;
  if(!exit) {
    if(!dirchangestart && dirchangeend) {
      maxslewfwd = drive_direction > 0 ? 24 : maxslewdecelrev;
      maxslewrev = drive_direction > 0 ? maxslewdecelfwd : 24;
    }
    if(dirchangestart && !dirchangeend) {
      maxslewfwd = drive_direction > 0 ? maxslewaccelfwd : 24;
      maxslewrev = drive_direction > 0 ? 24 : maxslewaccelrev;
      minspeed = true;
    }
    if(!dirchangestart && !dirchangeend) {
      maxslewfwd = 24;
      maxslewrev = 24;
      minspeed = true;
    }
  }

  PID pidout = PID(dkp, dki, dkd);
  PID pidturn = PID(ckp, cki, ckd);

  pidout.SetTarget(out_arc);
  pidout.SetIntegralMax(0);  
  pidout.SetIntegralRange(5);
  pidout.SetSmallBigErrorTolerance(0.3, 0.9);
  pidout.SetSmallBigErrorDuration(50, 250);
  pidout.SetDerivativeTolerance(threshold * 4.5);
  // 5 Iterations
  
  pidturn.SetTarget(0);
  pidturn.SetIntegralMax(0);  
  pidturn.SetIntegralRange(1);
  
  pidturn.SetSmallBigErrorTolerance(0, 0);
  pidturn.SetSmallBigErrorDuration(0, 0);
  pidturn.SetDerivativeTolerance(0);
  pidturn.SetArrive(false);
  // 5 Iterations

  // Reset the chassis.
  double start_time = Brain.timer(msec);
  double left_output = 0, right_output = 0, correction_output = 0;
  double current_right = 0, current_left = 0;
  
  if (curve_direction == -1 && exit == true) {
    while (!pidout.TargetArrived() && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_angle = GetInertialHeading();
      current_right = fabs(((GetRightRotationDegree() - startr) / 360.0) * wheel_distance_in);
      real_angle = current_right/out_arc * (result_angle_deg - correct_angle) + correct_angle;
      pidturn.SetTarget(NormalizeTarget(real_angle));
      right_output = pidout.Update(current_right) * drive_direction;
      left_output = right_output * ratio;
      correction_output = pidturn.Update(current_angle);

      //Minimum Output Check
      if(minspeed) {
        if(fabs(left_output) <= fabs(right_output) && left_output < min_output && left_output > 0) {
          right_output = right_output / left_output * min_output;
          left_output = min_output;
        } else if(fabs(right_output) < fabs(left_output) && right_output < min_output && right_output > 0) {
          left_output = left_output / right_output * min_output;
          right_output = min_output;
        } else if(fabs(left_output) <= fabs(right_output) && left_output > -min_output && left_output < 0) {
          right_output = right_output / left_output * -min_output;
          left_output = -min_output;
        } else if(fabs(right_output) < fabs(left_output) && right_output > -min_output && right_output < 0) {
          left_output = left_output / right_output * -min_output;
          right_output = -min_output;
        }
      }

      left_output = left_output + correction_output;
      right_output = right_output - correction_output;

      //Max Output Check
      if(fabs(left_output) >= fabs(right_output) && left_output > max_output) {
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

      ChassisControl(left_output, right_output);
      wait(10, msec);
    }
  } else if (curve_direction == 1 && exit == true) {
    while (!pidout.TargetArrived() && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_angle = GetInertialHeading();
      current_left = fabs(((GetLeftRotationDegree() - startl) / 360.0) * wheel_distance_in);
      real_angle = current_left/out_arc * (result_angle_deg - correct_angle) + correct_angle;
      pidturn.SetTarget(NormalizeTarget(real_angle));
      left_output = pidout.Update(current_left) * drive_direction;
      right_output = left_output * ratio;
      correction_output = pidturn.Update(current_angle);

      //Minimum Output Check
      if(minspeed) {
        if(fabs(left_output) <= fabs(right_output) && left_output < min_output && left_output > 0) {
          right_output = right_output / left_output * min_output;
          left_output = min_output;
        } else if(fabs(right_output) < fabs(left_output) && right_output < min_output && right_output > 0) {
          left_output = left_output / right_output * min_output;
          right_output = min_output;
        } else if(fabs(left_output) <= fabs(right_output) && left_output > -min_output && left_output < 0) {
          right_output = right_output / left_output * -min_output;
          left_output = -min_output;
        } else if(fabs(right_output) < fabs(left_output) && right_output > -min_output && right_output < 0) {
          left_output = left_output / right_output * -min_output;
          right_output = -min_output;
        }
      }

      left_output = left_output + correction_output;
      right_output = right_output - correction_output;

      //Max Output Check
      if(fabs(left_output) >= fabs(right_output) && left_output > max_output) {
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

      ChassisControl(left_output, right_output);
      wait(10, msec);
    }
  } else if (curve_direction == -1 && exit == false) {
    while (current_right < out_arc && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_angle = GetInertialHeading();
      current_right = fabs(((GetRightRotationDegree() - startr) / 360.0) * wheel_distance_in);
      real_angle = current_right/out_arc * (result_angle_deg - correct_angle) + correct_angle;
      pidturn.SetTarget(NormalizeTarget(real_angle));
      right_output = pidout.Update(current_right) * drive_direction;
      left_output = right_output * ratio;
      correction_output = pidturn.Update(current_angle);

      //Minimum Output Check
      if(minspeed) {
        if(fabs(left_output) <= fabs(right_output) && left_output < min_output && left_output > 0) {
          right_output = right_output / left_output * min_output;
          left_output = min_output;
        } else if(fabs(right_output) < fabs(left_output) && right_output < min_output && right_output > 0) {
          left_output = left_output / right_output * min_output;
          right_output = min_output;
        } else if(fabs(left_output) <= fabs(right_output) && left_output > -min_output && left_output < 0) {
          right_output = right_output / left_output * -min_output;
          left_output = -min_output;
        } else if(fabs(right_output) < fabs(left_output) && right_output > -min_output && right_output < 0) {
          left_output = left_output / right_output * -min_output;
          right_output = -min_output;
        }
      }
      
      left_output = left_output + correction_output;
      right_output = right_output - correction_output;

      //Max Output Check
      if(fabs(left_output) >= fabs(right_output) && left_output > max_output) {
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

      ChassisControl(left_output, right_output);
      wait(10, msec);
    }
  } else {
    while (current_left < out_arc && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_angle = GetInertialHeading();
      current_left = fabs(((GetLeftRotationDegree() - startl) / 360.0) * wheel_distance_in);
      real_angle = current_left/out_arc * (result_angle_deg - correct_angle) + correct_angle;
      pidturn.SetTarget(NormalizeTarget(real_angle));
      left_output = pidout.Update(current_left) * drive_direction;
      right_output = left_output * ratio;
      correction_output = pidturn.Update(current_angle);

      //Minimum Output Check
      if(minspeed) {
        if(fabs(left_output) <= fabs(right_output) && left_output < min_output && left_output > 0) {
          right_output = right_output / left_output * min_output;
          left_output = min_output;
        } else if(fabs(right_output) < fabs(left_output) && right_output < min_output && right_output > 0) {
          left_output = left_output / right_output * min_output;
          right_output = min_output;
        } else if(fabs(left_output) <= fabs(right_output) && left_output > -min_output && left_output < 0) {
          right_output = right_output / left_output * -min_output;
          left_output = -min_output;
        } else if(fabs(right_output) < fabs(left_output) && right_output > -min_output && right_output < 0) {
          left_output = left_output / right_output * -min_output;
          right_output = -min_output;
        }
      }

      left_output = left_output + correction_output;
      right_output = right_output - correction_output;

      //Max Output Check
      if(fabs(left_output) >= fabs(right_output) && left_output > max_output) {
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

      ChassisControl(left_output, right_output);
      wait(10, msec);
    }
  }
  if(exit == true) {
    Stop(vex::brakeType::hold);
  }
  correct_angle = result_angle_deg;
  isturning = false;
}

void Swing(double swing_angle, double drive_direction, double time_limit_msec, bool exit, double max_output) {
  swing_angle *= (isRed ? 1 : -1);
  correct_angle *= (isRed ? 1 : -1);
  Stop(vex::brakeType::coast);
  isturning = true;
  double threshold = 1;
  PID pid = PID(tkp, tki, tkd);
  
  swing_angle = NormalizeTarget(swing_angle);
  pid.SetTarget(swing_angle);
  pid.SetIntegralMax(0);  
  pid.SetIntegralRange(5);
  
  pid.SetSmallBigErrorTolerance(threshold, threshold * 3);
  pid.SetSmallBigErrorDuration(50, 250);
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
  double current_heading = correct_angle;
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
  if(choice == 1 && exit == false) {
    while (current_heading > swing_angle && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_heading = GetInertialHeading();
      output = pid.Update(current_heading);
      
      // Draw line
      Brain.Screen.drawLine(
          index * 3, fabs(previous_heading) * draw_amplifier, 
          (index + 1) * 3, fabs(current_heading * draw_amplifier));
      index++;
      previous_heading = current_heading;
      // End
      if(output < min_output) {
        output = min_output;
      }
      if(output > max_output) {
        output = max_output;
      } else if(output < -max_output) {
        output = -max_output;
      }

      left_chassis1.stop(hold);
      left_chassis2.stop(hold);
      left_chassis3.stop(hold);
      right_chassis1.spin(fwd, output * drive_direction, volt);
      right_chassis2.spin(fwd, output * drive_direction, volt);
      right_chassis3.spin(fwd, output * drive_direction, volt);
      wait(10, msec);
    }
  } else if(choice == 2 && exit == false) {
    while (current_heading < swing_angle && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_heading = GetInertialHeading();
      output = pid.Update(current_heading);
      
      // Draw line
      Brain.Screen.drawLine(
          index * 3, fabs(previous_heading) * draw_amplifier, 
          (index + 1) * 3, fabs(current_heading * draw_amplifier));
      index++;
      previous_heading = current_heading;
      // End
      if(output < min_output) {
        output = min_output;
      }
      if(output > max_output) {
        output = max_output;
      } else if(output < -max_output) {
        output = -max_output;
      }

      left_chassis1.spin(fwd, output * drive_direction, volt);
      left_chassis2.spin(fwd, output * drive_direction, volt);
      left_chassis3.spin(fwd, output * drive_direction, volt);
      right_chassis1.stop(hold);
      right_chassis2.stop(hold);
      right_chassis3.stop(hold);
      wait(10, msec);
    }
  } else if(choice == 3 && exit == false) {
    while (current_heading > swing_angle && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_heading = GetInertialHeading();
      output = pid.Update(current_heading);
      
      // Draw line
      Brain.Screen.drawLine(
          index * 3, fabs(previous_heading) * draw_amplifier, 
          (index + 1) * 3, fabs(current_heading * draw_amplifier));
      index++;
      previous_heading = current_heading;
      // End
      if(output < min_output) {
        output = min_output;
      }
      if(output > max_output) {
        output = max_output;
      } else if(output < -max_output) {
        output = -max_output;
      }

      left_chassis1.spin(fwd, output * drive_direction, volt);
      left_chassis2.spin(fwd, output * drive_direction, volt);
      left_chassis3.spin(fwd, output * drive_direction, volt);
      right_chassis1.stop(hold);
      right_chassis2.stop(hold);
      right_chassis3.stop(hold);
      wait(10, msec);
    }
  } else {
    while (current_heading < swing_angle && Brain.timer(msec) - start_time <= time_limit_msec && exit == false) {
      current_heading = GetInertialHeading();
      output = pid.Update(current_heading);
      
      // Draw line
      Brain.Screen.drawLine(
          index * 3, fabs(previous_heading) * draw_amplifier, 
          (index + 1) * 3, fabs(current_heading * draw_amplifier));
      index++;
      previous_heading = current_heading;
      // End
      if(output < min_output) {
        output = min_output;
      }
      if(output > max_output) {
        output = max_output;
      } else if(output < -max_output) {
        output = -max_output;
      }

      left_chassis1.stop(hold);
      left_chassis2.stop(hold);
      left_chassis3.stop(hold);
      right_chassis1.spin(fwd, output * drive_direction, volt);
      right_chassis2.spin(fwd, output * drive_direction, volt);
      right_chassis3.spin(fwd, output * drive_direction, volt);
      wait(10, msec);
    }
  }
  while (!pid.TargetArrived() && Brain.timer(msec) - start_time <= time_limit_msec && exit == true) {
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
  if(exit == true) {
    Stop(vex::hold);
  }
  correct_angle = swing_angle * (isRed ? 1 : -1);
  isturning = false;
}

void heading_correction() {
  double output = 0;
  PID pid = PID(ckp, cki, ckd);
  
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
  double l = 0, r = 0, dl = 0, dr = 0, a = 0, da = 0, dlyl = 0, dlyr = 0, dly = 0;
  while(true) {
    da = to_rad(GetInertialHeading()) - a;
    dl = (GetLeftRotationDegree() - l) * wheel_distance_in / 360.0;
    dr = (GetRightRotationDegree() - r) * wheel_distance_in / 360.0;
    da = 0;
    if(da == 0) {
      dly = (dl + dr) / 2.0;
    } else {
      dlyl = 2.0 * sin(da / 2.0) * (dl / da + distancebetweenwheels / 2.0);
      dlyr = 2.0 * sin(da / 2.0) * (dr / da + distancebetweenwheels / 2.0);
      dly = (dlyl + dlyr) / 2.0;
    }
    xpos += dly * sin(a + da / 2.0);
    ypos += dly * cos(a + da / 2.0);
    a = to_rad(GetInertialHeading());
    l = GetLeftRotationDegree();
    r = GetRightRotationDegree();
    wait(10, msec);
  }
}

void trackodomwheel() {
  xpos = 0;
  ypos = 0;
  ResetChassis();
  double previousHeading = to_rad(GetInertialHeading());
  double previousX = X.position(degrees);
  double previousY = Y.position(degrees);
  double SidewaysTracker_center_distance = 2.71875;
  // (13-1/8)/2 , 6.5+1/16
  double ForwardTracker_center_distance = -0.03125;
  double Forward_tracker_diameter = 1.97;
  double Sideways_tracker_diameter = 1.975;

  while(true) {
    double currentX = X.position(degrees);
    double currentY = Y.position(degrees);
    double Forward_delta = (currentY - previousY) * Forward_tracker_diameter * M_PI  / 360.0;
    double Sideways_delta = (currentX - previousX) * Sideways_tracker_diameter * M_PI / 360.0;
    double newHeading = to_rad(GetInertialHeading());
    double orientation_delta_rad = newHeading - previousHeading;
    previousX = currentX;
    previousY = currentY;

    double local_X_position;
    double local_Y_position;

    if (fabs(orientation_delta_rad) < 1e-9) {
      local_X_position = Sideways_delta;
      local_Y_position = Forward_delta;
    } else {
      double sincalc= 2 * sin(orientation_delta_rad / 2);
      local_X_position = sincalc * ((Sideways_delta/orientation_delta_rad) + SidewaysTracker_center_distance); 
      local_Y_position = sincalc * ((Forward_delta/orientation_delta_rad) + ForwardTracker_center_distance);
    }

    double local_polar_angle;
    double local_polar_length;

    if (fabs(local_X_position) < 1e-9 && fabs(local_Y_position) < 1e-9){
      local_polar_angle = 0;
      local_polar_length = 0;
    } else {
      local_polar_angle = atan2(local_Y_position, local_X_position); 
      local_polar_length = sqrt(pow(local_X_position, 2) + pow(local_Y_position, 2)); 
    }

    double global_polar_angle = local_polar_angle - previousHeading - (orientation_delta_rad/2);

    double X_position_delta = local_polar_length * cos(global_polar_angle); 
    double Y_position_delta = local_polar_length * sin(global_polar_angle);
    xpos += X_position_delta;
    ypos += Y_position_delta;
    previousHeading = newHeading;
    wait(10, msec);
  }
}

void TurnToPoint(double x, double y, int d, double time_limit_msec) {
  x *= (isRed ? 1 : -1);
  Stop(vex::brakeType::coast);
  isturning = true;
  double threshold = 1, add = 0;
  if(d == -1) {
    add = 180;
  }
  double turn_angle = NormalizeTarget(to_deg(atan2(x - xpos, y - ypos))) + add;
  PID pid = PID(tkp, tki, tkd);
  
  pid.SetTarget(turn_angle);
  pid.SetIntegralMax(0);  
  pid.SetIntegralRange(3);
  
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
  while (!pid.TargetArrived() && Brain.timer(msec) - start_time <= time_limit_msec) {
    pid.SetTarget(NormalizeTarget(to_deg(atan2(x - xpos, y - ypos))) + add);
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
  correct_angle = GetInertialHeading() * (isRed ? 1 : -1);
  cx = x;
  cy = y;
  isturning = false;
}

void MoveToPoint(double x, double y, int dir, double time_limit_msec, bool exit, double max_output, bool overturn) {
  x *= (isRed ? 1 : -1);
  Stop(vex::brakeType::coast);
  isturning = true;
  // Current robot setup, 3.25 inch wheel, 60/36 gear ratio.
  // Tuned parameters, DO NOT CHANGE!!!
  double threshold = 0.5;
  int add = dir > 0 ? 0 : 180;
  double maxslewfwd = dir > 0 ? maxslewaccelfwd : maxslewdecelrev;
  double maxslewrev = dir > 0 ? maxslewdecelfwd : maxslewaccelrev;
  bool minspeed = false;
  if(!exit) {
    if(!dirchangestart && dirchangeend) {
      maxslewfwd = dir > 0 ? 24 : maxslewdecelrev;
      maxslewrev = dir > 0 ? maxslewdecelfwd : 24;
    }
    if(dirchangestart && !dirchangeend) {
      maxslewfwd = dir > 0 ? maxslewaccelfwd : 24;
      maxslewrev = dir > 0 ? 24 : maxslewaccelrev;
      minspeed = true;
    }
    if(!dirchangestart && !dirchangeend) {
      maxslewfwd = 24;
      maxslewrev = 24;
      minspeed = true;
    }
  }

  //double maxslewfwd = 0.9;
  //double maxslewrev = 0.3;
  PID piddistance = PID(dkp, dki, dkd);
  PID pidh = PID(ckp, cki, ckd);

  piddistance.SetTarget(hypot(x - xpos, y - ypos));
  piddistance.SetIntegralMax(0);  
  piddistance.SetIntegralRange(3);
  piddistance.SetSmallBigErrorTolerance(threshold, threshold * 3);
  piddistance.SetSmallBigErrorDuration(50, 250);
  piddistance.SetDerivativeTolerance(5);
  // 5 Iterations
  
  pidh.SetTarget(NormalizeTarget(to_deg(atan2(x - xpos, y - ypos)) + add));
  pidh.SetIntegralMax(0);  
  pidh.SetIntegralRange(1);
  
  pidh.SetSmallBigErrorTolerance(0, 0);
  pidh.SetSmallBigErrorDuration(0, 0);
  pidh.SetDerivativeTolerance(0);
  pidh.SetArrive(false);
  // 5 Iterations

  // Reset the chassis.
  double start_time = Brain.timer(msec);
  double leftoutput = 0, rightoutput = 0, correction_output = 0, prevleftoutput = 0, prevrightoutput = 0;
  double exittolerance = 1;
  bool pline = false, prevpline = true;

  double current_angle = 0, ot = 0;
  bool ch = true;

  while (Brain.timer(msec) - start_time <= time_limit_msec) {
    pidh.SetTarget(NormalizeTarget(to_deg(atan2(x - xpos, y - ypos)) + add));
    piddistance.SetTarget(hypot(x - xpos, y - ypos));
    current_angle = GetInertialHeading();
    leftoutput = piddistance.Update(0) * cos(to_rad(atan2(x - xpos, y - ypos) * 180 / M_PI + add - current_angle)) * dir;
    rightoutput = leftoutput;
    pline = ((ypos - y) * -cos(to_rad(NormalizeTarget(current_angle + add))) <= (xpos - x) * sin(to_rad(NormalizeTarget(current_angle + add))) + exittolerance);
    if(pline && !prevpline) {
      break;
    }
    prevpline = pline;

    if(hypot(x - xpos, y - ypos) > 7 && ch == true) {
      correction_output = pidh.Update(current_angle);
    } else {
      correction_output = 0;
      ch = false;
    }

    //Minimum Output Check
    if(minspeed) {
      if(fabs(leftoutput) <= fabs(rightoutput) && leftoutput < min_output && leftoutput > 0) {
        rightoutput = rightoutput / leftoutput * min_output;
        leftoutput = min_output;
      } else if(fabs(rightoutput) < fabs(leftoutput) && rightoutput < min_output && rightoutput > 0) {
        leftoutput = leftoutput / rightoutput * min_output;
        rightoutput = min_output;
      } else if(fabs(leftoutput) <= fabs(rightoutput) && leftoutput > -min_output && leftoutput < 0) {
        rightoutput = rightoutput / leftoutput * -min_output;
        leftoutput = -min_output;
      } else if(fabs(rightoutput) < fabs(leftoutput) && rightoutput > -min_output && rightoutput < 0) {
        leftoutput = leftoutput / rightoutput * -min_output;
        rightoutput = -min_output;
      }
    }

    ot = fabs(leftoutput) + fabs(correction_output) - max_output;
    if(ot > 0 && overturn) {
      if(leftoutput > 0) {
        leftoutput -= ot;
      }
      else {
        leftoutput += ot;
      }
    }
    rightoutput = leftoutput;
    leftoutput = leftoutput + correction_output;
    rightoutput = rightoutput - correction_output;

    //Max Output Check
    if(fabs(leftoutput) >= fabs(rightoutput) && leftoutput > max_output) {
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
    if(prevleftoutput - leftoutput > maxslewrev) {
      leftoutput = prevleftoutput - maxslewrev;
    }
    if(prevrightoutput - rightoutput > maxslewrev) {
      rightoutput = prevrightoutput - maxslewrev;
    }
    if(leftoutput - prevleftoutput > maxslewfwd) {
      leftoutput = prevleftoutput + maxslewfwd;
    }
    if(rightoutput - prevrightoutput > maxslewfwd) {
      rightoutput = prevrightoutput + maxslewfwd;
    }
    prevleftoutput = leftoutput;
    prevrightoutput = rightoutput;
    ChassisControl(leftoutput, rightoutput);
    wait(10, msec);
  }
  if(exit == true) {
    prevleftoutput = 0;
    prevrightoutput = 0;
    Stop(vex::hold);
  }
  correct_angle = GetInertialHeading() * (isRed ? 1 : -1);
  isturning = false;
}

void MoveToPointEarly(double x, double y, int dir, double time_limit_msec, bool exit, double max_output, bool overturn) {
  x *= (isRed ? 1 : -1);
  Stop(vex::brakeType::coast);
  isturning = true;
  // Current robot setup, 3.25 inch wheel, 60/36 gear ratio.
  // Tuned parameters, DO NOT CHANGE!!!
  int add = dir > 0 ? 0 : 180;
  double maxslewfwd = dir > 0 ? maxslewaccelfwd : maxslewdecelrev;
  double maxslewrev = dir > 0 ? maxslewdecelfwd : maxslewaccelrev;
  bool minspeed = false;
  if(!exit) {
    if(!dirchangestart && dirchangeend) {
      maxslewfwd = dir > 0 ? 24 : maxslewdecelrev;
      maxslewrev = dir > 0 ? maxslewdecelfwd : 24;
    }
    if(dirchangestart && !dirchangeend) {
      maxslewfwd = dir > 0 ? maxslewaccelfwd : 24;
      maxslewrev = dir > 0 ? 24 : maxslewaccelrev;
      minspeed = true;
    }
    if(!dirchangestart && !dirchangeend) {
      maxslewfwd = 24;
      maxslewrev = 24;
      minspeed = true;
    }
  }

  //double maxslewfwd = 0.9;
  //double maxslewrev = 0.3;
  PID piddistance = PID(dkp - 0.6, 0, dkd-3);
  PID pidh = PID(ckp, cki, ckd);

  piddistance.SetTarget(hypot(x - xpos, y - ypos));
  piddistance.SetIntegralMax(0);  
  piddistance.SetIntegralRange(3);
  piddistance.SetSmallBigErrorTolerance(1, 3);
  piddistance.SetSmallBigErrorDuration(100, 250);
  piddistance.SetDerivativeTolerance(5);
  // 5 Iterations
  
  pidh.SetTarget(NormalizeTarget(to_deg(atan2(x - xpos, y - ypos)) + add));
  pidh.SetIntegralMax(0);  
  pidh.SetIntegralRange(1);
  
  pidh.SetSmallBigErrorTolerance(0, 0);
  pidh.SetSmallBigErrorDuration(0, 0);
  pidh.SetDerivativeTolerance(0);
  pidh.SetArrive(false);
  // 5 Iterations

  // Reset the chassis.
  double start_time = Brain.timer(msec);
  double leftoutput = 0, rightoutput = 0, correction_output = 0, prevleftoutput = 0, prevrightoutput = 0;
  double exittolerance = 1;
  bool pline = false, prevpline = true;

  double current_angle = 0, ot = 0;
  bool ch = true;

  while (Brain.timer(msec) - start_time <= time_limit_msec) {
    pidh.SetTarget(NormalizeTarget(to_deg(atan2(x - xpos, y - ypos)) + add));
    piddistance.SetTarget(hypot(x - xpos, y - ypos));
    current_angle = GetInertialHeading();
    leftoutput = piddistance.Update(0) * cos(to_rad(atan2(x - xpos, y - ypos) * 180 / M_PI + add - current_angle)) * dir;
    rightoutput = leftoutput;
    pline = ((ypos - y) * -cos(to_rad(NormalizeTarget(current_angle + add))) <= (xpos - x) * sin(to_rad(NormalizeTarget(current_angle + add))) + exittolerance);
    if(hypot(x - xpos, y - ypos) < 25) {
      break;
    }
    prevpline = pline;

    if(hypot(x - xpos, y - ypos) > 8 && ch == true) {
      correction_output = pidh.Update(current_angle);
    } else {
      correction_output = 0;
      ch = false;
    }

    //Minimum Output Check
    if(minspeed) {
      if(fabs(leftoutput) <= fabs(rightoutput) && leftoutput < min_output && leftoutput > 0) {
        rightoutput = rightoutput / leftoutput * min_output;
        leftoutput = min_output;
      } else if(fabs(rightoutput) < fabs(leftoutput) && rightoutput < min_output && rightoutput > 0) {
        leftoutput = leftoutput / rightoutput * min_output;
        rightoutput = min_output;
      } else if(fabs(leftoutput) <= fabs(rightoutput) && leftoutput > -min_output && leftoutput < 0) {
        rightoutput = rightoutput / leftoutput * -min_output;
        leftoutput = -min_output;
      } else if(fabs(rightoutput) < fabs(leftoutput) && rightoutput > -min_output && rightoutput < 0) {
        leftoutput = leftoutput / rightoutput * -min_output;
        rightoutput = -min_output;
      }
    }

    ot = fabs(leftoutput) + fabs(correction_output) - max_output;
    if(ot > 0 && overturn) {
      if(leftoutput > 0) {
        leftoutput -= ot;
      }
      else {
        leftoutput += ot;
      }
    }
    rightoutput = leftoutput;
    leftoutput = leftoutput + correction_output;
    rightoutput = rightoutput - correction_output;

    //Max Output Check
    if(fabs(leftoutput) >= fabs(rightoutput) && leftoutput > max_output) {
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
    if(prevleftoutput - leftoutput > maxslewrev) {
      leftoutput = prevleftoutput - maxslewrev;
    }
    if(prevrightoutput - rightoutput > maxslewrev) {
      rightoutput = prevrightoutput - maxslewrev;
    }
    if(leftoutput - prevleftoutput > maxslewfwd) {
      leftoutput = prevleftoutput + maxslewfwd;
    }
    if(rightoutput - prevrightoutput > maxslewfwd) {
      rightoutput = prevrightoutput + maxslewfwd;
    }
    prevleftoutput = leftoutput;
    prevrightoutput = rightoutput;
    ChassisControl(leftoutput, rightoutput);
    wait(10, msec);
  }
  if(exit == true) {
    prevleftoutput = 0;
    prevrightoutput = 0;
    Stop(vex::hold);
  }
  correct_angle = GetInertialHeading();
  isturning = false;
}

void boomerang(double x, double y, double a, double dlead, double time_limit_msec, int dir, bool exit, double max_output, bool overturn) {
  x *= (isRed ? 1 : -1);
  a *= (isRed ? 1 : -1);
  Stop(vex::brakeType::coast);
  isturning = true;
  // Current robot setup, 3.25 inch wheel, 60/36 gear ratio.
  // Tuned parameters, DO NOT CHANGE!!!
  double threshold = 0.5;
  int add = dir > 0 ? 0 : 180;
  double maxslewfwd = dir > 0 ? maxslewaccelfwd : maxslewdecelrev;
  double maxslewrev = dir > 0 ? maxslewdecelfwd : maxslewaccelrev;
  bool minspeed = false, cc = true;
  if(!exit) {
    if(!dirchangestart && dirchangeend) {
      maxslewfwd = dir > 0 ? 24 : maxslewdecelrev;
      maxslewrev = dir > 0 ? maxslewdecelfwd : 24;
    }
    if(dirchangestart && !dirchangeend) {
      maxslewfwd = dir > 0 ? maxslewaccelfwd : 24;
      maxslewrev = dir > 0 ? 24 : maxslewaccelrev;
      minspeed = true;
    }
    if(!dirchangestart && !dirchangeend) {
      maxslewfwd = 24;
      maxslewrev = 24;
      minspeed = true;
    }
  }

  PID piddistance = PID(dkp, dki, dkd);
  PID pidh = PID(ckp, cki, ckd);

  piddistance.SetTarget(0);
  piddistance.SetIntegralMax(3);  
  piddistance.SetSmallBigErrorTolerance(threshold, threshold * 3);
  piddistance.SetSmallBigErrorDuration(50, 250);
  piddistance.SetDerivativeTolerance(5);
  // 5 Iterations
  
  pidh.SetTarget(NormalizeTarget(to_deg(atan2(x - xpos, y - ypos))));
  pidh.SetIntegralMax(0);  
  pidh.SetIntegralRange(1);
  
  pidh.SetSmallBigErrorTolerance(0, 0);
  pidh.SetSmallBigErrorDuration(0, 0);
  pidh.SetDerivativeTolerance(0);
  pidh.SetArrive(false);
  // 5 Iterations

  // Reset the chassis.
  double start_time = Brain.timer(msec);
  double leftoutput = 0, rightoutput = 0, correction_output = 0, ts = 0, ot = 0;
  double exittolerance = 3;
  bool pline = false, prevpline = true;

  double current_angle = 0, h = 0, cax = 0, cay = 0;

  while ((!piddistance.TargetArrived()) && Brain.timer(msec) - start_time <= time_limit_msec) {
    h = hypot(xpos - x, ypos - y);
    cax = x - h * sin(to_rad(a + add)) * dlead;
    cay = y - h * cos(to_rad(a + add)) * dlead;
    piddistance.SetTarget(hypot(cax - xpos, cay - ypos) * dir);
    current_angle = GetInertialHeading();
    if(cc) {
      leftoutput = piddistance.Update(0) * cos(to_rad(atan2(cax - xpos, cay - ypos) * 180 / M_PI + add - current_angle));
    } else {
      leftoutput = piddistance.Update(0);
    }
    rightoutput = leftoutput;
    pline = ((ypos - y) * -cos(to_rad(NormalizeTarget(a))) <= (xpos - x) * sin(to_rad(NormalizeTarget(a))) + exittolerance);
    if(pline && !prevpline) {
      break;
    }
    prevpline = pline;

    //Minimum Output Check
    if(minspeed) {
      if(fabs(leftoutput) <= fabs(rightoutput) && leftoutput < min_output && leftoutput > 0) {
        rightoutput = rightoutput / leftoutput * min_output;
        leftoutput = min_output;
      } else if(fabs(rightoutput) < fabs(leftoutput) && rightoutput < min_output && rightoutput > 0) {
        leftoutput = leftoutput / rightoutput * min_output;
        rightoutput = min_output;
      } else if(fabs(leftoutput) <= fabs(rightoutput) && leftoutput > -min_output && leftoutput < 0) {
        rightoutput = rightoutput / leftoutput * -min_output;
        leftoutput = -min_output;
      } else if(fabs(rightoutput) < fabs(leftoutput) && rightoutput > -min_output && rightoutput < 0) {
        leftoutput = leftoutput / rightoutput * -min_output;
        rightoutput = -min_output;
      }
    }

    if(hypot(cax - xpos, cay - ypos) > 8) {
      pidh.SetTarget(NormalizeTarget(to_deg(atan2(cax - xpos, cay - ypos)) + add));
      correction_output = pidh.Update(current_angle);
    } else if(hypot(x - xpos, y - ypos) > 6) {
      pidh.SetTarget(NormalizeTarget(to_deg(atan2(x - xpos, y - ypos)) + add));
      correction_output = pidh.Update(current_angle);
    } else {
      cc = true;
      pidh.SetTarget(NormalizeTarget(a));
      correction_output = pidh.Update(current_angle);
      if(exit && hypot(x - xpos, y - ypos) < 5) {
        break;
      }
    }

    ts = sqrt(cp * getRadius(xpos, ypos, cax, cay, current_angle) * 9.8);
    if(leftoutput > ts) {
      leftoutput = ts;
    } else if(leftoutput < -ts) {
      leftoutput = -ts;
    }

    ot = fabs(leftoutput) + fabs(correction_output) - max_output;
    if(ot > 0 && overturn) {
      if(leftoutput > 0) {
        leftoutput -= ot;
      }
      else {
        leftoutput += ot;
      }
    }
    rightoutput = leftoutput;
    leftoutput = leftoutput + correction_output;
    rightoutput = rightoutput - correction_output;

    //Max Output Check
    if(fabs(leftoutput) >= fabs(rightoutput) && leftoutput > max_output) {
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
    if(prevleftoutput - leftoutput > maxslewrev) {
      leftoutput = prevleftoutput - maxslewrev;
    }
    if(prevrightoutput - rightoutput > maxslewrev) {
      rightoutput = prevrightoutput - maxslewrev;
    }
    if(leftoutput - prevleftoutput > maxslewfwd) {
      leftoutput = prevleftoutput + maxslewfwd;
    }
    if(rightoutput - prevrightoutput > maxslewfwd) {
      rightoutput = prevrightoutput + maxslewfwd;
    }
    prevleftoutput = leftoutput;
    prevrightoutput = rightoutput;
    ChassisControl(leftoutput, rightoutput);
    wait(10, msec);
  }
  if(exit) {
    prevleftoutput = 0;
    prevrightoutput = 0;
    Stop(vex::hold);
  }
  correct_angle = (a * (isRed ? 1 : -1));
  isturning = false;
}


void BarCross() {
  int i = 0;
  while(InertialA.pitch() < 15 && i < 100) {
    ChassisControl(8, 8);
    wait(10, msec);
    i++;
  }
  i = 0;
  while(InertialA.pitch() > 0 && i < 100) {
    wait(10, msec);
    i++;
  }
  i = 0;
  while(InertialA.pitch() < 0 && i < 100) {
    ChassisControl(6, 6);
    wait(10, msec);
    i++;
  }
  i = 0;
  while(i < 50) {
    ChassisControl(-4, -4);
    wait(10, msec);
    i++;
  }
  ChassisControl(0, 0);
  xpos = 0;
  ypos = 0;
}

void friction_test() {
  long long sum = 0;
  //left motor 1
  Brain.Screen.clearScreen();
  left_chassis1.spin(fwd, 12, volt);
  wait(5000, msec);
  for(int i = 0; i < 1000; i++) {
    sum += left_chassis1.velocity(rpm);
    wait(10, msec);
  }
  Brain.Screen.print(sum / 1000.0);
  Brain.Screen.newLine();
  left_chassis1.stop(hold);
  wait(1000, msec);
  left_chassis1.stop(coast);
  //left motor 2
  sum = 0;
  left_chassis2.spin(fwd, 12, volt);
  wait(5000, msec);
  for(int i = 0; i < 1000; i++) {
    sum += left_chassis2.velocity(rpm);
    wait(10, msec);
  }
  Brain.Screen.print(sum / 1000.0);
  Brain.Screen.newLine();
  left_chassis2.stop(hold);
  wait(1000, msec);
  left_chassis2.stop(coast);
  //left motor 3
  sum = 0;
  left_chassis3.spin(fwd, 12, volt);
  wait(5000, msec);
  for(int i = 0; i < 1000; i++) {
    sum += left_chassis3.velocity(rpm);
    wait(10, msec);
  }
  Brain.Screen.print(sum / 1000.0);
  Brain.Screen.newLine();
  left_chassis3.stop(hold);
  wait(1000, msec);
  left_chassis3.stop(coast);
  //right motor 1
  sum = 0;
  right_chassis1.spin(fwd, 12, volt);
  wait(5000, msec);
  for(int i = 0; i < 1000; i++) {
    sum += right_chassis1.velocity(rpm);
    wait(10, msec);
  }
  Brain.Screen.print(sum / 1000.0);
  Brain.Screen.newLine();
  right_chassis1.stop(hold);
  wait(1000, msec);
  right_chassis1.stop(coast);
  //right motor 2
  sum = 0;
  right_chassis2.spin(fwd, 12, volt);
  wait(5000, msec);
  for(int i = 0; i < 1000; i++) {
    sum += right_chassis2.velocity(rpm);
    wait(10, msec);
  }
  Brain.Screen.print(sum / 1000.0);
  Brain.Screen.newLine();
  right_chassis2.stop(hold);
  wait(1000, msec);
  right_chassis2.stop(coast);
  //right motor 3
  sum = 0;
  right_chassis3.spin(fwd, 12, volt);
  wait(5000, msec);
  for(int i = 0; i < 1000; i++) {
    sum += right_chassis3.velocity(rpm);
    wait(10, msec);
  }
  Brain.Screen.print(sum / 1000.0);
  Brain.Screen.newLine();
  right_chassis3.stop(hold);
  wait(1000, msec);
  right_chassis3.stop(coast);
}

void arm_thread() {
  if(arm_motor.position(deg) < arm_angle_target) {
    while(arm_motor.position(deg) < arm_angle_target){
      arm_motor.spin(fwd, 12, volt);
    }
  } else {
    while(arm_motor.position(deg) > arm_angle_target){
      arm_motor.spin(fwd, -12, volt);
    }
  }
  arm_motor.stop(hold);
}

void arm_pid(double arm_target) {
  PID pidarm = PID(0.1, 0, 0.5);
  pidarm.SetTarget(arm_target);
  pidarm.SetIntegralMax(0);  
  pidarm.SetIntegralRange(1);
  pidarm.SetSmallBigErrorTolerance(1, 1);
  pidarm.SetSmallBigErrorDuration(0, 0);
  pidarm.SetDerivativeTolerance(100);
  pidarm.SetArrive(true);
  arm_motor.spin(fwd, pidarm.Update(arm_motor.position(deg)), volt);
}

void arm_pid_loop() {
  while(true) {
    if(arm_pid_target == arm_load_target) {
      targetIntakeVolts = 0;
    }
    arm_pid(arm_pid_target);
    wait(10, msec);
  }
}

void wait_intake() {
  int i = 0;
  while(i < 50 && distance_sensor.objectDistance(mm) > 50) {
    wait(10, msec);
    i++;
  }
  wait(300, msec);
}

void wait_intake_thread() {
  int i = 0;
  while(i < 50 && distance_sensor.objectDistance(mm) > 50) {
    wait(10, msec);
    i++;
  }
}

void wait_load() {
  int i = 0;
  intake(12);
  while(i++ < 100 /* && clip_sensor.objectDistance(mm) > 55*/) {
    wait(10, msec);
  }
  wait(100, msec);
  // clipper.set(true);
}

void arm_load() {
  intake(12);
  int i = 30;
  targetIntakeVolts = 0;
  while(i-- > 0 || distance_sensor.objectDistance(mm) > 50) {
    arm_pid(arm_load_target);
    wait(10, msec);
  }
  arm_motor.stop(hold);
  intake(12);
  targetIntakeVolts = 0;
  wait(600, msec);
  // clipper.set(true);
  intake(-6);
  wait(50, msec);
  intake_stop();
  wait(100, msec);
  while(true) {
    arm_pid(arm_store_target);
    wait(10, msec);
  }
}
void armToAngle(double target, double time_limit_msec) {
  PID pidarm = PID(0.5, 0, 0);
  pidarm.SetTarget(target);
  pidarm.SetIntegralMax(0);  
  pidarm.SetIntegralRange(1);
  pidarm.SetSmallBigErrorTolerance(1, 1);
  pidarm.SetSmallBigErrorDuration(0, 0);
  pidarm.SetDerivativeTolerance(5);
  pidarm.SetArrive(true);
  double start_time = Brain.timer(msec);
  while (Brain.timer(msec) - start_time <= time_limit_msec) {
    if (fabs(arm_motor.position(deg)-target) <= 3) {
      break;
    }
    arm_motor.spin(fwd, pidarm.Update(arm_motor.position(deg)), volt);  
  }
  arm_motor.stop(brake);
}

void MoveToObject(aivision &vision, const aivision::colordesc &color, double centerX, double width, int dir, double time_limit_msec, bool exit, double max_output, bool overturn) {
  Stop(vex::brakeType::coast);
  isturning = true;
  // Current robot setup, 3.25 inch wheel, 60/36 gear ratio.
  // Tuned parameters, DO NOT CHANGE!!!
  double threshold = 0.5;
  int add = dir > 0 ? 0 : 180;
  double maxslewfwd = dir > 0 ? maxslewaccelfwd : maxslewdecelrev;
  double maxslewrev = dir > 0 ? maxslewdecelfwd : maxslewaccelrev;

  PID piddistance = PID(0.15, 0, 0.4);
  PID pidh = PID(0.05, 0, 0.15);

  vision.takeSnapshot(color);
  if (!vision.largestObject.exists) return;

  double dY = width - vision.largestObject.width;
  double dX = centerX - vision.largestObject.centerX;

  piddistance.SetTarget(dY);
  piddistance.SetIntegralMax(0);  
  piddistance.SetIntegralRange(3);
  piddistance.SetSmallBigErrorTolerance(threshold, threshold * 3);
  piddistance.SetSmallBigErrorDuration(50, 250);
  piddistance.SetDerivativeTolerance(5);
  // 5 Iterations
  
  pidh.SetTarget(dX);
  pidh.SetIntegralMax(0);  
  pidh.SetIntegralRange(3);
  pidh.SetSmallBigErrorTolerance(threshold, threshold * 3);
  pidh.SetSmallBigErrorDuration(50, 250);
  pidh.SetDerivativeTolerance(5);
  // 5 Iterations

  // Reset the chassis.
  double start_time = Brain.timer(msec);
  double leftoutput = 0, rightoutput = 0, correction_output = 0, prevleftoutput = 0, prevrightoutput = 0;
  double exittolerance = 1;
  bool pline = false, prevpline = true;

  double current_angle = 0, ot = 0;
  bool ch = true;
  int hitCnt = 0;

  while (Brain.timer(msec) - start_time <= time_limit_msec) {
    vision.takeSnapshot(color);
    if (!vision.largestObject.exists) {
      break;
    }
 
    dY = width - vision.largestObject.width;
    piddistance.SetTarget(dY);
    leftoutput = piddistance.Update(0);
    rightoutput = leftoutput;

    dX = centerX - vision.largestObject.centerX;
    if (fabs(dY) <= 5.0 && fabs(dX) <= 5.0) {
      hitCnt++;
      if (hitCnt > 3) {
        break;
      }
    } else {
      if (hitCnt>0) hitCnt--;
    }
    pidh.SetTarget(dX);
    correction_output = -pidh.Update(0);
    leftoutput = leftoutput + correction_output;
    rightoutput = rightoutput - correction_output;

    //Max Output Check
    if(fabs(leftoutput) >= fabs(rightoutput) && leftoutput > max_output) {
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
    if(prevleftoutput - leftoutput > maxslewrev) {
      leftoutput = prevleftoutput - maxslewrev;
    }
    if(prevrightoutput - rightoutput > maxslewrev) {
      rightoutput = prevrightoutput - maxslewrev;
    }
    if(leftoutput - prevleftoutput > maxslewfwd) {
      leftoutput = prevleftoutput + maxslewfwd;
    }
    if(rightoutput - prevrightoutput > maxslewfwd) {
      rightoutput = prevrightoutput + maxslewfwd;
    }
    prevleftoutput = leftoutput;
    prevrightoutput = rightoutput;
    ChassisControl(leftoutput, rightoutput);
    wait(10, msec);
  }
  if(exit == true) {
    prevleftoutput = 0;
    prevrightoutput = 0;
    Stop(vex::hold);
  }
  correct_angle = GetInertialHeading();
  isturning = false;
}
