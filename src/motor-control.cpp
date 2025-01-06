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
bool spinfw = false;
bool dirchangestart = true;
bool dirchangeend = true;
double min_output = 6;
double dkp = 1, dki = 0.3, dkd = 7;
double tkp = 0.45, tki = 0, tkd = 4;
double ckp = 0.3, cki = 0.05, ckd = 4;
double maxslewaccelfwd = 24;
double maxslewdecelfwd = 0.2;
double maxslewaccelrev = 0.5;
double maxslewdecelrev = 24;
double prevleftoutput = 0, prevrightoutput = 0;
double xpos = 0, ypos = 0;
double correct_angle = 0, cx = 0, cy = 0;
double distance_value = 35;
double distancebetweenwheels = 11.28;
double cp = 2;
double hangangletarget = 0;
double hangangletimelimit = 0;

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
  if (angle - InertialA.rotation() > 180) {
    while (angle - InertialA.rotation() > 180) {
      angle = angle - 360;
    }
  } else if (angle - InertialA.rotation() < -180) {
    while (angle - InertialA.rotation() < -180) {
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

void TurnToAngle(double turn_angle, double time_limit_msec, double max_output) {
  Stop(vex::brakeType::coast);
  isturning = true;
  double threshold = 1;
  PID pid = PID(tkp, tki, tkd);
  
  pid.SetTarget(NormalizeTarget(turn_angle));
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
  double current_heading;
  double previous_heading = 0;
  int index = 1;
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
  Stop(vex::hold);
  correct_angle = turn_angle;
  isturning = false;
}

void DriveTo(double distance_in, double time_limit_msec, bool exit, double max_output) {
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

  while (((!piddistance.TargetArrived()) && Brain.timer(msec) - start_time <= time_limit_msec) || (exit == false && current_distance < distance_in)) {
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
      } else if(fabs(leftoutput) <= fabs(rightoutput) && leftoutput > -min_output && leftoutput < 0) {
        rightoutput = rightoutput / leftoutput * -min_output;
        leftoutput = -min_output;
      } else if(fabs(rightoutput) < fabs(leftoutput) && rightoutput > -min_output && rightoutput < 0) {
        leftoutput = leftoutput / rightoutput * -min_output;
        rightoutput = -min_output;
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
  if(exit == true) {
    prevleftoutput = 0;
    prevrightoutput = 0;
    Stop(vex::hold);
  }
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

void CurveCircle(double result_angle_deg, double center_radius, double time_limit_msec, bool exit, double max_output) {
  double startr = GetRightRotationDegree(), startl = GetLeftRotationDegree();
  double in_arc, out_arc;
  double real_angle = 0, current_angle = 0;
  double ratio, result_angle;
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

      right_output = right_output + correction_output;
      left_output = left_output - correction_output;

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

      ChassisControl(left_output * drive_direction, right_output * drive_direction);
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

      right_output = right_output + correction_output;
      left_output = left_output - correction_output;

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

      ChassisControl(left_output * drive_direction, right_output * drive_direction);
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
      
      right_output = right_output + correction_output;
      left_output = left_output - correction_output;

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

      ChassisControl(left_output * drive_direction, right_output * drive_direction);
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

      right_output = right_output + correction_output;
      left_output = left_output - correction_output;

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

      ChassisControl(left_output * drive_direction, right_output * drive_direction);
      wait(10, msec);
    }
  }
  if(exit == true) {
    Stop(vex::brakeType::hold);
  }
  correct_angle = result_angle_deg;
  isturning = false;
  dkp = 0.8, dki = 0.1, dkd = 7.5;
}

void Swing(double swing_angle, double drive_direction, double time_limit_msec, bool exit, double max_output) {
  Stop(vex::brakeType::coast);
  isturning = true;
  double threshold = 1;
  PID pid = PID(tkp, tki, tkd);
  
  pid.SetTarget(NormalizeTarget(swing_angle));
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

      if(output > max_output) {
        output = max_output;
      } else if(output < -max_output) {
        output = -max_output;
      }

      left_chassis1.stop(hold);
      left_chassis2.stop(hold);
      left_chassis3.stop(hold);
      right_chassis1.spin(fwd, -output * drive_direction, volt);
      right_chassis2.spin(fwd, -output * drive_direction, volt);
      right_chassis3.spin(fwd, -output * drive_direction, volt);
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

      if(output > max_output) {
        output = max_output;
      } else if(output < -max_output) {
        output = -max_output;
      }

      left_chassis1.spin(fwd, -output * drive_direction, volt);
      left_chassis2.spin(fwd, -output * drive_direction, volt);
      left_chassis3.spin(fwd, -output * drive_direction, volt);
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
  correct_angle = swing_angle;
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
  double r = 0, dr = 0, a = 0, da = 0, dly = 0;
  while(true) {
    da = to_rad(GetInertialHeading()) - a;
    dr = (GetRightRotationDegree() - r) * wheel_distance_in / 360;
    if(da == 0) {
      dly = dr / 2;
    } else {
      dly = 2 * sin(da / 2) * (dr / da + distancebetweenwheels / 2);
    }
    xpos += dly * sin(a + da / 2);
    ypos += dly * cos(a + da / 2);
    a = to_rad(GetInertialHeading());
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
  correct_angle = turn_angle;
  cx = x;
  cy = y;
  isturning = false;
}

void MoveToPoint(double x, double y, int dir, double time_limit_msec, bool exit, double max_output, bool overturn) {
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
  double exittolerance = 4;
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

    if(hypot(x - xpos, y - ypos) > 3 && ch == true) {
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
  isturning = false;
}

void boomerang(double x, double y, double a, double dlead, double time_limit_msec, int dir, bool exit, double max_output, bool overturn) {
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
  double exittolerance = 1;
  bool pline = false, prevpline = true;

  double current_angle = 0, h = 0, cax = 0, cay = 0;
  bool ch = true;

  while ((!piddistance.TargetArrived()) && Brain.timer(msec) - start_time <= time_limit_msec) {
    h = hypot(xpos - x, ypos - y);
    cax = x - h * sin(to_rad(a + add)) * dlead;
    cay = y - h * cos(to_rad(a + add)) * dlead;
    piddistance.SetTarget(hypot(cax - xpos, cay - ypos) * dir);
    current_angle = GetInertialHeading();
    leftoutput = piddistance.Update(0) * cos(to_rad(atan2(cax - xpos, cay - ypos) * 180 / M_PI + add - current_angle));
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

    if(hypot(cax - xpos, cay - ypos) > 3 && ch == true) {
      pidh.SetTarget(NormalizeTarget(to_deg(atan2(cax - xpos, cay - ypos)) + add));
      correction_output = pidh.Update(current_angle);
    } else {
      pidh.SetTarget(NormalizeTarget(a));
      correction_output = pidh.Update(current_angle);
      ch = false;
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
  if(exit == true) {
    prevleftoutput = 0;
    prevrightoutput = 0;
    Stop(vex::hold);
  }
  correct_angle = a;
  isturning = false;
}

void hangangle() {
  catapult_motor.stop(coast);
  double threshold = 1;
  double kp = 3;
  double ki = 0;
  double kd = 0;
  PID pid = PID(kp, ki, kd);
  
  pid.SetTarget(hangangletarget);
  pid.SetIntegralMax(0);  
  pid.SetIntegralRange(5);
  
  pid.SetSmallBigErrorTolerance(threshold, threshold * 3);
  pid.SetSmallBigErrorDuration(50, 250);
  pid.SetDerivativeTolerance(threshold * 4.5);
  // 5 Iterations
  
  // Start the PID loop.
  double start_time = Brain.timer(msec);
  double output;
  while (!pid.TargetArrived() && Brain.timer(msec) - start_time <= hangangletimelimit) {
    output = pid.Update(catapult_motor.position(degrees) * 12 / 84);
    catapult_motor.spin(fwd, output, voltageUnits::volt);
    wait(10, msec);
  }  
  catapult_motor.stop(hold);
}

void Arm(double arm_power) {
  awp_motor.spin(fwd, arm_power, voltageUnits::volt);
}

void fwpid(double rpm) {
  double kv = 0.01;
  double ff = kv * rpm;
  double bbrange = 50;
  double kp = 0.6;
  double ki = 0.03;
  double kd = 0;
  PID pid = PID(kp, ki, kd);
  
  pid.SetTarget(rpm);
  pid.SetIntegralMax(0);  
  pid.SetIntegralRange(5);
  
  pid.SetSmallBigErrorTolerance(0, 0);
  pid.SetSmallBigErrorDuration(0, 0);
  pid.SetDerivativeTolerance(0);
  pid.SetArrive(false);
  // 5 Iterations
  
  // Draw the baseline.
  double draw_amplifier = 230 / fabs(rpm);
  Brain.Screen.clearScreen(black);
  Brain.Screen.setPenColor(green);
  Brain.Screen.drawLine(0, fabs(rpm) * draw_amplifier, 
                        600, fabs(rpm) * draw_amplifier);
  Brain.Screen.setPenColor(red);
  
  // Start the PID loop.
  double output;
  double current_rpm;
  double previous_rpm = 0;
  int index = 1;
  while (spinfw) {
    current_rpm = intake_motor.velocity(velocityUnits::rpm);
    output = pid.Update(current_rpm);
    
    // Draw line
    Brain.Screen.drawLine(
        index * 3, fabs(previous_rpm) * draw_amplifier, 
        (index + 1) * 3, fabs(current_rpm * draw_amplifier));
    index++;
    previous_rpm = current_rpm;
    if(rpm - current_rpm > bbrange) {
      output = 12;
    } else if(current_rpm - rpm > bbrange) {
      output = -12;
    }
    // End
    intake_motor.spin(fwd, output + ff, voltageUnits::volt);
    wait(10, msec);
  }
}