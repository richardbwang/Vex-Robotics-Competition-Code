/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Richard Wang                                              */
/*    Created:      July 11 2022                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller
// leftintake_motor     motor         6
// left_chassis1        motor         4
// right_chassis1       motor         3
// LimitSwitchA         limit         A
// lift_down            motor         8
// righttintake_motor   motor         7
// left_chassis2        motor         2
// right_chassis2       motor         1
// xi_motor             motor         5
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "vex.h"
#include "auto.h"
#include "math.h"
#include "motor-control.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  // Inertial.calibrate();
  // waitUntil(!Inertial.isCalibrating());
  // Inertial.resetRotation();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  //Starting to move and turning to roller
  ChassisControl(75, 75);
  wait(250, msec);
  ChassisControl(0, 0);
  wait(600, msec);
  ChassisControl(-50, 75);
  wait(330, msec);
  ChassisControl(25, 25);
  wait(200, msec);

  //Spinning the roller and starting the flywheel
  xi(-100);
  intake(75, 75);
  wait(600, msec);
  ChassisControl(0, 0);
  ChassisControl(-50, -70);
  wait(45, msec);
  ChassisControl(0, 0);
  xi(0);
  wait(1000, msec);

  //Shooting first two discs into high goal
  wait(200, msec);
  for (int i = 0; i < 3; ++i) {
    liftDown(100);
    wait(100, msec);
    liftDown(-50);
    wait(150, msec);
    liftDown(0);
    wait(400, msec);
  }
  intake(0, 0);

  //Picking up three dics
  ChassisControl(-50, 50);
  wait(300, msec);
  ChassisControl(100, 100);
  wait(25, msec);
  ChassisControl(-50, 50);
  wait(390, msec);
  ChassisControl(50, 50);
  xi(-100);
  wait(2250, msec);
  ChassisControl(-100, -100);
  wait(500, msec);
  ChassisControl(0, 0);
  wait(250, msec);
  ChassisControl(75, -75);
  wait(255, msec);
  ChassisControl(0, 0);

  //Shooting discs into high goal
  intake(65, 65);
  wait(2000, msec);
  for (int i = 0; i < 4; ++i) {
    liftDown(100);
    wait(100, msec);
    liftDown(-50);
    wait(150, msec);
    liftDown(0);
    wait(400, msec);
  }
  intake(0, 0);
  xi(0);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

int Ch1, Ch2, Ch3, Ch4;                                              //声明
bool L1, L2, R1, R2, BtnA, BtnB, BtnX, BtnY, BtnU, BtnD, BtnL, BtnR; //声明
int intake_flag = 0, intake_flag1 = 0;
int lift_flag = 0, lift_flag1 = 0;
int intake_time = 0;
int fly_flag = 0, fly_flag1 = 0;
int long_fly_flag = 0, long_fly_flag1 = 0;
int rum_flag = 0;
int i = 471;
int last_s_l = 0, last_s_r = 0;
void usercontrol(void) {

  // User control code here, inside the loop
  while (1) {
    //{
    // int Brain_time_111=Brain.timer(msec);
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    // User control code here, inside the loop

    Ch1 = Controller1.Axis1.value();
    Ch2 = Controller1.Axis2.value();
    Ch3 = Controller1.Axis3.value();
    Ch4 = Controller1.Axis4.value();
    L1 = Controller1.ButtonL1.pressing();
    L2 = Controller1.ButtonL2.pressing();
    R1 = Controller1.ButtonR1.pressing();
    R2 = Controller1.ButtonR2.pressing();
    BtnA = Controller1.ButtonA.pressing();
    BtnB = Controller1.ButtonB.pressing();
    BtnX = Controller1.ButtonX.pressing();
    BtnY = Controller1.ButtonY.pressing();
    BtnU = Controller1.ButtonUp.pressing();
    BtnD = Controller1.ButtonDown.pressing();
    BtnL = Controller1.ButtonLeft.pressing();
    BtnR = Controller1.ButtonRight.pressing();
    //=========================================================================
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print(leftintake_motor.velocity(rpm));
    Brain.Screen.newLine();
    Brain.Screen.print(righttintake_motor.velocity(rpm));
    Brain.Screen.newLine();

    //=========================================================================
    if (abs(Ch4) < 5 &&
        abs(Ch3) > 10)
    {
      ChassisControl(Ch3, Ch3);
    } else if (abs(Ch4) > 5 && (intake_flag == 1 || BtnA)) {
      ChassisControl((Ch3 - Ch4) * 0.5, (Ch3 + Ch4) * 0.35);
    } else if (abs(Ch4) > 5)
    {
      ChassisControl((Ch3 - Ch4) * 0.65, (Ch3 + Ch4) * 0.65);
    } else {
      Stop(coast);
    }

    if (BtnX && fly_flag1 == 0 && fly_flag == 0) {
      fly_flag = 1;
      fly_flag1 = 1;
    } else if (BtnX && fly_flag1 == 0 && fly_flag == 1) {
      fly_flag = 0;
      fly_flag1 = 1;
    } else if (!BtnX) {
      fly_flag1 = 0;
    }

    if (BtnY && long_fly_flag1 == 0 && long_fly_flag == 0) {
      long_fly_flag = 1;
      long_fly_flag1 = 1;
    } else if (BtnY && long_fly_flag1 == 0 && long_fly_flag == 1) {
      long_fly_flag = 0;
      long_fly_flag1 = 1;
    } else if (!BtnY) {
      long_fly_flag1 = 0;
    }
    if (BtnA) {
      fly_flag = 0;
      intake_pid(70, 70); /////////////////////////////
      if (leftintake_motor.velocity(rpm) >= 0 * 600 &&
          righttintake_motor.velocity(rpm) >= 0 * 600 && intake_flag1 == 0) {
        intake_flag = 1;
        intake_flag1 = 1;
      }
      if (intake_flag == 1 && lift_flag1 == 0) {
        intake_time = Brain.timer(msec);
        lift_flag1 = 1;
      }
      if (intake_flag == 1 && Brain.timer(msec) - intake_time < 1) {
        liftDown(100);
      } else if (LimitSwitchA.pressing() == 0) {
        liftDown(-50);
        lift_flag = 1;
      }
      if (LimitSwitchA.pressing() == 1 && lift_flag == 1) {
        intake_flag = 0;
        intake_flag1 = 0;
        lift_flag = 0;
        lift_flag1 = 0;
        intake_time = 0;
        lift_down.resetRotation();
        liftDown(0);
      }
    }
    //========================================================
    if (fly_flag == 1 && !BtnA) {
      intake_pid(70, 70); ////////////////////
      long_fly_flag = 0;
    } else if (long_fly_flag == 1)
    {
      intake_pid(120, 120);
      fly_flag = 0;
    } else if (BtnD) {
      intake(-50, -50);
    } else if (!BtnA && fly_flag == 0 && long_fly_flag == 0) {
      intake(0, 0);
    }

    //===============================

    //
    //========================================================
    if (BtnB && !BtnA) {
      if (lift_down.rotation(deg) < 20) {
        liftDown(100);
      } else {
        lift_down.stop(brake);
      }
    }

    if (!BtnB && LimitSwitchA.pressing() == 0 && !BtnA) {
      liftDown(-20);
    } else if (!BtnB && !BtnA) {
      intake_flag = 0;
      intake_flag1 = 0;
      lift_flag = 0;
      lift_flag1 = 0;
      intake_time = 0;
      lift_down.resetRotation();
      liftDown(0);
    }

    //========================================================
    if (R1) {
      xi(100);
    } else if (R2) {
      xi(-100);
    } else {
      xi(0);
    }

    if ((fly_flag == 1 && rum_flag == 0 &&
         leftintake_motor.velocity(rpm) >= 0.4 * 600 &&
         righttintake_motor.velocity(rpm) >= 0.75 * 600) ||
        (long_fly_flag == 1 && rum_flag == 0 &&
         leftintake_motor.velocity(rpm) >= 0.65 * 600 &&
         righttintake_motor.velocity(rpm) >= 1 * 600)) {
      Controller1.rumble(rumblePulse);
      rum_flag = 1;
    } else if (fly_flag == 0) {
      rum_flag = 0;
    }
    //=======================================================================
    if (fly_flag == 1 || BtnA || long_fly_flag == 1) {

      if (i >= 470) {
        Brain.Screen.clearScreen(black);
        Brain.Screen.setPenColor(white);
        Brain.Screen.drawLine(20, 1, 20, 230);
        Brain.Screen.drawLine(20, 230, 272, 230);
        Brain.Screen.setPenColor(blue);
        Brain.Screen.drawLine(211, 20, 220, 20);
        Brain.Screen.printAt(221, 20, ":左");
        Brain.Screen.setPenColor(green);
        Brain.Screen.drawLine(231, 20, 240, 20);
        Brain.Screen.printAt(241, 20, ":右");
        i = 0;
      } else {
        Brain.Screen.setPenColor(red);
        Brain.Screen.drawLine(i + 20, 220 - leftintake_motor.velocity(rpm) / 3,
                              i + 20, 220 - last_s_l / 3);
        Brain.Screen.setPenColor(green);
        Brain.Screen.drawLine(i + 20, 30 + righttintake_motor.velocity(rpm) / 3,
                              i + 20, 30 + last_s_r / 3);
        i++;
        last_s_l = leftintake_motor.velocity(rpm);
        last_s_r = righttintake_motor.velocity(rpm);
      }
    } else {
      last_s_l = 0;
      last_s_r = 0;
    }

    if (BtnU) {
      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(1, 1);
      Controller1.Screen.print(leftintake_motor.temperature());
      Controller1.Screen.newLine();
      Controller1.Screen.print(righttintake_motor.temperature());
    }

    wait(10, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
                    // Brain.Screen.newLine();
                    // Brain.Screen.print(Brain.timer(msec) - Brain_time_111);
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();
  /*
  Inertial15.calibrate();
  // waits for the Inertial Sensor to calibrate
  while (Inertial15.isCalibrating()) {
    wait(100, msec);
  }*/
  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
