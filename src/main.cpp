/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Richard Wang                                              */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// LimitSwitchA         limit         A               
// catapult_motor       motor         11              
// intake_motor         motor         8               
// left_chassis1        motor         19              
// left_chassis2        motor         12              
// right_chassis1       motor         17              
// right_chassis2       motor         16              
// left_chassis3        motor         13              
// right_chassis3       motor         20              
// InertialA            inertial      18              
// DigitalOutB          digital_out   B               
// DigitalOutC          digital_out   C               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "motor-control.h"
#include "math.h"



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
  float point;
  point = (InertialA.heading(degrees));
  //Brain.Screen.print("Point : %f\n", point);
  // Initializing Robot Configuration
  vexcodeInit();
  //calibrate inertial sensor
  InertialA.calibrate();
  // waits for the Inertial Sensor to calibrate
  while (InertialA.isCalibrating()) {
    wait(100, msec);
  }
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*                                                                           */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  for (int i = 0; i < 4; i++) {
    TurnToAngle(90, 1200);
    wait(1000, msec);
  }
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

int Ch1, Ch2, Ch3, Ch4;
bool L1, L2, R1, R2, BtnA, BtnB, BtnX, BtnY, BtnU, BtnD, BtnL, BtnR;//声明
int dipan_flag = 0;
int xi_flag = 0, xi_flag1 = 0;

void usercontrol(void) {
  Brain.Screen.clearScreen();
  
  // User control code here, inside the loop
  while (1) {
   //{
      //int Brain_time_111=Brain.timer(msec);
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
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print(LimitSwitchA.pressing());
//=========================================================================
    if(abs(Ch4) < 12 && abs(Ch3) > 12 ) {
      ChassisControl(Ch3, Ch3);
      dipan_flag = 0;
    } else if(abs(Ch4) >= 12 ) {
      ChassisControl((Ch3 - Ch4) * 0.55, (Ch3 + Ch4) * 0.55);
      dipan_flag = 1;
    } else {
      Stop(dipan_flag == 0 ? coast : brake);
    }
//=========================================================================
    
    if(BtnA) {
      catapult_motor.spin(fwd,100*0.128,volt);
    } else if(LimitSwitchA.pressing() == 0) {
      catapult_motor.spin(fwd,100*0.128,volt);
    } else {
      catapult_motor.stop(hold);
    }

//=========================================================================
    if (R1 && LimitSwitchA.pressing() == 1) {
      intake_motor.spin(fwd, 0.128 *100, voltageUnits::volt);
    } else if(R2) {
      intake_motor.spin(fwd, 0.128 *-100, voltageUnits::volt);
    } else { 
      intake_motor.stop(hold);
    }
   
    if (L1) {
      if (L2) {
         DigitalOutB.set(true);
      }
    } else if (L2) {
      DigitalOutB.set(false);
    }
 
    if (BtnY) {
      DigitalOutC.set(true);
    } 
    // Sleep the task for a short amount of time to prevetning wasting 
    // too much resources.
    wait(10, msec); 
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

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
 