/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Richard Wang (99116X)                                        */
/*    Created:      Feburary 15, 2023                                         */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// LimitSwitchA         limit         A               
// catapult_motor       motor         16              
// intake_motor         motor         2               
// left_chassis1        motor         14              
// left_chassis2        motor         13              
// right_chassis1       motor         18              
// right_chassis2       motor         17              
// left_chassis3        motor         12              
// right_chassis3       motor         19              
// InertialA            inertial      4               
// DigitalOutC          digital_out   C               
// DigitalOutD          digital_out   D               
// DigitalOutE          digital_out   E               
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
  point = (InertialA.rotation(degrees));
  
  // Initializing Robot Configuration
  vexcodeInit();
  
  //calibrate inertial sensor
  InertialA.calibrate();

  // waits for the Inertial Sensor to calibrate
  while (InertialA.isCalibrating()) {
    wait(100, msec);
  }

  double current_heading = InertialA.heading();
  Brain.Screen.print(current_heading);
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
  double begin_time = Brain.timer(msec);
  thread PullCatapult1 = thread(PullCatapult);
  thread anglestabilization = thread(AngleStabilization);
  
  // Spin the roller.
  Grab(-100);
  ChassisControl(17, 17);
  wait(200, msec);
  Grab(0);
  ChassisControl(0, 0);
  
  // Knock down the 3 disk stack.
  DriveTo(-10.5, 0, 1100);
  TurnSmall(56, 900);
  DriveTo(-30, 0, 1600);

  // Back up a bit, turn and shoot the first disk.
  DriveTo(4, 0, 1100);
  TurnMedium(-75, 1000);
  FireCatapult();
  
  // Lower the catapult again, load the second disk, shoot.
  PullCatapult();
  Grab(70);
  wait(800, msec);
  FireCatapult();
  
  // Lower the catapult, pick the 3rd disk.
  thread PullCatapult2 = thread(PullCatapult);
  TurnMedium(-102, 1200);
  while(LimitSwitchA.pressing() == 0) {
    wait(10, msec);
  }
  Grab(70);
  DriveTo(8, 0, 1200);
  TurnMedium(95, 1100);
  FireCatapult();

  // Lower the catapult, pick up the 4th disk
  thread PullCatapult3 = thread(PullCatapult);
  thread grabcheck1 = thread(GrabCheck);
  TurnMedium(-95, 1100);
  while(LimitSwitchA.pressing() == 0) {
    wait(10, msec);
  }
  DriveTo(8, 0, 1100);
  TurnMedium(90, 1000);
  FireCatapult();
  /*
  DigitalOutC.set(true);
  double end_time = Brain.timer(msec);
  Brain.Screen.newLine();
  Brain.Screen.printAt(80, 80, "%f", end_time - begin_time);
  */
  thread PullCatapult4 = thread(PullCatapult);
  thread grabcheck2 = thread(GrabCheck);
  TurnMedium(-90, 1000);
  while(LimitSwitchA.pressing() == 0) {
    wait(10, msec);
  }
  DriveTo(12, 0, 1000);
  double remainder_time = Brain.timer(msec);
  Brain.Screen.print(remainder_time);
  TurnMedium(80, 1000);
  FireCatapult();
  Grab(0);
  DigitalOutC.set(true);
  
  double end_time = Brain.timer(msec);
  Brain.Screen.newLine();
  Brain.Screen.printAt(110, 110, "%f", end_time - begin_time);
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
  bool setup_catapult = true;
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
      catapult_motor.spin(fwd, 100 * 0.128, volt);
    } else if(LimitSwitchA.pressing() == 0) {
      if (setup_catapult) {
        catapult_motor.spin(fwd, 100 * 0.128, volt);
      } else {
        catapult_motor.spin(directionType::rev, 1 * 0.128, volt);
      }
    } else {
      catapult_motor.stop(hold);
    }

//=========================================================================
    if (R2 && LimitSwitchA.pressing() == 1) {
      intake_motor.spin(fwd, 0.128 * 75, voltageUnits::volt);
    } else if(R1) {
      intake_motor.spin(fwd, 0.128 * -100, voltageUnits::volt);
    } else { 
      intake_motor.stop(hold);
    }
   
    if (L1) {
      if (L2) {
         DigitalOutD.set(true);
         DigitalOutE.set(true);
      }
    } 

    if (BtnY) {
      catapult_motor.spin(fwd,100*0.128, volt);
      wait(100, msec);
      DigitalOutC.set(true);
    } 

    if (BtnX) {
      setup_catapult = false;
    }

    if (BtnB) {
      setup_catapult = true;
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
 