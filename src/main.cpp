/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Richard Wang (1698X)                                      */
/*    Created:      July 9, 2023                                              */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// intake_motor         motor         8               
// right_chassis1       motor         2               
// right_chassis2       motor         6               
// left_chassis1        motor         7               
// left_chassis2        motor         5               
// right_chassis3       motor         3               
// left_chassis3        motor         4               
// InertialA            inertial      21              
// catapult_motor       motor         15              
// awp_motor            motor         19              
// Distance13           distance      9               
// DigitalOutA          digital_out   A               
// DigitalOutB          digital_out   B               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "motor-control.h"
#include "math.h"
#include "autonomous.h"



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
  drawGUI();
  Brain.Screen.pressed(selectAuton);
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
  ResetChassis();
  thread track_odom = thread(trackodom);
  catapult_motor.setPosition(0, degrees);
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
  AutonSelected = 2;
  switch(AutonSelected) {
    case 1:
      Far6TopAntiDisruption();
      break;
    case 2:
      Far6LowAntiDisruption();
      break;
    case 3:
      NearAWP();
      break;
    case 4:
      Far6SafeNoBar();
      break;
    case 5:
      ProgSkills();
      break;
    case 6:
      Far6SafeBar();
      break;
    case 7:
      Far3();
      break;
    case 8:
      tag();
      break;
    case 9:
      TestDriveMotors();
      break;
    case 10:
      TestPID();
      break;
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
bool L1, L2, R1, R2, BtnA, BtnB, BtnX, BtnY, BtnU, BtnD, BtnL, BtnR;
int dipan_flag = 0, hang_flag = 0, wing_flag = 0;
int xi_flag = 0, xi_flag1 = 0;

void usercontrol(void) {
  /*
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  drawGUI();
  Brain.Screen.pressed(selectAuton);
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
  ResetChassis();
  thread track_odom = thread(trackodom);
  catapult_motor.setPosition(0, degrees);
  wait(3000, msec);
  ProgSkills();
  */
  /*
  awp_motor.stop(hold);
  dirchangestart = false;
  dirchangeend = false;
  boomerang(-25, -15, 90, 0.5, 2000, -1, false);
  Grab(-100);
  DriveToGoal(0, -1, 1000);
  MoveToPoint(-18, -9, 1, 2000, false);
  Arm(100);
  boomerang(-15, -13, -15, 0.3, 1500, -1);
  TurnToAngle(-15, 200);
  Arm(0);
  DigitalOutA.set(true);
  Grab(0);
  catapult_motor.spin(fwd, 12, volt);
  while(true) {
    if(Controller1.ButtonX.pressing() == true) {
      break;
    }
    wait(10, msec);
  }
  DigitalOutA.set(false);
  catapult_motor.stop(coast);
  */
  Stop(coast);
  headingcorrection = false;
  Brain.Screen.clearScreen();

  // User control code here, inside the loop  
  while (true) {
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

//=========================================================================
    if(abs(Ch4) < 12 && abs(Ch3) > 12) {
      DriveControl(Ch3 * 1.5, Ch3 * 1.5);
      dipan_flag = 0;
    } else if(abs(Ch4) >= 12 ) {
      DriveControl((Ch3 + Ch4) * 0.8, (Ch3 - Ch4) * 0.8);
      dipan_flag = 1;
    } else {
      Stop(dipan_flag == 0 ? coast : brake);
    }
//=========================================================================

//=========================================================================
    if(BtnA) {
      catapult_motor.spin(fwd, 12, volt);
    } else {
      catapult_motor.stop(coast);
    }
    if (R2) {
      intake_motor.spin(fwd, 12, voltageUnits::volt);
    } else if(R1) {
      intake_motor.spin(fwd, -12, voltageUnits::volt);
    } else { 
      intake_motor.stop(hold);
    }

    if(BtnB) {
      DigitalOutA.set(true);
      wing_flag = 1;
    }

    if(BtnX) {
      DigitalOutA.set(true);
      wing_flag = 0;
    } else if(wing_flag == 0) {
      DigitalOutA.set(false);
    }

    if(BtnY) {
      DigitalOutB.set(true);
    } else {
      DigitalOutB.set(false);
    }

    //Skills Controls
    /*
    if(BtnX) {
      DigitalOutA.set(true);
      DigitalOutB.set(true);
    } else {
      DigitalOutA.set(false);
      DigitalOutB.set(false);
    }
    */
    
    if (L1) {
      awp_motor.spin(fwd, -12, voltageUnits::volt);
      hang_flag = 0;
    } else if(L2) {
      awp_motor.spin(fwd, 12, voltageUnits::volt);
      hang_flag = 1;
    } else if(hang_flag == 0) { 
      awp_motor.stop(coast);
    }

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
 