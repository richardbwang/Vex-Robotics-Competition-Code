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
// hang_motor           motor         2               
// intake_motor         motor         7               
// right_chassis1       motor         17              
// right_chassis2       motor         18              
// left_chassis1        motor         12              
// left_chassis2        motor         13              
// right_chassis3       motor         19              
// left_chassis3        motor         14              
// InertialA            inertial      15              
// arm_motor            motor         8               
// catapult_motor       motor         4               
// Distance5            distance      5               
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
  thread track_odom = thread(trackodom);
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
  AutonSelected = 9;
  switch(AutonSelected) {
    case 1:
      Far4Bar();
      break;
    case 2:
      Far5();
      break;
    case 3:
      Near();
      break;
    case 4:
      NearElim();
      break;
    case 5:
      ProgSkills();
      break;
    case 6:
      NearAWP();
      break;
    case 7:
      Far3();
      break;
    case 8:
      tag();
      break;
    case 9:
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
int dipan_flag = 0;
int xi_flag = 0, xi_flag1 = 0;

void usercontrol(void) {
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
    if(BtnA) {
      catapult_motor.spin(fwd, -100 * 0.128, volt);
    } else if(Distance5.value() > distance_value) {
      catapult_motor.spin(fwd, -100 * 0.128, volt);
    } else {
      catapult_motor.stop(hold);
    }
    
    if(BtnX) {
      DigitalOutA.set(true);
      DigitalOutB.set(true);
    } else if(BtnY) {
      DigitalOutA.set(false);
      DigitalOutB.set(false);
    }

    if(abs(Ch4) < 12 && abs(Ch3) > 12 ) {
      DriveControl(Ch3, Ch3);
      dipan_flag = 0;
    } else if(abs(Ch4) >= 12 ) {
      DriveControl((Ch3 + Ch4) * 0.55, (Ch3 - Ch4) * 0.55);
      dipan_flag = 1;
    } else {
      Stop(dipan_flag == 0 ? coast : brake);
    }
//=========================================================================

//=========================================================================
    if (R2) {
      intake_motor.spin(fwd, 0.128 * 100, voltageUnits::volt);
    } else if(R1) {
      intake_motor.spin(fwd, 0.128 * -100, voltageUnits::volt);
    } else { 
      intake_motor.stop(hold);
    }
   
    if (L1) {
      arm_motor.spin(fwd, 0.128 * -100, voltageUnits::volt);
    } else if (L2) {
      arm_motor.spin(fwd, 0.128 * 100, voltageUnits::volt);
    } else {
      arm_motor.stop(coast);
    }
    
    wait(10, msec); 
  }
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
  wait(4000, msec);
  thread headingcorrection = thread(heading_correction);
  thread pull_catapult = thread(pullcatapult);
  double begin_time = Brain.timer(msec);
  CurveCircle(90, 13, 1500, false);
  Grab(-100);
  DriveTo(20, 1500);
  Swing(50, -1, 700);
  CurveCircle(35, 13, 1500, false);
  Swing(160, -1, 1000);
  Arm(100);
  wait(500, msec);
  Arm(0);
  catapultlaunch(47, 1000);
  thread pull_catapult1 = thread(pullcatapult);
  Arm(-100);
  wait(500, msec);
  TurnToAngle(30, 1000);
  CurveCircle(0, 40, 1000, false);
  CurveCircle(-10, 300, 2000, false);
  DigitalOutB.set(true);
  CurveCircle(-90, 40, 1500, false);
  DriveTo(-10, 800);
  DigitalOutB.set(false);
  CurveCircle(-55, 20, 1000, false);
  CurveCircle(-65, 600, 1000);
  TurnToAngle(-40, 500);
  CurveCircle(-90, 70, 1500);
  TurnToAngle(0, 800);
  DigitalOutB.set(false);
  DriveTo(60, 3000);
  DigitalOutA.set(true);
  DigitalOutB.set(true);
  CurveCircle(-60, 10, 1000, false);
  CurveCircle(-65, 100, 1000, false);
  Swing(0, -1, 800);
  DriveTo(-25, 800);
  DigitalOutA.set(false);
  DigitalOutB.set(false);
  TurnToAngle(-30, 800);
  CurveCircle(-35, -350, 1000, false);
  CurveCircle(-90, -20, 2000);
  DigitalOutA.set(true);
  DigitalOutB.set(true);
  CurveCircle(-85, -100, 1000, false);
  CurveCircle(-10, -30, 2000, false);
  CurveCircle(0, -250, 1200), false;
  DriveTo(-25, 800);
  DigitalOutA.set(false);
  DigitalOutB.set(false);
  CurveCircle(10, 90, 1000, false);
  Swing(80, 1, 1000);
  CurveCircle(90, 150, 1000);
  TurnToAngle(40, 800);
  DigitalOutA.set(true);
  DigitalOutB.set(true);
  CurveCircle(30, 200, 1700, false);
  Swing(0, -1, 1000);
  DriveTo(-25, 800);
  DigitalOutB.set(false);
  TurnToAngle(-110, 800);
  CurveCircle(90, -25, 2500, false);
  DriveTo(-25, 1500);
  DigitalOutA.set(false);
  CurveCircle(55, -20, 1000, false);
  CurveCircle(65, -600, 1000);
  TurnToAngle(40, 500);
  CurveCircle(90, -70, 1500);
  double end_time = Brain.timer(msec);
  Brain.Screen.newLine();
  Brain.Screen.printAt(80, 80, "%f", end_time - begin_time);
  */
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
 