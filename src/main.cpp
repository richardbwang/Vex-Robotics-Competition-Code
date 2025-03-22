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
// left_chassis1        motor         17               
// left_chassis2        motor         19               
// left_chassis3        motor         20               
// right_chassis1       motor         11              
// right_chassis2       motor         12               
// right_chassis3       motor         13               
// intake_motor         motor         9              
// InertialA            inertial      3              
// mogo_mech            digital_out   E               
// clipper              digital_out   B               
// Optical              optical       15              
// intakeraise          digital_out   A               
// Doinker              digital_out   C               
// distance_sensor      distance      2              
// clip_sensor          distance      5
// arm_motor            motor         21              
// X                    rotation      1               
// Y                    rotation      10               
// Vision1              vision        7              
// arm_stop             bumper        F           
// aiVisionArm          aivsion       8
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

void arm_touch() {
  wait(100, msec);
  if (arm_stop.pressing()) {
    arm_motor.setPosition(-23, deg);
    wait(10, msec);
  }
}

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
  // InertialA.setRotation(-rushsetupangle, degrees);
  // InertialA.setRotation(rushsetupangle, degrees);
  thread track_odom = thread(trackodomwheel);
  Controller1.Screen.clearScreen();
  while(!Competition.isEnabled()) {
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print(xpos);
    Controller1.Screen.print(", ");
    Controller1.Screen.print(ypos);
    Controller1.Screen.print(", ");
    Controller1.Screen.print(InertialA.rotation());
    wait(100, msec);
    Controller1.Screen.clearLine();
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
/*-------------------------------
--------------------------------------------*/

void autonomous(void) {
  arm_stop.pressed(arm_touch);
  AutonSelected = 6;
  switch(AutonSelected) {
    case 1:
    // 1 red awp right
      // AwpStake(true);
      // // elim red;
      isRed = true;
      // AwpPositive();
      NegativeAWP();
      // SigSoloAWP();
      // TestColorSort();
      break;
    case 2:
    // 2 blue awp left
      // AwpStake(false);
      isRed = true;
      SigSoloAWP();
      break;  
    case 3:
      // goal rush
      // GoalRushStraight();
      isRed = true;
      NegativeElim();
      break;
    case 4:
      // 4 blue awp left
      isRed = false;
      SetupAwp();
      break;
    case 5:
      isRed = false;
      AwpPositive(); // 5 red elim right
      break;
    case 6:
      isRed = true;
      GoalRush(); // 5 blue elim left
      break;
    case 7:
      isRed = true;
      skills();
      break;
    case 8:
      isRed = true;
      SetupGoalRush();
      break;
    case 9: 
      testPID();
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
bool first_time = true;
int dipan_flag = 0, hang_flag = 0, wing_flag = 0;
int xi_flag = 0, xi_flag1 = 0;
int intake_speed = 12;
double temp = 0;
bool wrong_color = false;
bool raising = false;
int arm_click = 0;
double arm_load_pos = 80;
double arm_score_pos = 410;
bool intake_back = false;

bool detectcolor(bool isred){
  if (isred == true){
    return (Optical.hue() < 300 and Optical.hue() > 90);
  }
  return (Optical.hue() > 340 or Optical.hue() < 60);
}

void intakeBack() {
  intake_motor.spinFor(reverse, 30, degrees, 100, velocityUnits::pct, true);
  wait(100, msec);
  intake_back = false;
}

void armToLoadPos() {
  arm_motor.spinToPosition(arm_load_pos, deg, 100, velocityUnits::pct, true);
  arm_motor.spinToPosition(arm_load_pos, deg, 50, velocityUnits::pct, true);
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("arm: %.2f", arm_motor.position(deg));
  raising = false;
}

void armToScoreTh() {
  intake_motor.spinFor(reverse, 20, degrees, 100, velocityUnits::pct, true);
  intake_back = false;
  arm_motor.spinToPosition(arm_score_pos, deg, 100, velocityUnits::pct, true);
  raising = false;
}

void usercontrol(void) {
  // thread icr = thread(intake_color_red);
  arm_stop.pressed(arm_touch);
  arm_motor.setPosition(arm_load_pos - 7, deg);

  // friction_test();
  Stop(coast);
  headingcorrection = false;
  //Brain.Screen.clearScreen();
 
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

    // Controller1.Screen.clearScreen();
    // Controller1.Screen.setCursor(1, 1);
    // aiVisionArmRed.takeSnapshot(redStakeColor);
    // if (aiVisionArmRed.objects[0].exists) {
    //   Controller1.Screen.print("Cen: %d, w: %d", aiVisionArmRed.largestObject.centerX, aiVisionArmRed.largestObject.width);
    // } else {
    //   Controller1.Screen.print("no object");
    // }

    // Controller1.Screen.clearScreen();
    // Controller1.Screen.setCursor(1, 1);
    // Controller1.Screen.print("arm: %0.1f", arm_motor.position(deg));

    if(abs(Ch4) < 12 && abs(Ch3) > 12) {
      ChassisControl(Ch3 * 0.12 * 1.5, Ch3 * 0.12 * 1.5);
      dipan_flag = 0;
    } else if(abs(Ch4) >= 12 ) {
      Ch4 *= (0.5 + 0.2 * (abs(Ch3) / 100.0));
      //Ch4 *= 0.5;
      ChassisControl((Ch3 + Ch4) * 0.12 * 1, (Ch3 - Ch4) * 0.12 * 1);
      dipan_flag = 1;
    } else {
      Stop(dipan_flag == 0 ? coast : brake);
    }

    if(abs(Ch2) < 30 && Ch1 < -50){
      if (!raising && fabs(arm_motor.position(deg) - arm_load_pos) > 10) {
        raising = true;
        thread armToLoad = thread(armToLoadPos);
      }
    } else if(abs(Ch1) < 40 && Ch2 > 50) {
      if (fabs(arm_motor.position(deg) - arm_load_pos) <= 25) {
        if (!intake_back) {
          intake_back = true;
          thread intakeBackThread = thread(intakeBack);
        }
      }
      arm_motor.spin(fwd, 12, volt);
      raising = false;
    } else if(abs(Ch1) < 32 && Ch2 < -50){
      arm_motor.spin(fwd, -12, volt);
      raising = false;
    } else if(abs(Ch2) < 30 && Ch1 > 50){
    } else if(!raising) {
      arm_motor.stop(hold);
    }

    if(BtnY) {
    }

    if (BtnU){
    } else if (BtnR){
    }

    if (R2){
      intake(12);
    } else if(R1) {
      intake(-12);
    } else {
      if (!intake_back) {
        intake_stop(hold);
      }
      Optical.setLight(ledState::off);
    }

    if (L1) {
      mogo_mech.set(false);
    } else if(L2) {
      mogo_mech.set(true);
    }
    
    if (BtnX){
      intakeraise.set(true);
    }
    if (BtnY){
      intakeraise.set(false);
    }
    if (BtnA){
      Doinker.set(true);
    }else if(BtnB){
      Doinker.set(false);
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
 