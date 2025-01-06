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
// InertialA            inertial      14              
// mogo_mech            digital_out   E               
// clipper              digital_out   B               
// Optical              optical       15              
// intakeraise          digital_out   A               
// Doinker              digital_out   C               
// distance_sensor      distance      2               
// arm_motor            motor         21              
// X                    rotation      1               
// Y                    rotation      10              
// arm_stop             bumper        F               
// Sort                 digital_out   H               
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
  wait(500, msec);
  if (arm_stop.pressing()) {
    arm_motor.setPosition(-15, deg);
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
  InertialA.setRotation(-rushsetupangle, degrees);
  //InertialA.setRotation(rushsetupangle, degrees);
  thread track_odom = thread(trackodomwheel);
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
  AutonSelected = 3;
  isRed = true;
  switch(AutonSelected) {
    case 1:
    // 1 red awp right
      // AwpStake(true);
      // // elim red;
      SigSoloAWP();
      break;
    case 2:
    // 2 blue awp left
      // AwpStake(false);
      AwpStake(true);
      break;  
    case 3:
      // goal rush
      GoalRushStraight();
      break;
    case 4:
      // 4 blue awp left
      skills();
      break;
    case 5:
      ElimMogoRed(); // 5 red elim right
      break;
    case 6:
      ElimRing(false); // 5 blue elim left
      break;
    case 7:
      skills();
      break;
    case 8:
      SetupGoalRush();
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
bool first_time = true;
int dipan_flag = 0, hang_flag = 0, wing_flag = 0;
int xi_flag = 0, xi_flag1 = 0;
int intake_speed = 12;
double temp = 0;
bool wrong_color = false;
bool arm_ready = false, raising = false;;

bool detectcolor(bool isred){
  if (isred == true){
    return (Optical.hue() < 300 and Optical.hue() > 90);
  }
  return (Optical.hue() > 340 or Optical.hue() < 60);
}

void raiseBackpack() {
  intake(12);
  wait(600, msec);
  clipper.set(true);
  intake(-12);
  wait(100, msec);
  intake_stop();
  wait(100, msec);
  while(arm_ready) {
    arm_pid(arm_store_target);
    wait(10, msec);
  }
  raising = false;
}

void usercontrol(void) {
  arm_stop.pressed(arm_touch);
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
    //Controller1.Screen.clearScreen();
    //Controller1.Screen.setCursor(1, 1);
    //Controller1.Screen.print("X: %0.1f Y: %0.1f arm: %d", xpos, ypos);
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

    if(!raising && arm_ready && distance_sensor.objectDistance(mm) < 50) {
      raising = true;
      thread rb = thread(raiseBackpack);
    }
    if(!raising && abs(Ch2) < 30 && Ch1 < -50){
      clipper.set(false);
      arm_pid(arm_load_target);
      arm_ready = true;
    } else if(abs(Ch1) < 30 && Ch2 > 50) {
      arm_motor.spin(fwd, 12, volt);
      arm_ready = false;
    } else if(abs(Ch1) < 30 && Ch2 < -50){
      arm_motor.spin(fwd, -12, volt);
      arm_ready = false;
    } else if(abs(Ch2) < 30 && Ch1 > 50){
      arm_pid(arm_score_target);
      arm_ready = false;
    } else if(!raising) {
      arm_motor.stop(hold);
    }

    if(BtnB) {
      arm_motor.stop(coast);
      arm_motor.setPosition(0, deg);
    }

    if (BtnU){
      clipper.set(false);
    } else if (BtnR){
      clipper.set(true);
    }

    if (R2 && !raising){
      intake(12);
    } else if(R1) {
      intake(-12);
    } else if(!raising) {
      intake_stop(hold);
      Optical.setLight(ledState::off);
    }

    if (L2) {
      mogo_mech.set(false);
    } else if(L1) {
      mogo_mech.set(true);
    }
    
    if (BtnL){
      intakeraise.set(true);
    }
    if (BtnD){
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
 