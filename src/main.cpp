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
  wait(200, msec);
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
  // InertialA.setRotation(rushsetupangle, degrees);
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
bool raising = false;
int arm_click = 0;
int clipper_sensor_cnt = 0;
double arm_load_pos = 47;

bool detectcolor(bool isred){
  if (isred == true){
    return (Optical.hue() < 300 and Optical.hue() > 90);
  }
  return (Optical.hue() > 340 or Optical.hue() < 60);
}

void armToLoadPos() {
  raising = true;
  arm_pid_target = arm_load_target;
  arm_pid(arm_pid_target);
  //arm_motor.spinToPosition(arm_load_pos, deg, 100, velocityUnits::pct, true);
  //arm_motor.spinToPosition(arm_load_pos, deg, 50, velocityUnits::pct, true);
  // Controller1.Screen.clearScreen();
  // Controller1.Screen.setCursor(1, 1);
  // Controller1.Screen.print("arm: %.2f", arm_motor.position(deg));
  raising = false;
}

void usercontrol(void) {
  // thread icr = thread(intake_color_red);
  arm_stop.pressed(arm_touch);
  arm_motor.setPosition(arm_load_pos, deg);

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
    // Controller1.Screen.print("X: %0.1f Y: %0.1f", gps1.xPosition(inches), gps1.yPosition(inches));

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
      clipper.set(false);
      if (!raising) {
        thread armToLoad = thread(armToLoadPos);
      }
    } else if(abs(Ch1) < 32 && Ch2 > 50) {
      if (clip_sensor.objectDistance(mm) < 90 && arm_motor.position(deg) >=37 && arm_motor.position(deg) <= 57) {
        intake_motor.spinFor(reverse, 30, degrees, 100, velocityUnits::pct, false);
        wait(100, msec);
      }
      arm_motor.spin(fwd, 12, volt);
      raising = false;
    } else if(abs(Ch1) < 32 && Ch2 < -50){
      if (clip_sensor.objectDistance(mm) > 100) {
        clipper.set(false);
      }
      arm_motor.spin(fwd, -12, volt);
      raising = false;
    } else if(abs(Ch2) < 30 && Ch1 > 50){
    } else if(!raising) {
      arm_motor.stop(hold);
    }

    if(BtnB) {
      // MoveToObject(aiVisionArmBlue, blueStakeColor, 160, 82, 1, 10000, true, 8);
      MoveToObject(aiVisionArmWall, wallStakeColor, 160, 92, 1, 10000, true, 8);
      Stop(vex::hold);

      aiVisionArmRed.takeSnapshot(redStakeColor);
      // aiVisionArm.takeSnapshot(wallStakeColor);
      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(1, 1);
      if (aiVisionArmRed.objects[0].exists) {
        Controller1.Screen.print("Cen: %d, w: %d", aiVisionArmRed.largestObject.centerX, aiVisionArmRed.largestObject.width);
        // arm_motor.spin(fwd, 12, volt);
        // wait(600, msec);
        // arm_motor.stop(coast);
        // DriveTo(-5, 1000, true, 5);
      } else {
        Controller1.Screen.print("no object");
      }
    }

    if (BtnU){
      clipper.set(false);
    } else if (BtnR){
      clipper.set(true);
    }

    if (R2){
      intake(12);
      if (clip_sensor.objectDistance(mm) < 70 && arm_motor.position(deg) >= 37 && arm_motor.position(deg) <= 57 ) {
        clipper_sensor_cnt++;
        if (clipper_sensor_cnt > 5) {
          clipper.set(true);
          arm_motor.stop(coast);
          //arm_motor.setPosition(arm_load_target, deg);
          //arm_motor.setPosition(arm_load_pos, deg);
          clipper_sensor_cnt = 0;
        }
      }
    } else if(R1) {
      intake(-12);
    } else {
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
 