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
// left_chassis1        motor         13              
// left_chassis2        motor         14              
// left_chassis3        motor         15              
// right_chassis1       motor         19              
// right_chassis2       motor         6               
// right_chassis3       motor         18              
// right_intake         motor         2               
// inertial_sensor      inertial      5               
// left_intake          motor         1               
// goal_clamp           digital_out   A               
// PTO                  digital_out   D               
// optical_sensor       optical       21              
// distance_sensor_arm  distance      9               
// hang_deploy          digital_out   F               
// filter               digital_out   E               
// distance_sensor      distance      17              
// Sort                 digital_out   B               
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
  point = (inertial_sensor.rotation(degrees));
  
  // Initializing Robot Configuration
  vexcodeInit();
  
  //calibrate inertial sensor
  inertial_sensor.calibrate();

  // waits for the Inertial Sensor to calibrate
  while (inertial_sensor.isCalibrating()) {
    wait(100, msec);
  }

  double current_heading = inertial_sensor.heading();
  Brain.Screen.print(current_heading);
  ResetChassis();
  //inertial_sensor.setRotation(-12.5, degrees);
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
/*-------------------------------
--------------------------------------------*/

bool isred=false;

void autonomous(void) {
  AutonSelected = 7;
  switch(AutonSelected) {
    case 1:
    // 1 red awp right
      BlueLeft();
      isred = false;
      break;
    case 2:
      // 2 red awp left
      AwpStake(false);
      isred=true;
      break;  
    case 3:
      // 3 blue awp right
      AwpStake(true);
      isred=false;
      break;
    case 4:
      // 4 blue awp left
      AwpStake(false);
      isred=false;
      break; 
    case 5:
      RedElimRing(); // 5 red elim right
      isred=true;
      break;
    case 6:
      BlueElimRing(); // 5 blue elim left
      isred=false;
      break;
    case 7:
      skills();
      isred=true;
      break;
    case 8:
      test();
      isred=true;
      break;
    case 9:
      TestDriveMotors();
      isred = true;
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
int chassis_flag = 0, hang_flag = 0, wing_flag = 0;
int intake_speed = 12;
double temp = 0;
bool wrong_color = false;
int counter=0;

bool detectcolor(bool isred){
  if (isred == true){
    return (optical_sensor.hue() < 300 and optical_sensor.hue() > 90);
  }
  return (optical_sensor.hue() > 340 or optical_sensor.hue() < 60);
}

void usercontrol(void) {
  //pre_auton();
  //wait(3000, msec);
  skills();
  /*
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
    Brain.Screen.setCursor(1, 1);

//=========================================================================
    if(abs(Ch4) < 12 && abs(Ch3) > 12) {
      ChassisControl(Ch3 * 0.12 * 1.5, Ch3 * 0.12 * 1.5);
      chassis_flag = 0;
    } else if(abs(Ch4) >= 12 ) {
      Ch4 *= (0.5 + 0.2 * (abs(Ch3) / 100.0));
      //Ch4 *= 0.5;
      ChassisControl((Ch3 + Ch4) * 0.12 * 1, (Ch3 - Ch4) * 0.12 * 1);
      chassis_flag = 1;
    } else {
      Stop(chassis_flag == 0 ? coast : brake);
    }
    if (R2 or BtnA) {
      intake(intake_speed);
      if (R2){
        PTO.set(false);
        optical_sensor.setLight(ledState::on);
        optical_sensor.setLightPower(100);
        if (optical_sensor.isNearObject() && detectcolor(isred)) {
          wrong_color = true;
         }
        if (distance_sensor.objectDistance(distanceUnits::mm) < 50 && wrong_color) {
          intake_stop(hold);
          task::sleep(400);
          wrong_color = false;
        }
      }else if(BtnA){
        PTO.set(true);
      }
    } else if(R1 or BtnB) {
      intake(-1 * intake_speed);
      if (R1){
        PTO.set(false);
      }else if(BtnB){
        PTO.set(true);
      }
    } else { 
      intake_stop(hold);
      optical_sensor.setLight(ledState::off);
    }
    if (L2) {
      goal_clamp.set(true);
    } else if(L1) {
      goal_clamp.set(false);
    }
    if (BtnX){
      PTO.set(false);
      if (counter==0){
        if (optical_sensor.isNearObject()) {
          intake(0);
          counter=15;
        }else{
          intake(12);
        }
      }else{
        intake(-9);
        counter--;
      }
      if (counter<0){
        counter=0;
      }
    }
    if (BtnY){
      hang_deploy.set(true);
    }
    if (BtnU){
      hang_deploy.set(false);
    }
    wait(10, msec); 
  }
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
 