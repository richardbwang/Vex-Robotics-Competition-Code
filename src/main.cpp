/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Richard Wang                                              */
/*    Created:      Thu August 2, 2022                                        */
/*    Description:  V1 code for 08/12 signature event using                   */
/*                  competition template.                                     */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// left_fw_motor        motor         8               
// left_chassis1        motor         3               
// right_chassis1       motor         2               
// LimitSwitchA         limit         A               
// lift_down            motor         9               
// right_fw_motor       motor         5               
// left_chassis2        motor         4               
// right_chassis2       motor         1               
// grab_motor           motor         6               
// InertialA            inertial      10              
// DigitalOutG          digital_out   G               
// VisionA              vision        19              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "motor-control.h"
#include "math.h"
#include "auto.h"

#include <cstdio>
#include <iostream>
#include <iomanip>
#include <fstream>

using namespace vex;

// A global instance of competition
competition Competition;

std::ofstream ofs;

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
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // Roll the first roller.
  double start_time = Brain.timer(msec);
  fw_rpm(75, 78);
  DriveFor(27, 2000);
  TurnForAngle(88, 700);
  wait(50, msec);
  for(int i = 0; i < 2; i++) {
    liftDown(100);
    wait(80, msec);
    liftDown(-50);
    wait(80, msec);
    liftDown(0);
    lift_down.resetRotation();
    for (int j = 0; j < 3; j++) {
      fw_pid_rpm_with_time_limit(458, 458, 220);
    }
    if (i == 0) {
      wait(400, msec);
    }
  }
  fw_rpm(0, 0);

  grab(100);
  DriveFor(5, 500);
  ChassisControl(55,  55);
  wait(300, msec);
  grab(0);
  DriveFor(-6, 1000);
  TurnForAngle(130, 1000);

  grab(-100);
  DriveFor(33, 2100);
  wait(200, msec);
  DriveFor(34, 2300);
  fw_rpm(68, 68);
  wait(200, msec);
  TurnForAngle(-97, 800);
  DriveFor(-7, 800);
  double target_rpm = 400;
  for(int i = 0; i < 3; i++) {
    liftDown(100);
    wait(80, msec);
    liftDown(-50);
    wait(80, msec);
    liftDown(0);
    lift_down.resetRotation();
    for (int j = 0; j < 3; j++) {
      fw_pid_rpm_with_time_limit(target_rpm, target_rpm + 10, 220);
    }
    if (i < 2) {
      wait(350, msec);
    }
    grab(0);
  }
  fw_rpm(0, 0);

  Brain.Screen.printAt(100, 100, "Finihsed %f",  (Brain.timer(msec) - start_time) / 1000);
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

// Controller axises, Ch1: Axis1, Ch2: Axis2, Ch3: Axis3, Ch4: Axis4
int Ch1, Ch2, Ch3, Ch4;

// Controller buttons
bool L1, L2, R1, R2, BtnA, BtnB, BtnX, BtnY, BtnU, BtnD, BtnL, BtnR; 

// Flags for flying wheels.
int fw_flag = 0, fw_flag1 = 0;
int fw_time = 0;

// Flags for push bar.
int lift_flag = 0, lift_flag1 = 0;

// Flags for shooting, either short or log distance.
int fly_flag = 0 , fly_flag1 = 0;
int long_fly_flag = 0,long_fly_flag1 = 0;

int i=471;
int last_s_l=0, last_s_r = 0;

bool cylinder_on = true;
int first_shoot = 0;
int second_shoot = 0;
int xxxx = 1, yyyy = 1;

void usercontrol(void) {   
  // User control code here, inside the loop
  // Brain.Screen.clearScreen(black);
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
    
    // Get the current values from joysticks and all buttons. 
    // We mianly use Ch3 and Ch4 for controlling the robot movement.
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
    // Control the moving direction while allowing certain percentage noices.
    if(abs(Ch4) < 5 && abs(Ch3) > 10 ) {
      // Go straight. 5 is the threshold for ignoring the turning signal. 
      // 10 is the threshold for go straight.
      ChassisControl(Ch3 ,  Ch3 );
    } else if(abs(Ch4) > 5) {
      // Turn if the Ch4 signal is over the threshold.
      if (fw_flag == 1 || BtnB || BtnA) {
        // Slow the turn since we are about to shoot.
        // ChassisControl((Ch3 - Ch4) * 0.5, (Ch3 + Ch4) * 0.35);
        ChassisControl((Ch3 - Ch4) * 0.3, (Ch3 + Ch4) * 0.20);
      } else {
        // Turn normally.
        ChassisControl((Ch3 - Ch4) * 0.65, (Ch3 + Ch4) * 0.65 );
      }
    } else {
      // No operation from the joystick, stop using the coast mode.
      Stop(coast);
    }
      
    if(BtnX && fly_flag1 == 0 && fly_flag == 0) {
      fly_flag = 1;
      fly_flag1 = 1;
    } else if(BtnX && fly_flag1 == 0 && fly_flag == 1) {
      fly_flag = 0;
      fly_flag1 = 1;
    } else if(!BtnX) {
      fly_flag1 = 0;
    }

    if (BtnY) {
      Brain.Screen.clearScreen(black);
      xxxx = 1;
      yyyy = 1;
      //AimingGoal();
    }

    /*
    if(BtnY && long_fly_flag1 == 0 && long_fly_flag == 0) {
      long_fly_flag = 1;
      long_fly_flag1 = 1;
    } else if(BtnY && long_fly_flag1 == 0 && long_fly_flag == 1) {
      long_fly_flag = 0;
      long_fly_flag1 = 1;
    } else if(!BtnY) {
      long_fly_flag1 = 0;
    }*/
    
    if (L1) {
      if (L2) {
         DigitalOutG.set(true);
      }
    } else if (L2) {
      DigitalOutG.set(false);
    }
 
    // BtnB will shot continueously
    if(BtnB) {
      double l_rpm = left_fw_motor.velocity(rpm);
      double r_rpm = right_fw_motor.velocity(rpm);
      fly_flag = 0;
      double target_rpm = 405;
      if (first_shoot) {
        first_shoot--;
      } else {
        target_rpm = 385;
      }
      for (int i = 1; i < 3; i++) {
        fw_pid_rpm(target_rpm, target_rpm);
      }
      wait(100, msec);       
      if(fw_flag1 == 0) {
        fw_flag = 1;
        fw_flag1 = 1;
      }
      
      if(fw_flag == 1 && lift_flag1 == 0) {
        fw_time = Brain.timer(msec);
        lift_flag1 = 1;
      }
      
      double c_l_rpm = left_fw_motor.velocity(rpm);
      double c_r_rpm = right_fw_motor.velocity(rpm);

      if(fw_flag == 1 && Brain.timer(msec) - fw_time < 1) {  
        liftDown(100);  
        wait(30, msec); 
      } else if(LimitSwitchA.pressing() == 0) {
        liftDown(-35);
        lift_flag = 1;   
      }

      if(LimitSwitchA.pressing() == 1 && lift_flag == 1) {
        fw_flag = 0;
        fw_flag1 = 0;
        lift_flag = 0;
        lift_flag1 = 0;
        fw_time = 0;
        lift_down.resetRotation();
        liftDown(0); 
      }
      
      double f_l_rpm = left_fw_motor.velocity(rpm);
      double f_r_rpm = right_fw_motor.velocity(rpm);


      Brain.Screen.printAt(15, 15 * (yyyy++), "%d L:%.2f %.2f %.2f| R:%.2f %.2f %.2f", 
                           xxxx, l_rpm, c_l_rpm, f_l_rpm, r_rpm, c_r_rpm, f_r_rpm);     
      xxxx++;
    }
        
    //========================================================
    if(fly_flag == 1 && !BtnB) {
      fw_rpm(57, 57);
      first_shoot = 4;
      second_shoot = 2;
      long_fly_flag = 0;
    } else if(long_fly_flag == 1) {
      fw_rpm(100, 100);
      fly_flag = 0;
    } else if(BtnD) {
      fw(-50, -50);
    } else if(!BtnB && fly_flag == 0 && long_fly_flag == 0) {
      fw(0, 0);
    }  
      
    // Single shot
    if(BtnA && !BtnB) { 
      // Stablize the motor speed first for single shot.
      fw_rpm(67, 67);
      liftDown(100);
    }

    if(!BtnA && LimitSwitchA.pressing() == 0 && !BtnB) {
      liftDown(-20);
    } else if(!BtnA && !BtnB) {
      fw_flag = 0;
      fw_flag1 = 0;
      lift_flag = 0;
      lift_flag1 = 0;
      fw_time = 0;
      lift_down.resetRotation();
      liftDown(0);
    }

    //========================================================
    if(R1) {
      grab(100);
    } else if(R2) {
      grab(-100);
    } else {
      grab(0);
    }
    
    /*
    if(fly_flag == 1 || BtnB || long_fly_flag == 1 ) {
      if(i >= 470) {

        Brain.Screen.clearScreen(black);
        Brain.Screen.setPenColor(white);
        Brain.Screen.drawLine(20,1,20,230);
        Brain.Screen.drawLine(20,230,272,230);
        Brain.Screen.setPenColor(blue);
        Brain.Screen.drawLine(211,20,220,20);
        Brain.Screen.printAt(221, 20, ":LEFT");
        Brain.Screen.setPenColor(green);
        Brain.Screen.drawLine(231,20,240,20);
        Brain.Screen.printAt(241, 20, ":RIGHT");
        
        i=0;
      } else {

        Brain.Screen.setPenColor(red);
        Brain.Screen.drawLine(i + 20, 220-left_fw_motor.velocity(rpm) / 3,i + 20, 220- last_s_l / 3);

        Brain.Screen.setPenColor(green);
        Brain.Screen.drawLine(i + 20, 30 + right_fw_motor.velocity(rpm) / 3, i + 20, 30 + last_s_r / 3);

        i++;
        last_s_l = left_fw_motor.velocity(rpm);
        last_s_r = right_fw_motor.velocity(rpm);
      }
    } else {
      last_s_l=0;
      last_s_r=0;
    }*/

    // Sleep the task for a short amount of time to prevetning wasting 
    // too much resources.
    wait(10, msec); 
  }
}

void SystemChecks() {
  // System Checks Battery FieldControl etc ...
  Brain.Screen.print(" ********** System Checks ********** ");
  if(Competition.isEnabled() == true) { // Checks if competition mode is enabled
        Brain.Screen.setFillColor(green);
          Brain.Screen.print("Competition Mode is Enabled Do your best out there :)");
  }

  if(Competition.isCompetitionSwitch() == true) {
    Brain.Screen.print("Connected to the Competition Switch ✅");  
  }  else if(Competition.isCompetitionSwitch() == false) {
     Brain.Screen.print("Not Connected to the Competition Switch ❌");
  }
  
  // Checks if the field controller is connected to the controller.
  if(Competition.isFieldControl() == true) {
    Brain.Screen.setFillColor(green);
    Brain.Screen.print("Connected to the Field Control System ✅");   
  } else {
    Brain.Screen.setFillColor(red);
    Brain.Screen.print("Not Connected to the Field Control System ❌");  
  }
  

  int Battery = Brain.Battery.capacity();
  if (Battery >= 80) { 
    // Checks if the battery capacity is equal to 80% or above 80%.
    Brain.Screen.setFillColor(green);
    Brain.Screen.setCursor(1,4);
    Brain.Screen.print("Battery Percent Above 80% Shouldn't need charging. ✅"); 
    
  } else if (Battery <= 25){ // Checks if the battery capacity is equal to 25% or below 25%.
    Brain.Screen.setFillColor(red);
    Brain.Screen.setCursor(1,4);
    Brain.Screen.print("Battery NEEDS charging RIGHT NOW. ❌");
  }
  Brain.Screen.print(" ********** System Checks DONE!!! ");
  //... End of Checks Start Code...
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // SystemChecks();
  // pre_auton();

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
 