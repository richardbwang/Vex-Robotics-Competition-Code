#include "vex.h"
#include "utils.h"
#include "pid.h"
#include <ctime>
#include <cmath>

#include "autonomous.h"
#include "motor-control.h"

int AutonSelected = 1;
int AutonMin = 1;
int AutonMax = 5;

void drawGUI() {
  // Draws 2 buttons to be used for selecting auto
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(1, 40, "Select Auton then Press Go");
  Brain.Screen.printAt(1, 180, "Auton Selected =  %d   ", AutonSelected);
  Brain.Screen.printAt(1, 215, "Auton Selected =  Left");
  Brain.Screen.setFillColor(red);
  Brain.Screen.drawRectangle(20, 50, 100, 100);
  Brain.Screen.drawCircle(300, 75, 25);
  Brain.Screen.printAt(25, 75, "Select");
  Brain.Screen.setFillColor(green);
  Brain.Screen.drawRectangle(170, 50, 100, 100);
  Brain.Screen.printAt(175, 75, "GO");
  Brain.Screen.setFillColor(black);
}

void selectAuton() {
  bool selectingAuton = true;

  int x = Brain.Screen.xPosition(); // get the x position of last touch of the screen
  int y = Brain.Screen.yPosition(); // get the y position of last touch of the screen
  // check to see if buttons were pressed

  if (x >= 20 && x <= 120 && y >= 50 && y <= 150) // select button pressed
  {
    AutonSelected++;
    if (AutonSelected > AutonMax)AutonSelected = AutonMin; // rollover
      
    Brain.Screen.printAt(1, 180, "Auton Selected =  %d   ", AutonSelected);

    switch(AutonSelected) {
    case 1:
      Brain.Screen.printAt(1, 215, "Auton Selected =  Left");
    break;

    case 2:
      Brain.Screen.printAt(1, 215, "Auton Selected =  Right");
    break;

    case 3:
      Brain.Screen.printAt(1, 215, "Auton Selected =  SoloAWP");
    break;

    case 4:
      Brain.Screen.printAt(1, 215, "Auton Selected =  Skills");
    break;

    case 5:
      Brain.Screen.printAt(1, 215, "Auton Selected =  PIDTest");
    break;
  }
  }
  if (x >= 170 && x <= 270 && y >= 50 && y <= 150) {
    selectingAuton = false; // GO button pressed
    Brain.Screen.printAt(1, 180, "Auton  =  %d   GO           ", AutonSelected);

    switch(AutonSelected) {
    case 1:
      Brain.Screen.printAt(1, 215, "Auton Selected =  Left");
    break;

    case 2:
      Brain.Screen.printAt(1, 215, "Auton Selected =  Right");
    break;

    case 3:
      Brain.Screen.printAt(1, 215, "Auton Selected =  SoloAWP");
    break;

    case 4:
      Brain.Screen.printAt(1, 215, "Auton Selected =  Skills");
    break;

    case 5:
      Brain.Screen.printAt(1, 215, "Auton Selected =  PIDTest");
    break;
  }
  }
  if (!selectingAuton) {
    Brain.Screen.setFillColor(green);
    Brain.Screen.drawCircle(300, 75, 25);
  } else {
    Brain.Screen.setFillColor(red);
    Brain.Screen.drawCircle(300, 75, 25);
  }

  wait(10, msec); // slow it down
  Brain.Screen.setFillColor(black);
}

void LeftAuton() {
  double begin_time = Brain.timer(msec);
  thread PullCatapult1 = thread(PullCatapult);
  thread headingcorrection = thread(heading_correction);
  thread grabcheck = thread(GrabCheck);
  
  //pick up disc and spin the roller
  DriveTo(6, 1000);
  Grab(20);
  DriveTo(-7, 1000);
  TurnBig(-135, 1200);
  Grab(0);
  DriveTo(4, 0, 1000);
  ResetChassis();
  isturning = true;
  Grab(-100);
  ChassisControl(20, 20);
  wait(200, msec);
  Grab(0);
  Stop(vex::hold);
  
  //shoot first 3 discs
  DriveToSeperate(-4 - ((GetLeftRotationDegree() / 360.0) * wheel_distance_in), -4 - ((GetRightRotationDegree() / 360.0) * wheel_distance_in), 1500);
  TurnSmall(34-GetInertialHeading(), 1000);
  Grab(70);
  DriveTo(-23, 2000);
  TurnMedium(-65, 1000);
  Grab(0);
  FireCatapult();
  DriveTo(13, 1500);
  thread PullCatapult2 = thread(PullCatapult);
  TurnMedium(-115, 1200);
  Grab(70);
  DriveTo(38, 3500, 9);

  //shoot the next three disk
  TurnMedium(95, 1000);
  FireCatapult();
  thread PullCatapult3 = thread(PullCatapult);
  thread grabcheck2 = thread(GrabCheckFast);

  //pick up low goal barrier discs and shoot
  TurnMedium(-80, 1000);
  ResetChassis();
  //CurveRight(24.4346, 0.1, 2000, false);
  if (121 < GetInertialHeading() || 119 > GetInertialHeading()) {
    TurnSmall(120-GetInertialHeading(), 800);
  }
  DriveTo(37, 2500, 13);
  ResetChassis();
  //CurveLeft(10, 0.1, 1000);
  DriveTo(-34, 3000);
  FireCatapult();

  
  //release auton tensioning
  DigitalOutC.set(true);
  
  double end_time = Brain.timer(msec);
  Brain.Screen.newLine();
  Brain.Screen.printAt(110, 110, "%f", end_time - begin_time);
}

void LeftAutonRoller() {
  double begin_time = Brain.timer(msec);
  thread PullCatapult1 = thread(PullCatapult);
  thread headingcorrection = thread(heading_correction);
  
  //pick up disc and spin the roller
  ResetChassis();
  isturning = true;
  Grab(-100);
  ChassisControl(20, 20);
  wait(200, msec);
  Grab(0);
  Stop(vex::hold);
  
  //shoot first 3 discs
  DriveToSeperate(-3 - ((GetLeftRotationDegree() / 360.0) * wheel_distance_in), -3 - ((GetRightRotationDegree() / 360.0) * wheel_distance_in), 1500);
  TurnBig(135, 1200);
  thread grabcheck = thread(GrabCheckFast);
  DriveTo(8, 1000);
  TurnMedium(45, 1000);
  DriveTo(-23, 2000);
  TurnMedium(-20, 1000);
  Grab(0);
  FireCatapult();
  DriveTo(13, 1500);
  thread PullCatapult2 = thread(PullCatapult);
  TurnMedium(-135, 1200);
  Grab(70);
  DriveTo(38, 3500, 9);

  //shoot the next three disk
  TurnMedium(-40, 1000);
  FireCatapult();
  thread PullCatapult3 = thread(PullCatapult);
  thread grabcheck2 = thread(GrabCheckFast);

  //pick up low goal barrier discs and shoot
  TurnMedium(-120, 1000);
  ResetChassis();
  //CurveRight(24.4346, 0.1, 2000, false);
  if (1 < GetInertialHeading() || -1 > GetInertialHeading()) {
    TurnSmall(0, 800);
  }
  DriveTo(37, 2500, 13);
  ResetChassis();
  //CurveLeft(10.5, 0.1, 900, false);
  if(GetInertialHeading() < -31.5 || GetInertialHeading() > -29.5) {
    TurnSmall(-30.5, 800);
  }
  DriveTo(-33, 3000);
  FireCatapult();

  
  //release auton tensioning
  DigitalOutC.set(true);
  
  double end_time = Brain.timer(msec);
  Brain.Screen.newLine();
  Brain.Screen.printAt(110, 110, "%f", end_time - begin_time);
}

void LeftAuton2() {
  double begin_time = Brain.timer(msec);
  thread PullCatapult1 = thread(PullCatapult);
  thread headingcorrection = thread(heading_correction);
  
  //pick up disc and spin the roller
  ResetChassis();
  isturning = true;
  Grab(-100);
  ChassisControl(25, 25);
  wait(170, msec);
  Grab(0);
  Stop(vex::hold);
  
  //shoot first 3 discs
  DriveToSeperate(-8 - ((GetLeftRotationDegree() / 360.0) * wheel_distance_in), -8 - ((GetRightRotationDegree() / 360.0) * wheel_distance_in), 1500);
  TurnSmall(56, 900);
  DriveTo(-30, 1600);

  // Back up a bit, turn and shoot the first disk.
  DriveTo(5, 1000);
  TurnMedium(-20, 1000);
  FireCatapult();
  wait(100, msec);
  
  // Lower the catapult, pick the 3rd disk.
  thread PullCatapult2 = thread(PullCatapult);
  TurnBig(-122, 1000);
  while(LimitSwitchA.pressing() == 0) {
    wait(10, msec);
  }
  Grab(100);
  DriveTo(20, 1200, 11);
  DriveTo(-5, 1000);
  TurnMedium(-32, 1000);
  Grab(0);
  FireCatapult();
  thread PullCatapult3 = thread(PullCatapult);
  TurnMedium(-150, 1000);
  Grab(100);
  DriveTo(8, 1200);
  //CurveCircle(150, 7, 1500, false);
  DriveTo(10, 1500);
  TurnSmall(-43, 900);
  DriveTo(-14, 1500);
  Grab(0);
  FireCatapult();
  /*
  thread PullCatapult4 = thread(PullCatapult);
  Grab(100);
  CurveCircle(39, 20, 2000);
  while(LimitSwitchA.pressing() == 0) {
    wait(10, msec);
  }
  DriveTo(20, 1500);
  CurveCircle(-34, -8, 1000);
  DriveTo(-25, 2500);
  FireCatapult();
  */

  
  //release auton tensioning
  DigitalOutC.set(true);
  
  double end_time = Brain.timer(msec);
  Brain.Screen.newLine();
  Brain.Screen.printAt(110, 110, "%f", end_time - begin_time);
}

void RightAuton() {
  ResetChassis();
  double begin_time = Brain.timer(msec);
  thread PullCatapult1 = thread(PullCatapult);
  thread headingcorrection = thread(heading_correction);

  //drive forward to pick up disc
  ResetChassis();
  isturning = true;
  //CurveLeft(30, 0.7, 2000, false);
  DriveTo(-3, 1000);
  TurnMedium(90, 1100);
  Grab(0);
  DriveTo(7.5, 1300);
  
  //spin the roller
  ResetChassis();
  isturning = true;
  Grab(-100);
  ChassisControl(20, 20);
  wait(200, msec);
  Grab(0);
  Stop(vex::hold);
  
  //shoot first 3 discs
  DriveToSeperate(-7 - ((GetLeftRotationDegree() / 360.0) * wheel_distance_in), -7 - ((GetRightRotationDegree() / 360.0) * wheel_distance_in), 0, 1500);
  thread grabcheck = thread(GrabCheckFast);
  TurnSmall(45, 1000);
  DriveTo(-22, 2000);
  TurnMedium(105, 1000);
  Grab(0);
  FireCatapult();
  DriveTo(6, 1500);
  thread PullCatapult2 = thread(PullCatapult);
  TurnMedium(-135, 1200);
  Grab(70);
  while (LimitSwitchA.pressing() == 0) {
    wait(10, msec);
  }
  DriveTo(51, 3500, 12);

  //shoot the next three disk
  TurnMedium(135, 1000);
  DriveTo(6, 1500);
  FireCatapult();
  thread PullCatapult3 = thread(PullCatapult);
  thread grabcheck2 = thread(GrabCheckFast);

  //pick up low goal barrier discs and shoot
  DriveTo(-3, 900);
  TurnSmall(90, 900);
  DriveTo(38, 5000, 7);
  ResetChassis();
  //CurveRight(10.5, 0.1, 900, false);
  if(GetInertialHeading() < 119.5 || GetInertialHeading() > 121.5) {
    TurnSmall(120.5, 800);
  }
  ResetChassis();
  DriveTo(-33, 3000);
  FireCatapult();

  
  //release auton tensioning
  DigitalOutC.set(true);
  
  double end_time = Brain.timer(msec);
  Brain.Screen.newLine();
  Brain.Screen.printAt(110, 110, "%f", end_time - begin_time);
}

void RightAuton2() {
  double begin_time = Brain.timer(msec);
  thread PullCatapult1 = thread(PullCatapult);
  thread headingcorrection = thread(heading_correction);
  
  //pick up disc and spin the roller
  //CurveCircle(180, 9, 1400);
  DriveTo(7, 1100);
  ResetChassis();
  isturning = true;
  Grab(-100);
  ChassisControl(25, 25);
  wait(200, msec);
  Grab(0);
  Stop(vex::hold);
  
  //shoot first 3 discs
  DriveToSeperate(-9 - ((GetLeftRotationDegree() / 360.0) * wheel_distance_in), -9 - ((GetRightRotationDegree() / 360.0) * wheel_distance_in), 1500);
  TurnSmall(135, 900);
  DriveTo(-23, 2000);

  // Back up a bit, turn and shoot the first disk.
  TurnMedium(198.5, 1000);
  FireCatapult();
  
  // Lower the catapult, pick the 3rd disk.
  thread PullCatapult2 = thread(PullCatapult);
  wait(200, msec);
  DriveTo(6.5, 1000);
  TurnMedium(315, 1000);
  while(LimitSwitchA.pressing() == 0) {
    wait(10, msec);
  }
  Grab(100);
  DriveTo(25, 2500, 11);
  TurnMedium(210, 1000);
  Grab(0);
  FireCatapult();
  thread PullCatapult3 = thread(PullCatapult);
  TurnMedium(315, 1000);
  Grab(100);
  DriveTo(20, 2000);
  DriveTo(-2, 600);
  TurnMedium(222, 1000);
  DriveTo(8, 1200);
  wait(200, msec);
  Grab(100);
  ChassisControl(-28, -28);
  wait(200, msec);
  FireCatapult();
  wait(1000, msec);
  ChassisControl(0, 0);
  Grab(0);
  
  //release auton tensioning
  DigitalOutC.set(true);
  
  double end_time = Brain.timer(msec);
  Brain.Screen.newLine();
  Brain.Screen.printAt(110, 110, "%f", end_time - begin_time);
}

void SoloAWP() {
  double begin_time = Brain.timer(msec);
  thread PullCatapult1 = thread(PullCatapult);
  thread headingcorrection = thread(heading_correction);
  
  //pick up disc and spin the roller
  ResetChassis();
  isturning = true;
  Grab(-100);
  ChassisControl(20, 20);
  wait(160, msec);
  Grab(0);
  Stop(vex::hold);
  
  //shoot first 3 discs
  DriveToSeperate(-3 - ((GetLeftRotationDegree() / 360.0) * wheel_distance_in), -3 - ((GetRightRotationDegree() / 360.0) * wheel_distance_in), 1000);
  TurnToAngle(145, 1200);
  thread grabcheck = thread(GrabCheckFast);
  DriveTo(8, 1300);
  DriveTo(-6, 1000);
  TurnToAngle(45, 800);
  DriveTo(-23, 2000);
  TurnToAngle(-20, 1000);
  Grab(0);
  FireCatapult();
  DriveTo(4, 2000);
  thread PullCatapult2 = thread(PullCatapult);
  TurnToAngle(-135, 1200);
  Grab(-50);
  DriveTo(10, 1500);
  Grab(100);
  DriveTo(25, 3000, 8);

  //shoot the next three disk
  TurnToAngle(-40, 1000);
  FireCatapult();
  thread PullCatapult3 = thread(PullCatapult);
  thread grabcheck2 = thread(GrabCheckFast);

  //pick up discs on other side of field and shoot
  TurnToAngle(-135, 1000);
  while(LimitSwitchA.pressing() == 0) {
    wait(10, msec);
  }
  Grab(100);
  DriveTo(50, 2000, 10);
  TurnToAngle(-72, 1000);
  FireCatapult();
  DriveTo(-4, 1000);
  Grab(0);
  TurnToAngle(-135, 1000);
  DriveTo(35, 3000);
  TurnToAngle(-90, 800);
  Grab(100);
  ChassisControl(20, 20);
  wait(300, msec);
  Grab(0);
  ChassisControl(0, 0);

  //release auton tensioning
  DigitalOutC.set(true);
  
  double end_time = Brain.timer(msec);
  Brain.Screen.newLine();
  Brain.Screen.printAt(110, 110, "%f", end_time - begin_time);
}

void SoloAWP2() {
  double begin_time = Brain.timer(msec);
  thread PullCatapult1 = thread(PullCatapult);
  thread headingcorrection = thread(heading_correction);
  
  //pick up disc and spin the roller
  ResetChassis();
  isturning = true;
  Grab(-100);
  ChassisControl(25, 25);
  wait(170, msec);
  Grab(0);
  Stop(vex::hold);
  
  //shoot first 3 discs
  DriveToSeperate(-8 - ((GetLeftRotationDegree() / 360.0) * wheel_distance_in), -8 - ((GetRightRotationDegree() / 360.0) * wheel_distance_in), 1500);
  TurnSmall(56, 1000);
  DriveTo(-30, 1600);

  // Back up a bit, turn and shoot the first disk.
  DriveTo(5.5, 1100);
  TurnMedium(-21, 1000);
  DriveTo(-6, 1000);
  FireCatapult();
  thread PullCatapult2 = thread(PullCatapult);
  wait(100, msec);
  DriveTo(6, 1000);
  
  // Lower the catapult, pick the 3rd disk.
  TurnBig(-122, 1000);
  while(LimitSwitchA.pressing() == 0) {
    wait(10, msec);
  }
  Grab(100);
  DriveTo(18, 2000, 10);
  TurnMedium(-32, 1000);
  Grab(0);
  FireCatapult();
  thread PullCatapult3 = thread(PullCatapult);
  wait(100, msec);
  DriveTo(-7, 1000);
  TurnMedium(-135, 1000);
  while(LimitSwitchA.pressing() == 0) {
    wait(10, msec);
  }
  Grab(100);
  ResetChassis();
  thread(grabwait) = thread(grab_wait);
  DriveTo(96, 2000);
  TurnSmall(-90, 1000);
  Grab(-100);
  ChassisControl(25, 25);
  wait(250, msec);
  Grab(0);
  Stop(vex::hold);
  
  //release auton tensioning
  DigitalOutC.set(true);
  
  double end_time = Brain.timer(msec);
  Brain.Screen.newLine();
  Brain.Screen.printAt(110, 110, "%f", end_time - begin_time);
}

void ProgSkills() {
  ResetChassis();
  double begin_time = Brain.timer(msec);

  thread PullCatapult1 = thread(PullCatapult);
  //thread anglestabilization = thread(AngleStabilization);
  
  //Spin the first rollers
  
  ChassisControl(25, 25);
  Grab(-80);
  wait(300, msec);
  Grab(0);
  Stop(vex::hold);
  DriveToSeperate(-2.5 - ((GetLeftRotationDegree() / 360.0) * wheel_distance_in), -2.5 - ((GetRightRotationDegree() / 360.0) * wheel_distance_in), 1500);

  // Pick up the preload disk
  do {
    wait(10, msec);
  } while(LimitSwitchA.pressing() == 0);
  
  Grab(75);
  // Turn to pick up the 3rd disk
  TurnBig(135, 1200);
  DriveTo(25, 2500, 15);
  TurnSmall(90, 1000);
  
  DriveTo(5, 1000);
  Grab(0);
  
  // spin the second roller
  ChassisControl(25, 25);
  Grab(-85);
  wait(250, msec);
  Grab(0);
  Stop(vex::hold);
  DriveToSeperate(-5 - ((GetLeftRotationDegree() / 360.0) * wheel_distance_in), -5 - ((GetRightRotationDegree() / 360.0) * wheel_distance_in), 1500);
  // Backup a bit, drive forward to shoot first 3 disks.
  TurnMedium(0, 1000);
  Grab(100);
  DriveTo(-46.5, 3000);
  FireCatapult();
  Grab(0);
  
  
  // Pick up the second batch.
  thread PullCatapult2 = thread(PullCatapult);
  TurnSmall(130.8, 1000);
  do {
    wait(10, msec);
  } while(LimitSwitchA.pressing() == 0);
  Grab(100);
  DriveTo(23, 4000, 13);
  TurnMedium(45, 1000);
  DriveTo(34, 4000, 10);
  TurnMedium(135, 1000);
  DriveTo(-8, 1000);
  FireCatapult();
  Grab(0);


  // Pick up the third batch
  thread PullCatapult3 = thread(PullCatapult);
  wait(100, msec);
  DriveTo(8, 1000);
  TurnMedium(-135, 1000);
  DriveTo(28, 3000);
  Grab(100);
  DriveTo(43, 6000, 10);
  DriveToSeperate(-4 - ((GetLeftRotationDegree() / 360.0) * wheel_distance_in), -4 - ((GetRightRotationDegree() / 360.0) * wheel_distance_in), 1500);
  TurnMedium(-90, 1000);
  DriveTo(-47, 3500);
  FireCatapult();
  
  // Pick up the forth batch
  thread PullCatapult4 = thread(PullCatapult);
  wait(100, msec);
  TurnSmall(-47, 900);
  DriveTo(38, 0, 3000);
  TurnSmall(-100, 1000);
  Grab(100);
  DriveTo(40, 12, 5000);
  TurnSmall(-90, 800);
  Grab(0);
  DriveTo(6, 0, 1000);
  ChassisControl(25, 25);
  Grab(100);
  wait(300, msec);
  Grab(0);
  Stop(vex::hold);
  DriveToSeperate(-5 - ((GetLeftRotationDegree() / 360.0) * wheel_distance_in), -5 - ((GetRightRotationDegree() / 360.0) * wheel_distance_in), 1500);
  // Backup a bit, drive forward to shoot first 3 disks.
  TurnMedium(-180, 1000);
  Grab(100);
  DriveTo(-46.5, 3000);
  FireCatapult();
  Grab(0);
  
  
  // Pick up the second batch.
  thread PullCatapult5 = thread(PullCatapult);
  TurnSmall(-229.2, 1100);
  do {
    wait(10, msec);
  } while(LimitSwitchA.pressing() == 0);
  Grab(100);
  DriveTo(23, 4000, 13);
  TurnMedium(-315, 1000);
  DriveTo(34, 4000, 10);
  TurnMedium(-225, 1000);
  DriveTo(-8, 1000);
  FireCatapult();
  Grab(0);


  // Pick up the third batch
  thread PullCatapult6 = thread(PullCatapult);
  wait(100, msec);
  DriveTo(8, 1000);
  TurnMedium(-225, 1000);
  DriveTo(28, 2500);
  Grab(100);
  DriveTo(42, 6000, 10);
  TurnSmall(90, 1000);
  DriveTo(-47, 3500);
  FireCatapult();
  DriveTo(65, 2500);
  TurnSmall(45, 800);
  DigitalOutD.set(true);
  DigitalOutE.set(true);
  double end_time = Brain.timer(msec);
  Brain.Screen.newLine();
  Brain.Screen.print(end_time - begin_time);
}

/*
void ProgSkillsLowGoal() {
  InertialA.setHeading(0, degrees);
  InertialA.setRotation(0, degrees);
  ResetChassis();
  double begin_time = Brain.timer(msec);
  thread PullCatapult1 = thread(PullCatapult);
  //thread anglestabilization = thread(AngleStabilization);
  
  //Spin the first rollers
  
  ChassisControl(25, 25);
  Grab(100);
  wait(300, msec);
  Grab(0);
  Stop(vex::hold);
  DriveToSeperate(-2.5 - ((GetLeftRotationDegree() / 360.0) * wheel_distance_in), -2.5 - ((GetRightRotationDegree() / 360.0) * wheel_distance_in), 1500);

  // Pick up the preload disk
  while (LimitSwitchA.pressing() == 0) {
    wait(10, msec);
  }
  
  Grab(100);
  // Turn to pick up the 3rd disk
  TurnBig(135-GetInertialHeading(), 1200);
  DriveTo(23, 3000, 15);
  TurnSmall(-45, 1000);
  Grab(0);
  wait(500, msec);
  InertialA.setHeading(0, degrees);
  InertialA.setRotation(0, degrees);
  DriveTo(8, 1000);
  
  // spin the second roller
  ResetChassis();
  ChassisControl(25, 25);
  Grab(100);
  wait(300, msec);
  Grab(0);
  Stop(vex::hold);
  DriveToSeperate(-6 - ((GetLeftRotationDegree() / 360.0) * wheel_distance_in), -6 - ((GetRightRotationDegree() / 360.0) * wheel_distance_in), 1500);
  // Backup a bit, drive forward to shoot first 3 disks.
  TurnMedium(-90-GetInertialHeading(), 1000);
  Grab(100);
  DriveTo(-56, 3000);
  TurnSmall(6, 800);
  FireCatapult();
  thread PullCatapult7 = thread(PullCatapult);
  TurnMedium(-96, 1000);
  while(LimitSwitchA.pressing() == 0) {
    wait(10, msec);
  }
  DriveTo(27, 4000, 7);
  DriveTo(-27, 2200);
  TurnMedium(96, 1000);
   
  // Pick up the second batch.
  thread PullCatapult2 = thread(PullCatapult);
  TurnSmall(-43, 1100);
  while(LimitSwitchA.pressing() == 0) {
    wait(10, msec);
  }
  Grab(100);
  DriveTo(17, 2000);
  DriveTo(15, 2000, 10);
  TurnMedium(-95, 1000);
  DriveTo(32, 4000, 12);
  TurnMedium(90, 1000);
  DriveTo(-8, 1100);
  FireCatapult();
  Grab(0);

  // Pick up the third batch
  wait(100, msec);
  thread PullCatapult3 = thread(PullCatapult);
  DriveTo(4, 1000);
  TurnBig(-135, 1100);
  Grab(100);
  DriveTo(40, 5000, 8);
  TurnMedium(84, 1000);
  TurnSmall(43, 900);
  Grab(100);
  DriveTo(17, 2000);
  DriveTo(25, 4000, 8);
  DriveTo(-42, 3000);
  TurnSmall(-43, 900);
  FireCatapult();
  thread pullcatapult8 = thread(PullCatapult);
  Grab(0);
  TurnSmall(30, 800);
  DriveTo(25, 2500);
  TurnSmall(-34, 800);
  Grab(100);
  DriveTo(20, 3000, 10);
  TurnSmall(10, 800);
  Grab(0);
  InertialA.setHeading(0, degrees);
  InertialA.setRotation(0, degrees);
  ChassisControl(25, 25);
  Grab(100);
  wait(300, msec);
  Grab(0);
  Stop(vex::hold);
  DriveToSeperate(-15 - ((GetLeftRotationDegree() / 360.0) * wheel_distance_in), -15 - ((GetRightRotationDegree() / 360.0) * wheel_distance_in), 1500);
  
  TurnMedium(90-GetInertialHeading(), 1000);
  InertialA.setHeading(0, degrees);
  InertialA.setRotation(0, degrees);
  DriveTo(17, 1000);
  
  // spin the second roller
  ChassisControl(25, 25);
  Grab(100);
  wait(300, msec);
  Grab(0);
  Stop(vex::hold);
  DriveToSeperate(-4 - ((GetLeftRotationDegree() / 360.0) * wheel_distance_in), -4 - ((GetRightRotationDegree() / 360.0) * wheel_distance_in), 0, 1500);
  // Backup a bit, drive forward to shoot first 3 disks.
  TurnMedium(-30-GetInertialHeading(), 1000);
  Grab(100);
  DriveTo(-70, 3000);
  TurnSmall(36, 800);
  FireCatapult();
  TurnMedium(-96, 1000);
  DriveTo(24, 3000, 10);
  DriveTo(-24, 2200);
  TurnMedium(96, 1000);
   
  // Pick up the second batch.
  thread PullCatapult4 = thread(PullCatapult);
  TurnSmall(-43, 1100);
  do {
    wait(10, msec);
  } while(LimitSwitchA.pressing() == 0);
  Grab(100);
  DriveTo(17, 2000);
  DriveTo(15, 2000, 10);
  TurnMedium(-95, 1000);
  DriveTo(32, 4000, 12);
  TurnMedium(90, 1000);
  DriveTo(-8, 1000);
  FireCatapult();
  Grab(0);

  // Pick up the third batch
  wait(100, msec);
  thread PullCatapult5 = thread(PullCatapult);
  DriveTo(4, 1000);
  TurnBig(-135, 1100);
  Grab(100);
  DriveTo(30, 2500, 10);
  TurnMedium(84, 1000);
  TurnSmall(43, 900);
  Grab(100);
  DriveTo(17, 2000);
  DriveTo(25, 4000, 8);
  DriveTo(-42, 3000);
  TurnSmall(-43, 900);
  FireCatapult();
  TurnSmall(10, 1000);
  DriveTo(50, 3500);
  TurnSmall(-49, 1000);
  //DigitalOutD.set(true);
  //DigitalOutE.set(true);

  double end_time = Brain.timer(msec);
  Brain.Screen.newLine();
  Brain.Screen.print(end_time - begin_time);
}
*/

void TestPID() {
  TurnToAngle(1, 1000);
  DriveToNew(30, 3000);
  DriveToNew(-30, 3000);
  DriveToNew(24, 3000);
  DriveToNew(-24, 3000);
  DriveToNew(10, 3000);
  DriveToNew(-10, 3000);
  DriveToNew(24, 3000);
  DriveToNew(-24, 3000);
}

void SoloAWP11() {
  double begin_time = Brain.timer(msec);
  thread pullsling = thread(PullCatapult);
  thread headingcorrection = thread(heading_correction);
  
  //pick up disc and spin the roller
  Grab(100);
  ChassisControl(30, 30);
  wait(200, msec);
  Grab(0);
  Stop(vex::hold);
  DriveToNew(-6, 1500);
  TurnToAngle(-5, 1000);
  FireCatapult();
  thread pullsling2 = thread(PullCatapult);
  TurnToAngle(-135, 1000);
  Grab(100);
  DriveToNew(30, 3000);
}

void Left8() {
  double begin_time = Brain.timer(msec);
  thread pullsling = thread(PullCatapult);
  thread headingcorrection = thread(heading_correction);
  
  //pick up disc and spin the roller
  DriveToNew(2, 700);
  Grab(-100);
  wait(100, msec);
  Grab(0);
  DriveTo(-5, 1000);
  TurnToAngle(-11, 1000);
  while(LimitSwitchA.pressing() == 0) {
    wait(10, msec);
  }
  FireCatapult();
  thread pullsling2 = thread(PullCatapult);
  TurnToAngle(-128, 1000);
  Grab(100);
  DriveToNew(46, 3000, 4);
  TurnToAngle(-33, 1000);
  DriveTo(-4, 1000);
  FireCatapult();
  thread pullsling3 = thread(PullCatapult);
  TurnToAngle(-90, 1000);
  CurveCircle(90, 13, 2000);
  DriveToNew(20, 2000, 4);
  CurveCircle(-35, -9, 1000);
  DriveTo(-38, 3000);
  FireCatapult();
}