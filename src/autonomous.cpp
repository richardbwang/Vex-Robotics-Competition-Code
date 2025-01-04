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
  Brain.Screen.printAt(1, 215, "Auton Selected =  Left 8");
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
      Brain.Screen.printAt(1, 215, "Auton Selected =  Left 8");
    break;

    case 2:
      Brain.Screen.printAt(1, 215, "Auton Selected =  Right 8");
    break;

    case 3:
      Brain.Screen.printAt(1, 215, "Auton Selected =  SoloAWP 8");
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
      Brain.Screen.printAt(1, 215, "Auton Selected =  Left 8");
    break;

    case 2:
      Brain.Screen.printAt(1, 215, "Auton Selected =  Right 8");
    break;

    case 3:
      Brain.Screen.printAt(1, 215, "Auton Selected =  SoloAWP 8");
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

void LeftAuton12() {
  double begin_time = Brain.timer(msec);
  thread pullsling = thread(PullSling);
  thread headingcorrection = thread(heading_correction);
  
  //pick up disc and spin the roller
  Grab(100);
  DriveTo(4, 800);
  thread grabwait = thread(grab_wait);
  CurveCircle(20, -19, 1000);
  Grab(-100);
  wait(250, msec);
  Grab(0);
  DriveTo(3, 800);
  Grab(100);
  TurnToAngle(90, 1000);
  DigitalOutB.set(true);
  CurveCircle(34, -11, 1100);
  FireSling();
  DigitalOutB.set(false);
  thread pullsling2 = thread(PullSling);
  Grab(100);
  wait(2000, msec);
  Grab(0);
  FireSling();
  thread pullsling3 = thread(PullSling);
  DigitalOutB.set(true);
  DriveTo(-3, 800);
  TurnToAngle(104, 1000);
  Grab(100);
  DriveTo(17, 2000);
  DigitalOutB.set(false);
  DriveTo(18, 2000);
  TurnToAngle(13, 1000);
  FireSling();
  thread pullsling4 = thread(PullSling);
  DriveTo(-29.5, 2500);
  TurnToAngle(45, 1000);
  Grab(100);
  DriveTo(32, 4000, 8);
  TurnToAngle(4, 900);
  FireSling();
  thread pullsling5 = thread(PullSling);

  //release auton tensioning
  //DigitalOutC.set(true);
  
  double end_time = Brain.timer(msec);
  Brain.Screen.newLine();
  Brain.Screen.printAt(110, 110, "%f", end_time - begin_time);
}

void LeftAuton9() {
  double begin_time = Brain.timer(msec);
  thread pullsling = thread(PullSling);
  thread headingcorrection = thread(heading_correction);
  Grab(100);
  CurveCircle(-30, -30, 2000);
  DriveTo(-2, 800);
  TurnToAngle(-12, 1000);
  FireSling();
  thread pullsling2 = thread(PullSling);
  TurnToAngle(0, 1000);
  DriveTo(-16, 2000);
  Grab(-100);
  wait(500, msec);
  Grab(0);
  DriveTo(3, 1000);
  TurnToAngle(45, 1000);
  Grab(100);  
  DriveTo(20, 3000);
  DriveTo(16, 3000, 4);
  TurnToAngle(-33, 1000);
  FireSling();
  thread pullsling3 = thread(PullSling);
  DriveTo(-33, 3000);
  TurnToAngle(0, 1000);
  Grab(100);
  DriveTo(30, 5000, 4);
  TurnToAngle(-41, 1000);
  FireSling();
  thread pullsling4 = thread(PullSling);

  double end_time = Brain.timer(msec);
  Brain.Screen.newLine();
  Brain.Screen.printAt(110, 110, "%f", end_time - begin_time);
}

void LeftAuton8() {
  double begin_time = Brain.timer(msec);
  thread pullsling = thread(PullSling);
  thread headingcorrection = thread(heading_correction);
  DriveTo(-3, 800);
  Grab(-100);
  wait(400, msec);
  Grab(0);
  DriveTo(2, 1000);
  TurnToAngle(-14, 1000);
  FireSling();
  thread pullsling2 = thread(PullSling);
  TurnToAngle(35, 1000);
  Grab(100);
  DriveTo(20, 2500);
  TurnToAngle(45, 1000);
  DriveTo(16, 2000, 5);
  TurnToAngle(-33, 1000);
  DriveTo(-5, 1000);
  FireSling();
  thread pullsling3 = thread(PullSling);
  DriveTo(-28, 2000);
  TurnToAngle(0, 1000);
  Grab(100);
  DriveTo(32, 5000, 3);
  DriveTo(-32, 2500);
  TurnToAngle(-33, 1000);
  DriveTo(28, 2000);
  FireSling();
  thread pullsling4 = thread(PullSling);
  DigitalOutC.set(true);
}

void RightAuton8() {
  double begin_time = Brain.timer(msec);
  thread pullsling = thread(PullSling);
  thread headingcorrection = thread(heading_correction);
  Swing(191, -1, 1000);
  DriveTo(-6, 1000);
  Grab(-100);
  wait(400, msec);
  Grab(0);
  DriveTo(3, 1000);
  FireSling();
  thread pullsling2 = thread(PullSling);
  TurnToAngle(132, 1000);
  Grab(100);
  DriveTo(57, 3000, 6);
  DriveTo(-16, 2000);
  TurnToAngle(214, 1000);
  FireSling();
  thread pullsling3 = thread(PullSling);  
  DriveTo(-32, 3000);
  TurnToAngle(180, 1000);
  Grab(100);
  DriveTo(32, 5000, 3);
  DriveTo(-32, 2500);
  TurnToAngle(212.5, 1000);
  DriveTo(29, 2500);
  FireSling();
  thread pullsling4 = thread(PullSling);
  DigitalOutC.set(true);
}

void RightAuton9() {
  double begin_time = Brain.timer(msec);
  thread pullsling = thread(PullSling);
  thread headingcorrection = thread(heading_correction);
  Grab(100);
  DriveTo(20, 2000);
  DriveTo(-2, 1000);
  TurnToAngle(-30, 1000);
  FireSling();
  thread pullsling2 = thread(PullSling);
  TurnToAngle(-59, 1000);
  DriveTo(-18.5, 1500);
  Grab(-100);
  wait(250, msec);
  Grab(0);
  DriveTo(5.5, 1000);
  TurnToAngle(-90, 1000);
  Grab(100);
  DriveTo(52, 4000, 8);
  DriveTo(-16, 2000);
  TurnToAngle(-16, 1000);
  FireSling();
  thread pullsling3 = thread(PullSling);
  DriveTo(-34.5, 3000);
  TurnToAngle(-45, 1000);
  Grab(100);
  DriveTo(34, 5000, 3);
  TurnToAngle(0, 1000);
  FireSling();
  thread pullsling4 = thread(PullSling);
}

void SoloAWP11() {
  double begin_time = Brain.timer(msec);
  thread pullsling = thread(PullSling);
  thread headingcorrection = thread(heading_correction);
  
  //pick up disc and spin the roller
  Grab(100);
  DriveTo(4, 1000);
  Grab(0);
  DriveTo(-8, 1000);
  Grab(-100);
  wait(200, msec);
  Grab(0);
  DriveTo(5, 1000);
  TurnToAngle(35, 1000);

  //FireSling();
  //thread pullsling2 = thread(PullSling);
}

void SoloAWP8() {
  double begin_time = Brain.timer(msec);
  thread pullsling = thread(PullSling);
  thread headingcorrection = thread(heading_correction);
  DriveTo(-3, 800);
  Grab(-100);
  wait(500, msec);
  Grab(0);
  DriveTo(2, 1000);
  TurnToAngle(-13, 1000);
  FireSling();
  thread pullsling2 = thread(PullSling);
  TurnToAngle(35, 1000);
  Grab(100);
  DriveTo(20, 2500);
  TurnToAngle(45, 1000);
  DriveTo(15, 2000, 5);
  TurnToAngle(-31, 1000);
  DriveTo(-5, 1000);
  FireSling();
  thread pullsling3 = thread(PullSling);
  TurnToAngle(40, 1000);
  Grab(100);
  DriveTo(56, 5000, 6);
  TurnToAngle(-68.5, 1000);
  FireSling();
  thread pullsling4 = thread(PullSling);
  DriveTo(5, 1000);
  TurnToAngle(-135, 1000);
  DriveTo(-35, 2000);
  ChassisControl(-15, -15);
  Grab(-100);
  wait(500, msec);
  Grab(0);
  double end_time = Brain.timer(msec);
  Brain.Screen.newLine();
  Brain.Screen.printAt(110, 110, "%f", end_time - begin_time);
  DigitalOutC.set(true);
}

void ProgSkills() {
  double begin_time = Brain.timer(msec);
  thread pullsling = thread(PullSling);
  thread headingcorrection = thread(heading_correction);
  DriveTo(-3, 800);
  ChassisControl(-20, -20);
  Grab(-100);
  wait(800, msec);
  Grab(0);
  Stop(vex::hold);
  Swing(-90, 1, 1000);
  Grab(100);
  DriveTo(21, 2500);
  Grab(0);
  DriveTo(4, 1000);
  ChassisControl(20, 20);
  Grab(100);
  wait(800, msec);
  Grab(0);
  Stop(vex::hold);
  DriveTo(-4, 1000);
  TurnToAngle(0, 1000);
  DriveTo(50, 4000);
  FireSling();
  thread pullsling2 = thread(PullSling);
  TurnToAngle(137, 1000);
  Grab(100);
  DriveTo(21, 2500);
  TurnToAngle(45, 1000);
  DriveTo(31, 4000, 7);
  TurnToAngle(-45, 1000);
  DriveTo(4.5, 1000);
  FireSling();
  thread pullsling3 = thread(PullSling);
  TurnToAngle(0, 1000);
  Grab(100);
  DriveTo(38, 5000, 5);
  TurnToAngle(-98, 1000);
  FireSling();
  thread pullsling4 = thread(PullSling);
  TurnToAngle(-233, 1000);
  Grab(100);
  DriveTo(28, 3000);
  DriveTo(15, 4000, 3);
  DriveTo(-43, 3000);
  TurnToAngle(-98, 1000);
  FireSling();
  thread pullsling5 = thread(PullSling);
  TurnToAngle(-248, 1000);
  Grab(100);
  DriveTo(48, 3500);
  DriveTo(15, 4000, 3);
  DriveTo(-15, 2000);
  TurnToAngle(-270, 1000);
  Grab(0);
  DriveTo(29, 2500);
  ChassisControl(20, 20);
  Grab(100);
  wait(800, msec);
  Grab(0);
  Stop(vex::hold);
  DriveTo(-22, 2500);
  TurnToAngle(-360, 1000);
  DriveTo(24, 2500);
  ChassisControl(20, 20);
  Grab(100);
  wait(800, msec);
  Grab(0);
  Stop(vex::hold);
  DriveTo(-3, 1000);
  TurnToAngle(-379, 1000);
  DriveTo(-61, 4000);
  TurnToAngle(-540, 1000);
  FireSling();
  thread pullsling6 = thread(PullSling);
  TurnToAngle(-403, 1000);
  Grab(100);
  DriveTo(21, 2500);
  TurnToAngle(-495, 1000);
  DriveTo(31, 4000, 7);
  TurnToAngle(-585, 1000);
  DriveTo(4.5, 1000);
  FireSling();
  thread pullsling7 = thread(PullSling);
  TurnToAngle(-540, 1000);
  Grab(100);
  DriveTo(38, 5000, 5);
  TurnToAngle(-638, 1000);
  FireSling();
  thread pullsling8 = thread(PullSling);
  TurnToAngle(-773, 1000);
  Grab(100);
  DriveTo(28, 3000);
  DriveTo(15, 4000, 3);
  DriveTo(-43, 3000);
  TurnToAngle(-638, 1000);
  FireSling();
  thread pullsling9 = thread(PullSling);
  TurnToAngle(-630, 1000);
  DriveTo(-56, 4000);
  TurnToAngle(-675, 1000);
  DigitalOutD.set(true);
  double end_time = Brain.timer(msec);
  Brain.Screen.newLine();
  Brain.Screen.printAt(110, 110, "%f", end_time - begin_time);
}

void TestPID() {
  CurveCircle(180, -5, 2000);
}