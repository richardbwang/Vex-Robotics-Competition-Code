#include "vex.h"
#include "utils.h"
#include "pid.h"
#include <ctime>
#include <cmath>

#include "autonomous.h"
#include "motor-control.h"

int AutonSelected = 1;
int AutonMin = 1;
int AutonMax = 3;

void drawGUI() {
  // Draws 2 buttons to be used for selecting auto
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(1, 40, "Select Auton then Press Go");
  Brain.Screen.printAt(1, 180, "Auton Selected =  %d   ", AutonSelected);
  Brain.Screen.printAt(1, 215, "Auton Selected =  Far");
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
      Brain.Screen.printAt(1, 215, "Auton Selected =  Far");
    break;

    case 2:
      Brain.Screen.printAt(1, 215, "Auton Selected =  Far3");
    break;

    case 3:
      Brain.Screen.printAt(1, 215, "Auton Selected =  Near");
    break;

    case 4:
      Brain.Screen.printAt(1, 215, "Auton Selected =  SoloAWP");
    break;

    case 5:
      Brain.Screen.printAt(1, 215, "Auton Selected =  TestPID");
    break;
  }
  }
  if (x >= 170 && x <= 270 && y >= 50 && y <= 150) {
    selectingAuton = false; // GO button pressed
    Brain.Screen.printAt(1, 180, "Auton  =  %d   GO           ", AutonSelected);

    switch(AutonSelected) {
    case 1:
      Brain.Screen.printAt(1, 215, "Auton Selected =  Far");
    break;

    case 2:
      Brain.Screen.printAt(1, 215, "Auton Selected =  Far3");
    break;

    case 3:
      Brain.Screen.printAt(1, 215, "Auton Selected =  Near");
    break;

    case 4:
      Brain.Screen.printAt(1, 215, "Auton Selected =  SoloAWP");
    break;

    case 5:
      Brain.Screen.printAt(1, 215, "Auton Selected =  TestPID");
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

void FarBar() {
  thread headingcorrection = thread(heading_correction);
  double begin_time = Brain.timer(msec);
  TurnToAngle(-25, 500);
  CurveCircle(-90, -35, 1500);
  Grab(-100);
  DriveTo(24, 1400);
  DriveTo(-17, 1400);
  Grab(0);
  TurnToAngle(-163, 1000);
  Grab(100);
  DriveTo(62, 2400);
  Swing(-180, 1, 400);
  TurnToAngle(-218, 800);
  DriveTo(-27, 1500);
  TurnToAngle(-360, 900);
  Grab(-100);
  wait(300, msec);
  DriveTo(27, 1400);
  DriveTo(-10, 1200);
  TurnToAngle(-405, 800);
  DriveTo(-45, 2000);
  Arm(100);
  double end_time = Brain.timer(msec);
  Brain.Screen.newLine();
  Brain.Screen.printAt(80, 80, "%f", end_time - begin_time);
}

void Far3() {
  thread headingcorrection = thread(heading_correction);
  double begin_time = Brain.timer(msec);
  TurnToAngle(-25, 500);
  CurveCircle(-90, -35, 1500);
  Grab(-100);
  DriveTo(24, 1400);
  DriveTo(-17, 1400);
  Grab(0);
  TurnToAngle(-163, 1200);
  Grab(100);
  DriveTo(63, 2400);
  Swing(-180, 1, 400);
  TurnToAngle(-210, 800);
  DriveTo(-30, 1800);
  TurnToAngle(-360, 1000);
  Grab(-100);
  DriveTo(20, 1600);
  DriveTo(-12, 1200);
  Grab(0);
  Swing(-450, -1, 1000);
  Grab(100);
  DriveTo(15, 1700, 6);
  TurnToAngle(-340, 1000);
  Grab(-100);
  CurveCircle(-370, -100, 3000, 8);
  double end_time = Brain.timer(msec);
  Brain.Screen.newLine();
  Brain.Screen.printAt(80, 80, "%f", end_time - begin_time);
}

void Far5() {
  thread headingcorrection = thread(heading_correction);
  double begin_time = Brain.timer(msec);
  Grab(100);
  DriveTo(5, 1000);
  CurveCircle(-10, 120, 1500, false);
  CurveCircle(-80, 30, 1500, false);
  CurveCircle(-90, 50, 1500, false);
  CurveCircle(-85, 100, 1000);
  TurnToAngle(90, 800);
  Grab(-100);
  wait(200, msec);
  DriveTo(18, 1200);
  DriveTo(-20, 1200);
  TurnToAngle(30, 800);
  CurveCircle(35, 300, 1500, false);
  Grab(100);
  CurveCircle(60, 150, 1500, false);
  Swing(0, 1, 800);
  DriveTo(-30, 2000);
  TurnToAngle(180, 800);
  Grab(-100);
  wait(200, msec);
  CurveCircle(100, 15, 2000);
  TurnToAngle(0, 800);
  Grab(100);
  DriveTo(15, 1500);
  CurveCircle(-70, 15, 1000);
  TurnToAngle(-180, 800);
  Grab(-100);
  DriveTo(20, 1500);
  double end_time = Brain.timer(msec);
  Brain.Screen.newLine();
  Brain.Screen.printAt(80, 80, "%f", end_time - begin_time);
}

void Near() {
  thread headingcorrection = thread(heading_correction);
  double begin_time = Brain.timer(msec);
  TurnToAngle(26, 500);
  CurveCircle(90, 22, 2000);
  Grab(-100);
  DriveTo(4, 1000);
  CurveCircle(66, 25, 1000);
  Swing(135, -1, 1000);
  //DriveTo(-1, 500);
  Arm(100);
  wait(500, msec);
  Arm(30);
  wait(500, msec);
  Arm(0);
  /*
  ChassisControl(-4, -4);
  wait(500, msec);
  ChassisControl(0, 0);
  */
  DriveTo(17, 1400);
  Grab(100);
  Arm(-100);
  CurveCircle(88, -80, 3000, 9);
  Arm(0);
  CurveCircle(215, -32, 3000);
  Arm(100);
  CurveCircle(177, -105, 3000);
  Arm(-100);
  Grab(-100);
  double end_time = Brain.timer(msec);
  Brain.Screen.newLine();
  Brain.Screen.printAt(80, 80, "%f", end_time - begin_time);
}

void NearElim() {
  thread headingcorrection = thread(heading_correction);
  double begin_time = Brain.timer(msec);
  DriveTo(52, 2500);
  TurnToAngle(-90, 1000);
  Grab(-100);
  DriveTo(3, 1000);
  DriveTo(-5, 1000);
  Swing(0, -1, 1000);
  Grab(100);
  DriveTo(17, 1200);
  CurveCircle(135, -32, 3000);
  CurveCircle(90, -85, 3000);
  DriveTo(7, 1000);
  Grab(-100);
  wait(300, msec);
  TurnToAngle(90, 400);
  DriveTo(-41, 2000);
  Arm(100);
  wait(500, msec);
  Arm(0);
  double end_time = Brain.timer(msec);
  Brain.Screen.newLine();
  Brain.Screen.printAt(80, 80, "%f", end_time - begin_time);
}

void SoloAWP() {
  thread headingcorrection = thread(heading_correction);
  thread pull_catapult = thread(pullcatapult);
  double begin_time = Brain.timer(msec);
  TurnToAngle(-25, 500);
  CurveCircle(-90, -30, 1500);
  Grab(-100);
  DriveTo(22, 1300);
  DriveTo(-18, 1400);
  TurnToAngle(-220, 1000);
  Grab(100);
  CurveCircle(-180, 60, 3000);
  DriveTo(74, 3000);
  TurnToAngle(-350, 900);
  ChassisControl(6, 6);
  Grab(-100);
  //catapult_motor.stop(coast);
  wait(300, msec);
  DriveTo(-10, 900);
  Swing(-405, -1, 800);
  Arm(100);
  wait(500, msec);
  Arm(30);
  wait(250, msec);
  ChassisControl(-4, -4);
  wait(250, msec);
  Arm(0);
  wait(250, msec);
  ChassisControl(0, 0);
  DriveTo(17, 1400);
  Arm(-100);
  TurnToAngle(-320, 800);
  CurveCircle(-371, -90, 3000);
  double end_time = Brain.timer(msec);
  Brain.Screen.newLine();
  Brain.Screen.printAt(80, 80, "%f", end_time - begin_time);
}

void ProgSkills(){
  thread headingcorrection = thread(heading_correction);
  double begin_time = Brain.timer(msec);
  DriveTo(10, 1000);
  Arm(100);
  wait(500, msec);
  Arm(0);
  launch = true;
  catapultlaunch(1000);
  double end_time = Brain.timer(msec);
  Brain.Screen.newLine();
  Brain.Screen.printAt(80, 80, "%f", end_time - begin_time);
}

void TestPID() {
  thread pull_catapult = thread(pullcatapult);
  CurveCircle(10, 40, 3000, false);
  CurveCircle(120, 13, 1500, false);
  CurveCircle(38, -80, 3000, false);
  CurveCircle(170, 13, 3000, false);
  CurveCircle(190, 165, 3000, false);
  CurveCircle(300, 13, 1500, false);
  CurveCircle(218, -80, 3000, false);
  CurveCircle(350, 13, 3000, false);
  CurveCircle(360, 300, 3000);
}