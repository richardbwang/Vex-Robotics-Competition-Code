#include "vex.h"
#include "utils.h"
#include "pid.h"
#include <ctime>
#include <cmath>
#include <thread>

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

void Far2Bar() {
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

void Far4Bar() {
    thread headingcorrection = thread(heading_correction);
  double begin_time = Brain.timer(msec);
  Grab(100);
  DriveTo(7, 1000);
  CurveCircle(-10, 130, 1500, false);
  CurveCircle(-80, 28, 1500, false);
  CurveCircle(-90, 50, 1500, false);
  CurveCircle(-85, 100, 1000);
  TurnToAngle(90, 800);
  Grab(-100);
  wait(200, msec);
  DriveTo(20, 1000);
  DriveTo(-20, 1000);
  TurnToAngle(30, 800);
  CurveCircle(35, 300, 1500, false);
  Grab(100);
  CurveCircle(53, 100, 1500, false);
  Swing(0, 1, 1000);
  DriveTo(-30, 1800);
  TurnToAngle(-180, 800);
  Grab(-100);
  wait(300, msec);
  DriveTo(15, 1000);
  Swing(-230, -1, 800);
  Arm(20);
  DriveTo(-54, 2000);
  Arm(100);
  double end_time = Brain.timer(msec);
  Brain.Screen.newLine();
  Brain.Screen.printAt(80, 80, "%f", end_time - begin_time);
}

void Far6LowAntiDisruption() {
  Arm(100);
  Grab(100);
  ChassisControl(6, 12);
  wait(200, msec);
  MoveToPoint(-11, 48, 1, 2000, false);
  Arm(-100);
  MoveToPoint(-4, -1, -1, 2000, false);
  TurnToAngle(50, 300);
  Grab(-100);
  wait(150, msec);
  TurnToPoint(-36, 0, 1, 300);
  Grab(100);
  boomerang(-36, 0, -90, 0.3, 2000, 1, false);
  MoveToPoint(-13, 2, -1, 2000, false);
  Arm(100);
  TurnToAngle(60, 400);
  DriveTo(6, 1000, false);
  Swing(45, 1, 500, false);
  while(InertialA.rotation() > -5) {
    ChassisControl(-4, 12);
    wait(10, msec);
  }
  //Swing(-10, 1, 600, false);
  //TurnToAngle(60, 300);
  //Swing(50, -1, 600, false);
  while(InertialA.rotation() < 50) {
    ChassisControl(6, -12);
    wait(10, msec);
  }
  Arm(-100);
  awp_motor.spin(fwd, -12, voltageUnits::volt);
  Swing(25, 1, 600, false);
  correct_angle = 10;
  DriveTo(90, 600);
  DriveTo(-15, 1500);
  correct_angle = 20;
  Grab(-100);
  DriveTo(90, 800);
  xpos = 0;
  ypos = 0;
  MoveToPoint(-12, -15, -1, 1500, false);
  TurnToAngle(200, 100);
  TurnToPoint(-53, 2, 1, 300);
  Grab(100);
  MoveToPoint(-53, 2, 1, 1200, false);
  TurnToAngle(40, 600);
  dirchangeend = false;
  MoveToPoint(-30, 12, 1, 1500, false);
  correct_angle = 90;;
  Grab(0);
  DriveTo(90, 500);
  xpos = 0;
  ypos = 0;
  dirchangeend = true;
  MoveToPoint(-7, 0, -1, 1000, false);
  TurnToPoint(-23, 8, 1, 300);
  Grab(100);
  MoveToPoint(-23, 8, 1, 1300, false);
  TurnToPoint(-35, -150, 1, 400);
  TurnToAngle(-270, 200);
  dirchangeend = false;
  MoveToPoint(5, -7, 1, 2000, false);
  Grab(0);
  DriveTo(200, 700);
  MoveToPoint(-10, -3, -1, 1500);
}

void Far6Low() {
  Arm(100);
  Grab(100);
  ChassisControl(6, 12);
  wait(200, msec);
  MoveToPoint(-11, 47, 1, 2000, false);
  Arm(-100);
  Stop(hold);
  wait(200, msec);
  TurnToAngle(95, 600);
  awp_motor.stop(hold);
  Grab(-100);
  wait(200, msec);
  Grab(100);
  ChassisControl(-12, 12);
  wait(100, msec);
  TurnToPoint(-19, 44, 1, 300);
  boomerang(-19, 44, -90, 0, 1500);
  ChassisControl(-8, -12);
  wait(200, msec);
  ChassisControl(0, 0);
  TurnToPoint(15, 32, 1, 500);
  Grab(-100);
  MoveToPoint(15, 32, 1, 800);
  ChassisControl(-12, -12);
  wait(100, msec);
  TurnToPoint(-50, 26, 1, 400);
  Grab(100);
  boomerang(-31.5, 29, -90, 0.5, 1500);
  //CurveCircle(0, -16, 2000);
  MoveToPoint(-4, 15, -1, 2000);
  TurnToAngle(70, 400);
  Grab(-100);
  wait(200, msec);
  //TurnToAngle(140, 600);
  //Swing(270, 1, 800, false);
  Grab(100);
  TurnToAngle(180, 500);
  ChassisControl(12, 10);
  wait(200, msec);
  Swing(250, 1, 800, false);
  boomerang(-36, 2.5, -90, 0.2, 2500);
  correct_angle = 270;
  //MoveToPoint(-30, 8, 1, 2000);
  CurveCircle(255, 110, 2000);
  TurnToAngle(90, 600);
  Grab(0);
  CurveCircle(45, -28, 1500, 6);
  Arm(100);
  wait(200, msec);
  Swing(-10, 1, 600, false);
  Swing(50, -1, 600, false);
  Arm(-100);
  awp_motor.spin(fwd, -12, voltageUnits::volt);
  ChassisControl(12, 12);
  wait(200, msec);
  Swing(30, 1, 500, false);
  correct_angle = 10;
  DriveTo(90, 800);
  Swing(430, -1, 800);
}

void Far6Top() {
  Arm(-100);
  Grab(100);
  MoveToPoint(-30, 48, 1, 2000);
  awp_motor.stop(hold);
  Stop(hold);
  TurnToPoint(16, 48, 1, 500);
  MoveToPoint(16, 48, 1, 1100);
  ChassisControl(-12, -12);
  wait(150, msec);
  TurnToPoint(-30, 31, 1, 400);
  Grab(100);
  boomerang(-30, 31, -90, 0.4, 2500);
  //CurveCircle(0, -16, 2000);
  MoveToPoint(-1, 20, -1, 2000);
  TurnToAngle(70, 500);
  Grab(-100);
  wait(300, msec);
  //TurnToAngle(140, 600);
  //Swing(270, 1, 800, false);
  Grab(100);
  TurnToAngle(180, 400);
  Brain.Screen.clearScreen();
  Brain.Screen.print(GetInertialHeading());
  ChassisControl(12, 10);
  wait(350, msec);
  Swing(610, 1, 800, false);
  boomerang(-31, 2, -90, 0.2, 2500);
  correct_angle = 630;
  //MoveToPoint(-30, 8, 1, 2000);
  CurveCircle(615, 110, 2000);
  TurnToAngle(450, 700);
  Grab(0);
  CurveCircle(405, -30, 1500);
  Arm(100);
  wait(200, msec);
  Swing(360, 1, 800, false);
  Swing(430, -1, 350, false);
  Arm(-100);
  awp_motor.spin(fwd, -12, voltageUnits::volt);
  ChassisControl(12, 12);
  wait(150, msec);
  ChassisControl(0, 12);
  Grab(-100);
  wait(250, msec);
  correct_angle = 360;
  DriveTo(70, 1000);
  /*
  CurveCircle(400, -50, 1000);
  correct_angle = 370;
  DriveTo(80, 1000);
  */
  Swing(480, -1, 800, false);
  MoveToPoint(-10, 60, -1, 1500);
}

void Far6Safe() {
  MoveToPoint(-8, 14, 1, 2000, false);
  correct_angle = -90;
  Grab(-100);
  DriveTo(50, 600);
  MoveToPoint(-8, 12, -1, 1500);
  Swing(-40, -1, 300);
  DriveTo(3, 500);
  Grab(0);
  Arm(100);
  wait(200, msec);
  Swing(-90, 1, 1000, false);
  Swing(-50, -1, 800, false);
  Arm(-100);
  ChassisControl(12, 12);
  wait(150, msec);
  Swing(-70, 1, 800, false);
  correct_angle = -80;
  DriveTo(70, 500);
  Brain.Screen.clearScreen();
  Brain.Screen.print(xpos);
  Brain.Screen.newLine();
  Brain.Screen.print(ypos);
  xpos = -32;
  ypos = 26;
  MoveToPoint(-18, 12, -1, 1000);
  TurnToPoint(-52, -20, 1, 200);
  Grab(100);
  MoveToPoint(-52, -20, 1, 2000);
  TurnToPoint(-45, 25, 1, 300);
  Grab(0);
  MoveToPoint(-45, 25, 1, 1100);
  ChassisControl(-12, -12);
  wait(100, msec);
  TurnToPoint(-35, -22, 1, 300);
  Grab(100);
  boomerang(-29, -22, 180, 0.4, 1500, 1);
  MoveToPoint(-46, -15, -1, 1500, false);
  TurnToPoint(-46, 20, 1, 200);
  Grab(-100);
  MoveToPoint(-46, 20, 1, 800);
}

void Far6SafeBar() {
  Far6Safe();
  ChassisControl(-12, -12);
  wait(100, msec);
  TurnToPoint(0, 8, 1, 500);
  MoveToPoint(0, 8, 1, 2000, false);
  MoveToPoint(0, -24, 1, 2000, false);
  TurnToAngle(180, 200);
  ChassisControl(2, 2);
  while(Distance14.value() > 70) {
    wait(10, msec);
  }
  ChassisControl(0, 0);
}

void Far6SafeNoBar() {
  Far6Safe();
  DriveTo(-20, 1500);
}

void FarAWPInside() {
  MoveToPoint(-8, 14, 1, 2000, false);
  correct_angle = -90;
  Grab(-100);
  DriveTo(35, 1500);
  Swing(0, -1, 800, false);
  CurveCircle(30, -32, 2000, false);
  CurveCircle(-63, 18, 2000);
  Arm(-100);
  wait(300, msec);
  Arm(0);
}

void FarAWPOutside() {
  MoveToPoint(-8, 14, 1, 2000, false);
  correct_angle = -90;
  Grab(-100);
  DriveTo(35, 1000);
  Swing(-60, -1, 800, false);
  CurveCircle(0, -30, 2000, false);
  MoveToPoint(-10, -24, -1, 2000);
  Arm(-100);
  wait(300, msec);
  Arm(0);
}

void Far5() {
  thread headingcorrection = thread(heading_correction);
  double begin_time = Brain.timer(msec);
  Grab(100);
  DriveTo(7, 1000);
  CurveCircle(-10, 130, 1500, false);
  CurveCircle(-80, 30, 1500, false);
  CurveCircle(-90, 50, 1500, false);
  CurveCircle(-85, 100, 1000);
  TurnToAngle(90, 800);
  Grab(-100);
  wait(200, msec);
  DriveTo(20, 1000);
  DriveTo(-20, 1000);
  TurnToAngle(30, 800);
  CurveCircle(35, 300, 1500, false);
  Grab(100);
  CurveCircle(53, 100, 1500, false);
  Swing(0, 1, 800);
  DriveTo(-30, 1800);
  TurnToAngle(-180, 800);
  Grab(-100);
  Swing(-375, -1, 1000);
  Grab(100);
  CurveCircle(-355, 60, 1500);
  Swing(-560, -1, 1000);
  Grab(-100);
  CurveCircle(-530, 60, 1000);
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
  DriveTo(18, 1500);
  Grab(100); 
  Arm(-100);
  CurveCircle(95, -92, 2000);
  DriveTo(3, 800, 6);
  Arm(0);
  Swing(-30, -1, 1000);
  //DigitalOutA.set(true);
  CurveCircle(-45, 30, 1000, false);
  Swing(0, -1, 700);
  /*
  CurveCircle(-20, -13, 800, false);
  //DigitalOutA.set(false);
  CurveCircle(-70, -76, 2000, false);
  Arm(100);
  CurveCircle(-180, -17, 1000, false);
  Arm(0);
  */
  //DigitalOutA.set(false);
  TurnToPoint(0, 0, 1, 1000);
  MoveToPoint(0, 0, 1, 4000);
  TurnToPoint(0, -41, 1, 1000);
  DriveTo(41, 2700, 11);
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
  DriveTo(-3, 1000);
  Swing(0, -1, 1000);
  Grab(100);
  DriveTo(17, 1200);
  Swing(-120, -1, 1000);
  //DigitalOutA.set(true);
  CurveCircle(-135, 30, 1000, false);
  Swing(-90, -1, 700);
  CurveCircle(-110, -13, 800, false);
  //DigitalOutA.set(false);
  CurveCircle(-160, -76, 2000, false);
  CurveCircle(-270, -17, 1000, false);
  DriveTo(41, 2700, 11);
  Grab(-100);
  wait(300, msec);
  CurveCircle(-260, -280, 2000, false);
  Swing(-300, -1, 800);
  Arm(100);
  double end_time = Brain.timer(msec);
  Brain.Screen.newLine();
  Brain.Screen.printAt(80, 80, "%f", end_time - begin_time);
}

void NearAWP() {
  double begin_time = Brain.timer(msec);
  Arm(100);
  wait(300, msec);
  //Swing(-90, 1, 1500, false);
  TurnToAngle(-90, 1000);
  Arm(-100);
  TurnToAngle(-25, 500);
  Arm(0);
  CurveCircle(45, -20, 1500);
  DriveTo(-25, 600);
  Brain.Screen.clearScreen();
  Brain.Screen.print(xpos);
  Brain.Screen.newLine();
  Brain.Screen.print(ypos);
  xpos = -4;
  ypos = -15;
  MoveToPoint(-10, 21, 1, 2000, false);
  Grab(-100);
  Swing(-35, 1, 800, false);
  correct_angle = -45;
  DriveTo(17, 2000, false);
  ChassisControl(2, 2);
  while(Distance14.value() > 70) {
    wait(10, msec);
  }
  ChassisControl(0, 0);
  double end_time = Brain.timer(msec);
  Brain.Screen.newLine();
  Brain.Screen.printAt(80, 80, "%f", end_time - begin_time);
}

void NearBottomElim() {
  awp_motor.stop(hold);
  Grab(100);
  MoveToPoint(10, 48, 1, 2000);
  MoveToPoint(22, 47, -1, 1500);
  TurnToAngle(-150, 1000);
  Arm(-12);
  wait(300, msec);
  awp_motor.stop(coast);
  TurnToAngle(-90, 700);
  Arm(12);
  Swing(0, -1, 800, false);
  TurnToPoint(7, 10, -1, 400);
  MoveToPoint(7, 10, -1, 2000);
  TurnToAngle(-20, 600);
  Arm(-12);
  wait(300, msec);
  awp_motor.stop(coast);
  TurnToAngle(90, 800);
  Arm(12);
  CurveCircle(180, -21, 2000);
  DriveTo(-25, 700);
  CurveCircle(135, -27, 2000, false);
  DriveTo(7, 1200);
  TurnToAngle(45, 800);
  Arm(-12);
  wait(300, msec);
  awp_motor.stop(coast);
  TurnToAngle(-30, 800);
  TurnToAngle(110, 800);
  Arm(12);
  CurveCircle(90, -100, 2000);
  Grab(-100);
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
  catapult_motor.stop(coast);
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
  double begin_time = Brain.timer(msec);
  boomerang(25, 14, 90, 0.5, 2000, 1, false);
  Grab(-100);
  DriveTo(50, 500);
  boomerang(15, 17, 165, 0.5, 2000, -1);
  /*
  Arm(-100);
  wait(300, msec);
  Arm(0);
  */
  Grab(0);
  double xpostemp = xpos;
  double ypostemp = ypos;
  catapult_motor.spin(fwd, 12, volt);
  wait(30000, msec);
  catapult_motor.stop(coast);
  xpos = xpostemp;
  ypos = ypostemp;
  //catapultlaunch(47, 1000);
  DriveTo(3, 500);
  TurnToAngle(60, 500);
  ChassisControl(-12, -12);
  wait(250, msec);
  boomerang(30, -98, -90, 0.5, 3000, -1);
  DriveTo(-70, 800);
  MoveToPoint(10, -90, 1, 1000);
  TurnToAngle(-30, 500);
  CurveCircle(-90, 30, 1500, false);
  DriveTo(-70, 500);
  DriveTo(3, 500);
  Swing(20, 1, 800, false);
  ChassisControl(12, 12);
  wait(330, msec);
  MoveToPoint(34, -69, 1, 3000);
  TurnToAngle(90, 600);
  TurnToAngle(-70, 500);
  CurveCircle(0, -15, 1500, false);
  DriveTo(-50, 1000);
  MoveToPoint(35, -69, 1, 2000);
  MoveToPoint(38, -60, -1, 2000, false);
  TurnToAngle(0, 300);
  DriveTo(-80, 1000);
  MoveToPoint(55, -70, 1, 2000, false);
  Swing(90, 1, 800, false);
  MoveToPoint(80, -70, 1, 2000);
  TurnToAngle(180, 500);
  MoveToPoint(70, -71, -1, 2000);
  Swing(10, -1, 800, false);
  correct_angle = 0;
  DriveTo(-60, 1000);
  TurnToAngle(90, 400);
  MoveToPoint(122, -110, 1, 2000);
  TurnToAngle(200, 800);
  CurveCircle(270, 40, 1200, false);
  DriveTo(70, 800);
  DriveTo(-20, 1500);
  correct_angle = 260;
  DriveTo(70, 800);
  double end_time = Brain.timer(msec);
  Brain.Screen.newLine();
  Brain.Screen.printAt(80, 80, "%f", end_time - begin_time);
}

void tag() {
  dirchangestart = true;
  dirchangeend = true;
  MoveToPoint(-40, 10, 1, 3000, false);
  MoveToPoint(0, -2, -1, 2000, false);
  /*
  dirchangestart = false;
  dirchangeend = false;
  MoveToPoint(-40, 5, 1, 3000, false);
  MoveToPoint(-81, -2, 1, 3000, false);
  MoveToPoint(-82, 37, 1, 3000, false);
  MoveToPoint(-81, 65, 1, 3000, false);
  MoveToPoint(-40, 60, 1, 3000, false);
  MoveToPoint(15, 65, 1, 3000, false);
  MoveToPoint(20, 35, 1, 3000, false);
  dirchangeend = true;
  MoveToPoint(15, 5, 1, 3000, false);
  dirchangestart = true;
  MoveToPoint(20, 35, 1, 3000, false);
  Stop(hold);
  */
  /*
  for(int i = 0; i < 10; i++) {
    //MoveToPoint(0, 0, 1, 2000, false);
    //distance between wheels
    MoveToPoint(-35, 10, 1, 3000, false);
    MoveToPoint(-75, 0, 1, 3000, false);
    MoveToPoint(-82, 37, 1, 3000, false);
    MoveToPoint(-81, 65, 1, 3000, false);
    MoveToPoint(-35, 60, 1, 3000, false);
    MoveToPoint(15, 65, 1, 3000, false);
    MoveToPoint(15, 35, 1, 3000, false);
    MoveToPoint(15, 0, 1, 3000, false);
    MoveToPoint(-35, 5, 1, 3000, false);
  }
  */
  /*
  MoveToPoint(-35, 15, 1, 3000, false);
  MoveToPoint(-75, 0, 1, 3000, false);
  MoveToPoint(-82, 37, 1, 3000, false);
  MoveToPoint(-81, 70, 1, 3000, false);
  MoveToPoint(-35, 55, 1, 3000, false);
  MoveToPoint(15, 70, 1, 3000, false);
  MoveToPoint(20, 35, 1, 3000, false);
  MoveToPoint(15, 0, 1, 3000, false);
  MoveToPoint(0, 0, 1, 2000);
  */
}

void TestDriveMotors() {
  Brain.Screen.clearScreen();
  Brain.Screen.print("Left Motor 1");
  left_chassis1.spin(fwd, 12, voltageUnits::volt);
  wait(2000, msec);
  left_chassis1.stop(coast);
  wait(1000, msec);
  Brain.Screen.clearScreen();
  Brain.Screen.print("Left Motor 2");
  left_chassis2.spin(fwd, 12, voltageUnits::volt);
  wait(2000, msec);
  left_chassis2.stop(coast);
  wait(1000, msec);
  Brain.Screen.clearScreen();
  Brain.Screen.print("Left Motor 3");
  left_chassis3.spin(fwd, 12, voltageUnits::volt);
  wait(2000, msec);
  left_chassis3.stop(coast);
  wait(1000, msec);
  Brain.Screen.clearScreen();
  Brain.Screen.print("Right Motor 1");
  right_chassis1.spin(fwd, 12, voltageUnits::volt);
  wait(2000, msec);
  right_chassis1.stop(coast);
  wait(1000, msec);
  Brain.Screen.clearScreen();
  Brain.Screen.print("Right Motor 2");
  right_chassis2.spin(fwd, 12, voltageUnits::volt);
  wait(2000, msec);
  right_chassis2.stop(coast);
  wait(1000, msec);
  Brain.Screen.clearScreen();
  Brain.Screen.print("Right Motor 3");
  right_chassis3.spin(fwd, 12, voltageUnits::volt);
  wait(2000, msec);
  right_chassis3.stop(coast);
}

void TestPID() {
  dirchangestart = true;
  dirchangeend = true;
  MoveToPoint(-10, 50, 1, 2000, false);
  dirchangeend = false;
  MoveToPoint(0, 0, -1, 2000, false);
  dirchangestart = false;
  dirchangeend = true;
  MoveToPoint(-30, -5, -1, 2000, false);
  dirchangestart = true;
  MoveToPoint(0, 0, 1, 2000, false);
  ChassisControl(0, 0);
  /*
  DriveTo(60, 5000);
  TurnToAngle(90, 1000);
  TurnToAngle(180, 1000);
  DriveTo(24, 5000);
  DriveTo(-24, 5000);
  DriveTo(60, 5000);
  TurnToAngle(0, 1000);
  */
  /*
  TurnToAngle(180, 5000);
  TurnToAngle(90, 5000);
  TurnToAngle(45, 5000);
  TurnToAngle(10, 5000);
  TurnToAngle(0, 5000);
  */
}