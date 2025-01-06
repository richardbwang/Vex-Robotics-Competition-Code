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

void intake_thread(){
  intake(12);
  optical_sensor.setLight(ledState::on);
  while(!optical_sensor.isNearObject()){
    task::sleep(10);
  }
  intake_stop();
}

void intake_th(){
  task::sleep(300);
  int cnt=0;
  while(!optical_sensor.isNearObject()){
    cnt++;
    task::sleep(10);
    if (cnt>100){
      break;
    }
  }
  cnt=0;
  intake(8);
  while(optical_sensor.isNearObject()){
    cnt++;
    task::sleep(10);
    if (cnt>100){
      break;
    }
  }
  intake_stop(hold);
}
void intake_color_red(){
  bool wrong_color=false;
  optical_sensor.setLight(ledState::on);
  optical_sensor.setLightPower(100);
  intake(12);
  while(1) {
    // // if (colr==0){ // red dispose
    // if (optical_sensor.isNearObject() && (optical_sensor.hue() > 320 || optical_sensor.hue() < 90)) {
    //   wrong_color = true;
    // }
    // }else{ // blue dispose
    if (optical_sensor.isNearObject() && (optical_sensor.hue() < 300 and optical_sensor.hue() > 90)) {
      wrong_color = true;
    }
    if (distance_sensor.objectDistance(distanceUnits::mm) < 50 && wrong_color) {
      Sort.set(true);
      wait(600, msec);
      Sort.set(false);
      wrong_color = false;
    }
    wait(10, msec);
  }
}

void intake_color_blue(){
  bool wrong_color=false;
  optical_sensor.setLight(ledState::on);
  optical_sensor.setLightPower(100);
  while(1) {
    // // if (colr==0){ // red dispose
    if (optical_sensor.isNearObject() && (optical_sensor.hue() > 340 || optical_sensor.hue() < 60)) {
      wrong_color = true;
    }
    // }else{ // blue dispose
    // if (optical_sensor.isNearObject() && (optical_sensor.hue() < 295 and optical_sensor.hue() > 90)) {
    //   wrong_color = true;
    // }
    if (distance_sensor.objectDistance(distanceUnits::mm) < 50 && wrong_color) {
      intake_stop(hold);
      task::sleep(300);
      wrong_color = false;
      intake(12);
    }
    // very important
    wait(10, msec);
  }
}

void intake_arm() {
  intake(12);
  while(true) {
    if(optical_sensor.isNearObject()) {
      break;
    }
    wait(10, msec);
  }
  intake(6);
  while(true) {
    if(distance_sensor_arm.objectDistance(distanceUnits::mm) < 30) {
      break;
    }
    wait(10, msec);
  }
  intake_stop(hold);
  filter.set(true);
}

void intake_arm_outtake() {
  intake(-6);
  wait(600, msec);
  filter.set(false);
}

void BlueLeft() {
  goal_clamp.set(true);
  MoveToPoint(-6, 18, 1, 2500, false);
  thread it = thread(intake_thread);
  correct_angle = NormalizeTarget(-45);
  DriveTo(3, 1000, false);
  TurnToAngle(-135, 800, false);
  correct_angle = NormalizeTarget(180);
  DriveTo(-12, 2000);
  goal_clamp.set(false);
  wait(400, msec);
  it.interrupt();
  intake(12);
  TurnToAngle(-150, 800, false);
  correct_angle = NormalizeTarget(-135);
  DriveTo(10, 1500, false);
  intake(-12);
  PTO.set(true);
  correct_angle = NormalizeTarget(-170);
  DriveTo(1000, 1500, false);
  while(inertial_sensor.rotation() > NormalizeTarget(90)) {
    ChassisControl(-6, 12);
    wait(10, msec);
  }
  correct_angle = NormalizeTarget(180);
  DriveTo(-10, 1500, false);
  correct_angle = NormalizeTarget(-170);
  DriveTo(1000, 1000, false);
  while(inertial_sensor.rotation() > NormalizeTarget(120)) {
    ChassisControl(-6, 12);
    wait(10, msec);
  }
  TurnToAngle(110, 700);
  correct_angle = NormalizeTarget(100);
  DriveTo(20, 4000, false, 3);
  Stop(hold);
}

void BlueRightAWP() {
  PTO.set(true);
  goal_clamp.set(true);
  intake(-12);
  CurveCircle(34, 21, 2500);
  intake(10);
  wait(300, msec);
  PTO.set(false);
  thread intakethread = thread(intake_thread);
  wait(200, msec);
  MoveToPoint(0, 0, -1, 3000, false, 6);
  intakethread.interrupt();
  intake(-8);
  Stop(hold);
  wait(200, msec);
  thread intakethread2 = thread(intake_thread);
  wait(400, msec);
  TurnToAngle(140, 800, false);
  intakethread2.interrupt();
  PTO.set(true);
  intake(-10);
  MoveToPoint(4, 16, -1, 2000);
  TurnToAngle(-90, 500);
  DriveTo(4, 1000);
  intake(12);
  wait(1000, msec);
  intake_stop(coast);
  DriveTo(-4, 800, false);
  PTO.set(false);
  Swing(-45, -1, 800, false);
  MoveToPoint(27, -3, -1, 2500, false);
  thread intakecolorblue = thread(intake_color_blue);
  goal_clamp.set(false);
  wait(200, msec);
  TurnToAngle(180, 800, false);
  MoveToPoint(38, -14, 1, 2000);
  wait(500, msec);
  DriveTo(-3, 800, false);
  MoveToPoint(28, -10, -1, 2000, false);
  MoveToPoint(39, -22, 1, 2000);
  wait(500, msec);
  MoveToPoint(28, -5, -1, 2000, false);
  TurnToAngle(170, 800, false);
  MoveToPoint(30, -20, 1, 2000, false);
  wait(200, msec);
  TurnToAngle(0, 800, false);
  Stop(hold);
  boomerang(40, 12, 45, 0.4, 2000, 1);
}

void AwpStake(bool isright){
  // start
  xpos=0;
  ypos=0;
  float startangle=33;
  inertial_sensor.setRotation((isright) ? startangle:180-startangle, degrees);

  // lift intake/arm for first red ring
  PTO.set(true);
  goal_clamp.set(true);
  intake(-10);
  dirchangestart = false;
  dirchangeend = false;
  MoveToPoint(xpos+5.5,(isright) ? ypos+7.5 : ypos-7.5,1,2000,true,4);
  intake_stop();
  intake(12);
  task::sleep(100);
  PTO.set(false);
  thread intakethread=thread(intake_thread); // thread intake process
  task::sleep(200);
  MoveToPoint(xpos-1.5,(isright) ? ypos-2.5 : ypos+2.5,-1,2000,true,4); 

  // prepare to stab ring onto alliance stake
  TurnToAngle((isright) ? 180 : 0,1000,true,6);
  task::sleep(600);
  intakethread.interrupt();
  intake_stop();
  optical_sensor.setLight(ledState::off);
  intake(6);
  PTO.set(true);
  task::sleep(150);
  intake(-8);
  MoveToPoint(xpos,(isright) ? ypos+14.25 : ypos-14.25,-1,2000,true,8);
  // push remaining out the way
  task::sleep(200);
  intake_stop(hold);
  TurnToAngle(-90,1000,true,10);
  MoveToPoint(xpos-4,ypos,1,2000,true,3);
  intake(8); // STAB!
  optical_sensor.setLight(ledState::off);
  task::sleep(600);
  intake_stop(coast); // removal process
  PTO.set(false);
  MoveToPoint(xpos+12,ypos,-1,1000,true,12);
  TurnToAngle(-90,1000,true,12);

  // reset coordinates
  xpos=0;ypos=0;
  MoveToPoint(xpos-4,ypos,1,1000,true,12);

  // line up and go to mogo
  TurnToAngle(((isright) ?  -50: -130),1000,true,12);
  MoveToPoint(xpos+18,(isright) ? ypos-14 : ypos+14,-1,4000,true,12);
  task::sleep(100);
  MoveToPoint(xpos+7,(isright) ? ypos-2 : ypos+2,-1,2000,true,3);
  goal_clamp.set(false); // GRAB!!
  PTO.set(false);
  intake(-3);
  task::sleep(150);
  intake(12);
  TurnToAngle((isright) ? 180 : 0,1000,true,12);
  MoveToPoint(xpos,(isright) ? ypos-15 : ypos+15,1,1000,true,12);
  task::sleep(500);
  MoveToPoint(xpos+6,(isright) ? ypos+10 : ypos-10,-1,2000,true,12);
  goal_clamp.set(true);
  TurnToAngle((isright) ? 0 : 180,1000,true,10);
  intake_stop();
  MoveToPoint(xpos+4,(isright) ? ypos+10 : ypos-10,1,2000,true,10);
  MoveToPoint(xpos,(isright) ? ypos+3 : ypos-3,1,1000,true,1);
}

void RedElimMogo(){
  xpos=0;
  ypos=0;
  inertial_sensor.setRotation(45, degrees);
  goal_clamp.set(true);
  thread intakethread=thread(intake_th);
  intake(12);
  MoveToPoint(xpos+20,ypos+20,1,4000,true,10);
  task::sleep(200);
  TurnToAngle(180,2000,true,7);
  MoveToPoint(xpos,ypos+13.5,-1,3500,false,4);
  goal_clamp.set(false);
  Stop(hold);
  task::sleep(200);
  intakethread.interrupt();
  intake(12);
  thread intake_thread=thread(intake_color_red);
  MoveToPoint(xpos,ypos-7,1,2000,true,3);
  task::sleep(500);
  intake_stop();
  goal_clamp.set(true);
  MoveToPoint(xpos,ypos-3.25,1,2000,true,8);
  task::sleep(20);
  TurnToAngle(90,1000,true,8);
  MoveToPoint(xpos-15,ypos,-1,4000,false,4.5);
  goal_clamp.set(false);
  Stop(hold);
  TurnToAngle(-135,1000,true,10);
  PTO.set(true);
  intake(-12);
  MoveToPoint(xpos-12.5,ypos-12.5,1,2000,true,8);
  PTO.set(false);
  intake(12);
  task::sleep(500);
  MoveToPoint(xpos+10,ypos+8,-1,2000,true,4);
  task::sleep(100);
  TurnToAngle(180,1000,true,5);
  goal_clamp.set(true);
  MoveToPoint(xpos,ypos-20,1,1000,true,10);
  task::sleep(50);
  TurnToAngle(90,1000,true,10);
  PTO.set(true);
  intake(-10);
  task::sleep(50);
  MoveToPoint(xpos-14.5,ypos,-1,1000,true,8);
  intake_stop(hold);
  task::sleep(50);
  TurnToAngle(180,1000,true,10);
  MoveToPoint(xpos,ypos-2.5,1,2000,true,5);
  intake(8);
  task::sleep(500);
  PTO.set(false);
  MoveToPoint(xpos,ypos+30,-1,2000,false,12);
}

void AwpMogo(){
  xpos=0;
  ypos=0;
  // mogo mech point to the front.
  inertial_sensor.setRotation(180,degrees);
  goal_clamp.set(true);
  // move to mogo
  MoveToPoint(xpos,ypos+20,-1,2000,true,10);
  MoveToPoint(xpos,ypos+5,-1,3000,false,3);
  goal_clamp.set(false);
  Stop(hold);
  task::sleep(100);

  // get side ring
  TurnToAngle(-90,1000,true,8); 
  intake(12);
  MoveToPoint(xpos-15,ypos,1,3000,true,10);
  TurnToAngle(116,1000,true,9);
  thread intake_thread=thread(intake_color_red);

  // get the center ring
  MoveToPoint(xpos+30,ypos-15,1,4000,true,8);
  MoveToPoint(xpos+4,ypos-6,1,3000,true,8);
  task::sleep(100);
  TurnToAngle(90,3000,true,8);
  task::sleep(50);
  MoveToPoint(xpos+10,ypos,1,2000,true,10);
  task::sleep(700);
  intake_thread.interrupt();
  goal_clamp.set(true);

  // move to second mogo
  MoveToPoint(xpos+10,ypos,1,2000,true,8);
  intake_stop(hold);
  TurnToAngle(180,10000,true,8);
  MoveToPoint(xpos,ypos+7,-1,1000,false,6);
  MoveToPoint(xpos,ypos+5,-1,1000,false,3);
  goal_clamp.set(false);
  Stop(hold);

  // get another side ring
  intake(12);
  TurnToAngle(90,1000,true,8);
  MoveToPoint(xpos+15,ypos,1,2000,true,10);
  task::sleep(100);
  TurnToAngle(-90,2000,true,4);
  MoveToPoint(xpos-20,ypos+5,1,1000,true,12);
  goal_clamp.set(true);
  MoveToPoint(xpos-7,ypos+1,1,1000,true,8);
}

void RedElimRing(){
  xpos=0;
  ypos=0;
  inertial_sensor.setRotation(180, degrees);
  goal_clamp.set(true);
  MoveToPoint(xpos,ypos+14,-1,2000,true,9);
  TurnToAngle(210,1000,true,8);
  MoveToPoint(xpos+5,ypos+8.6,-1,5000,false,3);
  goal_clamp.set(false);
  Stop(hold);
  TurnToAngle(-45,1000,true,8);
  intake(12);
  thread intake_thread(intake_color_red);
  MoveToPoint(xpos-9.25,ypos+9.25,1,3000,true,8);
  task::sleep(1000);
  MoveToPoint(xpos+3.75,ypos-10.75,-1,1000,true,8);
  task::sleep(10);
  MoveToPoint(xpos-9.75,ypos+9.75,1,3000,true,8);
  task::sleep(1000);
  MoveToPoint(xpos+10.75,ypos-9.75,-1,1000,true,10);
  task::sleep(10);
  TurnToAngle(-90,1000,true,8);
  MoveToPoint(xpos-15,ypos,1,1000,true,10);
  task::sleep(300);
  TurnToAngle(180,1000,true,4);
  CurveCircle(100,-18,2000,true,10);
  intake_stop();
  task::sleep(800);
  MoveToPoint(xpos+50,ypos,1,5000,false,12);
  goal_clamp.set(true);
  MoveToPoint(xpos+10,ypos,1,2000,true,12);
}

void BlueElimRing(){
  xpos=0;
  ypos=0;
  inertial_sensor.setRotation(180, degrees);
  goal_clamp.set(true);
  MoveToPoint(xpos,ypos+14,-1,2000,true,9);
  TurnToAngle(150,1000,true,8);
  MoveToPoint(xpos-5,ypos+8.6,-1,5000,false,3);
  goal_clamp.set(false);
  Stop(hold);
  TurnToAngle(45,1000,true,8);
  intake(12);
  thread intake_thread(intake_color_blue);
  MoveToPoint(xpos+9.25,ypos+9.25,1,3000,true,8);
  task::sleep(1000);
  MoveToPoint(xpos-3.75,ypos-10.75,-1,1000,true,8);
  task::sleep(10);
  MoveToPoint(xpos+9.75,ypos+9.75,1,3000,true,8);
  task::sleep(1000);
  MoveToPoint(xpos-10.75,ypos-9.75,-1,1000,true,10);
  task::sleep(10);
  TurnToAngle(90,1000,true,8);
  MoveToPoint(xpos+15,ypos,1,1000,true,10);
  task::sleep(300);
  TurnToAngle(230,1000,true,4);
  // move
  intake_stop();
  task::sleep(1000);
  MoveToPoint(xpos-50,ypos-15,1,5000,false,12);
  goal_clamp.set(true);
  MoveToPoint(xpos-10,ypos-15,1,2000,true,12);
}

void skills() {
  goal_clamp.set(false);
  PTO.set(true);
  intake(-8);
  DriveTo(-1, 800, false);
  intake(-12);
  DriveTo(-3, 800, false);
  DriveTo(5, 2000);
  intake(12);
  PTO.set(false);
  wait(800, msec);
  intake_stop(coast);
  DriveTo(-2, 800, false);
  TurnToAngle(60, 800, false);
  boomerang(-14, -2, 90, 0.5, 3000, -1, false);
  ChassisControl(-3, -3);
  wait(600, msec);
  goal_clamp.set(true);
  wait(200, msec);
  intake(-12);
  Stop(hold);
  wait(400, msec);
  intake(12);
  MoveToPoint(-18, -11, 1, 2000, false);
  MoveToPoint(-39, -34, 1, 3000, false);
  correct_angle = NormalizeTarget(-90);
  DriveTo(14, 2000);
  DriveTo(-4, 1000, false);
  intake(0);
  TurnToAngle(-60, 800, false);
  intake(12);
  MoveToPoint(-47, -16, 1, 2000, false);
  MoveToPoint(-49, 0, 1, 3500, false, 8);
  MoveToPoint(-50, 8, 1, 3500, false, 4);
  correct_angle = NormalizeTarget(-45);
  DriveTo(-13, 3000, false, 6);
  MoveToPoint(-59, 4, 1, 3000);
  wait(500, msec);
  intake(0);
  TurnToAngle(-170, 800, false);
  intake(12);
  correct_angle = NormalizeTarget(135);
  DriveTo(-1000, 1500, false, 8);
  Stop(hold);
  xpos = 0;
  ypos = 0;
  goal_clamp.set(false);
  wait(400, msec);
  DriveTo(4, 1000, false);
  TurnToAngle(-135, 800, false);
  intake(0);
  boomerang(50, -2, -90, 0.3, 4000, -1, false);
  ChassisControl(-3, -3);
  wait(1000, msec);
  goal_clamp.set(true);
  wait(200, msec);
  Stop(hold);
  intake(-12);
  wait(500, msec);
  intake(12);
  MoveToPoint(62, -18, 1, 2000, false);
  TurnToAngle(150, 800, false);
  MoveToPoint(80, -32, 1, 3000, false);
  correct_angle = NormalizeTarget(90);
  DriveTo(14, 2000);
  DriveTo(-4, 1000, false);
  intake(0);
  TurnToAngle(60, 800, false);
  intake(12);
  MoveToPoint(86, -22, 1, 2000, false);
  MoveToPoint(87, -4, 1, 3500, false, 8);
  MoveToPoint(88, 8, 1, 3500, false, 4);
  correct_angle = NormalizeTarget(45);
  DriveTo(-15, 3000, false, 6);
  MoveToPoint(99, 3, 1, 2500);
  thread ia = thread(intake_arm);
  wait(300, msec);
  TurnToAngle(170, 800, false);
  correct_angle = NormalizeTarget(-135);
  DriveTo(-1000, 1500, false, 8);
  Stop(hold);
  xpos = 0;
  ypos = 0;
  goal_clamp.set(false);
  ia.interrupt();
  intake_arm_outtake();
  intake(-12);
  PTO.set(true);
  MoveToPoint(-2, -41, 1, 5000, true, 6);
  TurnToAngle(90, 700);
  DriveTo(1000, 1500, false, 4);
  intake(12);
  PTO.set(false);
  wait(800, msec);
  intake_stop(coast);
  DriveTo(-4, 1200, false);
  TurnToAngle(150, 800, false);
  thread it2 = thread(intake_thread);
  MoveToPoint(-1, -59, 1, 3000, false);
  TurnToAngle(110, 800, false);
  //MoveToPoint(-30, -70, -1, 4000, false);
  correct_angle = NormalizeTarget(75);
  DriveTo(-4, 1500, false);
  boomerang(-23, -74, 90, 0.1, 4000, -1, false, 8);
  ChassisControl(-3, -3);
  wait(1200, msec);
  goal_clamp.set(true);
  wait(200, msec);
  it2.interrupt();
  intake(-12);
  DriveTo(3, 800, false);
  intake(12);
  MoveToPoint(-20, -54, 1, 3000, false);
  TurnToAngle(100, 800, false);
  thread ir = thread(intake_color_red);
  MoveToPoint(-1, -71, 1, 3000);
  wait(500, msec);
  correct_angle = NormalizeTarget(90);
  DriveTo(-10, 2500, false);
  MoveToPoint(2, -83, 1, 3000);
  wait(300, msec);
  correct_angle = NormalizeTarget(180);
  DriveTo(-18, 3000, false, 8);
  ir.interrupt();
  thread ia2 = thread(intake_arm);
  MoveToPoint(8, -76, 1, 3000);
  wait(1500, msec);
  ia2.interrupt();
  intake_arm_outtake();
  thread ir2 = thread(intake_color_red);
  for(int i = 0; i < 1; i++) {
    correct_angle = NormalizeTarget(160);
    DriveTo(1000, 1000, false);
    TurnToAngle(-90, 800, false);
    correct_angle = NormalizeTarget(180);
    DriveTo(-8, 1500, false);
  }
  correct_angle = NormalizeTarget(160);
  DriveTo(1000, 1000, false);
  TurnToAngle(-80, 800, false);
  correct_angle = NormalizeTarget(-45);
  DriveTo(-1000, 1500, false, 8);
  Stop(hold);
  xpos = 0;
  ypos = 0;
  ir2.interrupt();
  intake(-12);
  PTO.set(true);
  goal_clamp.set(false);
  wait(400, msec);
  MoveToPoint(-22, 15, 1, 3000, false, 8);
  MoveToPoint(-43, 0, 1, 3000, true, 6);
  TurnToAngle(180, 700);
  intake(12);
  PTO.set(false);
  wait(800, msec);
  DriveTo(-3, 800, false);
  TurnToAngle(90, 800, false);
  correct_angle = NormalizeTarget(45);
  DriveTo(-2, 800, false);
  MoveToPoint(-88, -8, -1, 4000, false, 8);
  TurnToAngle(45, 500);
  correct_angle = NormalizeTarget(90);
  DriveTo(4, 1000, false);
  thread ia3 = thread(intake_arm);
  correct_angle = NormalizeTarget(45);
  MoveToPoint(-90, 20, 1, 3000, false, 8);
  MoveToPoint(-83, 39, 1, 3000, true, 4);
  wait(200, msec);
  ia3.interrupt();
  intake_arm_outtake();
  intake(-12);
  PTO.set(true);
  TurnToAngle(-90, 700);
  DriveTo(1000, 1500, false, 4);
  intake(12);
  PTO.set(false);
  wait(800, msec);
}

void skills2(){
  xpos=0;
  ypos=0;
  inertial_sensor.setRotation(180, degrees);

  // score preload
  goal_clamp.set(true);
  MoveToPoint(xpos,ypos+2,-1,1000,false,3);
  intake(12);
  task::sleep(500);
  intake_stop();
  Stop(hold);
  task::sleep(100);
  xpos=0;
  ypos=0;
  TurnToAngle(180,1000,true,10);
  MoveToPoint(xpos,ypos-9,1,2000,true,4);

  // fill the right mogo with 6 rings
  TurnToAngle(90,1000,true,8);
  MoveToPoint(xpos-20,ypos,-1,3000,false,4);
  goal_clamp.set(false);
  Stop(hold);
  task::sleep(100);
  intake(12);
  TurnToAngle(170,1000,true,8);
  boomerang(xpos-35,ypos-33,254,0.9,5000,1,true,12); // ring 1+2
  task::sleep(100);
  TurnToAngle(30,3000,true,4);
  MoveToPoint(xpos+5,ypos+35,1,3000,false,8); // ring 3+4
  MoveToPoint(xpos,ypos+8,1,1000,true,2); // ring 5
  task::sleep(500);
  TurnToAngle(-45,1000,true,12);
  MoveToPoint(xpos+10,ypos-7,-1,2000,true,12);
  TurnToAngle(-90,1000,true,10);
  MoveToPoint(xpos-15,ypos,1,1000,true,10); // ring 6
  task::sleep(500);
  TurnToAngle(180,1000,true,10);
  MoveToPoint(xpos-7,ypos+11,-1,1000,true,7);
  task::sleep(200);
  goal_clamp.set(true); // stuff into corner
  intake_stop();
  task::sleep(800);
  MoveToPoint(xpos+7,ypos-7,1,1000,true,10);
  TurnToAngle(-90,1000,true,8);
  MoveToPoint(xpos+43,ypos,-1,5000,true,10);
  MoveToPoint(xpos+5,ypos,-1,2000,false,3);
  goal_clamp.set(false);
  Stop(hold);

}

void test(){
  DriveTo(60, 3000);
  TurnToAngle(90, 2000);
  TurnToAngle(135, 2000);
  TurnToAngle(150, 2000);
  TurnToAngle(160, 2000);
  TurnToAngle(165, 2000);
  TurnToAngle(0, 2000);
  DriveTo(-60, 3000);
  /*
  intake_arm();
  wait(1000, msec);
  intake_arm_outtake();
  */
}

void TestDriveMotors() {
  Brain.Screen.clearScreen();
  Brain.Screen.print("Left Motor 1 ");
  left_chassis1.spin(fwd, 12, voltageUnits::volt);
  wait(3000, msec);
  Brain.Screen.print(left_chassis1.velocity(rpm));
  wait(1000, msec);
  left_chassis1.stop(coast);
  wait(1000, msec);
  Brain.Screen.newLine();
  Brain.Screen.print("Left Motor 2 ");
  left_chassis2.spin(fwd, 12, voltageUnits::volt);
  wait(3000, msec);
  Brain.Screen.print(left_chassis2.velocity(rpm));
  wait(1000, msec);
  left_chassis2.stop(coast);
  wait(1000, msec);
  Brain.Screen.newLine();
  Brain.Screen.print("Left Motor 3 ");
  left_chassis3.spin(fwd, 12, voltageUnits::volt);
  wait(3000, msec);
  Brain.Screen.print(left_chassis3.velocity(rpm));
  wait(1000, msec);
  left_chassis3.stop(coast);
  wait(1000, msec);
  Brain.Screen.newLine();
  Brain.Screen.print("Right Motor 1 ");
  right_chassis1.spin(fwd, 12, voltageUnits::volt);
  wait(3000, msec);
  Brain.Screen.print(right_chassis1.velocity(rpm));
  wait(1000, msec);
  right_chassis1.stop(coast);
  wait(1000, msec);
  Brain.Screen.newLine();
  Brain.Screen.print("Right Motor 2 ");
  right_chassis2.spin(fwd, 12, voltageUnits::volt);
  wait(3000, msec);
  Brain.Screen.print(right_chassis2.velocity(rpm));
  wait(1000, msec);
  right_chassis2.stop(coast);
  wait(1000, msec);
  Brain.Screen.newLine();
  Brain.Screen.print("Right Motor 3 ");
  right_chassis3.spin(fwd, 12, voltageUnits::volt);
  wait(3000, msec);
  Brain.Screen.print(right_chassis3.velocity(rpm));
  wait(1000, msec);
  right_chassis3.stop(coast);
  wait(1000, msec);
}

void friction_test() {
  long long sum = 0;
  //left motor 1
  Brain.Screen.clearScreen();
  left_chassis1.spin(fwd, 12, volt);
  wait(5000, msec);
  for(int i = 0; i < 1000; i++) {
    sum += left_chassis1.velocity(rpm);
    wait(10, msec);
  }
  Brain.Screen.print(sum / 1000.0);
  Brain.Screen.newLine();
  left_chassis1.stop(hold);
  wait(1000, msec);
  left_chassis1.stop(coast);
  //left motor 2
  sum = 0;
  left_chassis2.spin(fwd, 12, volt);
  wait(5000, msec);
  for(int i = 0; i < 1000; i++) {
    sum += left_chassis2.velocity(rpm);
    wait(10, msec);
  }
  Brain.Screen.print(sum / 1000.0);
  Brain.Screen.newLine();
  left_chassis2.stop(hold);
  wait(1000, msec);
  left_chassis2.stop(coast);
  //left motor 3
  sum = 0;
  left_chassis3.spin(fwd, 12, volt);
  wait(5000, msec);
  for(int i = 0; i < 1000; i++) {
    sum += left_chassis3.velocity(rpm);
    wait(10, msec);
  }
  Brain.Screen.print(sum / 1000.0);
  Brain.Screen.newLine();
  left_chassis3.stop(hold);
  wait(1000, msec);
  left_chassis3.stop(coast);
  //right motor 1
  sum = 0;
  right_chassis1.spin(fwd, 12, volt);
  wait(5000, msec);
  for(int i = 0; i < 1000; i++) {
    sum += right_chassis1.velocity(rpm);
    wait(10, msec);
  }
  Brain.Screen.print(sum / 1000.0);
  Brain.Screen.newLine();
  right_chassis1.stop(hold);
  wait(1000, msec);
  right_chassis1.stop(coast);
  //right motor 2
  sum = 0;
  right_chassis2.spin(fwd, 12, volt);
  wait(5000, msec);
  for(int i = 0; i < 1000; i++) {
    sum += right_chassis2.velocity(rpm);
    wait(10, msec);
  }
  Brain.Screen.print(sum / 1000.0);
  Brain.Screen.newLine();
  right_chassis2.stop(hold);
  wait(1000, msec);
  right_chassis2.stop(coast);
  //right motor 3
  sum = 0;
  right_chassis3.spin(fwd, 12, volt);
  wait(5000, msec);
  for(int i = 0; i < 1000; i++) {
    sum += right_chassis3.velocity(rpm);
    wait(10, msec);
  }
  Brain.Screen.print(sum / 1000.0);
  Brain.Screen.newLine();
  right_chassis3.stop(hold);
  wait(1000, msec);
  right_chassis3.stop(coast);
}