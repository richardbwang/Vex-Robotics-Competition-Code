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

int numrings = 1;
void intake_thread(){
  intake(12);
  Optical.setLight(ledState::on);
  /*
  bool found_color = false;
  while(!found_color) {
    intake(12);
    while(!Optical.isNearObject()){
      wait(10, msec);
    }
    intake(5);
    while(distance_sensor.objectDistance(mm) > 50){
      if(!detectcolor2(isRed)) {
        found_color = true;
      }
      wait(10, msec);
    }
  }*/
  for(int i = 0; i < numrings - 1; i++) {
    intake(12);
    wait(200, msec);
    while(distance_sensor.objectDistance(mm) > 50){
      wait(10, msec);
    }
  }
  intake(12);
  wait(200, msec);
  while(!Optical.isNearObject()) {
    wait(10, msec);
  }
  intake(5);
  while(distance_sensor.objectDistance(mm) > 50){
    wait(10, msec);
  }
  intake_stop(hold);
}

void intake_th(){
  Optical.setLight(ledState::on);
  Optical.setLightPower(100);
  while(!Optical.isNearObject() and distance_sensor.objectDistance(mm)>50){
    wait(10, msec);
  }
  intake(6);
  wait(10, msec);
  intake_stop(brake);
}

bool detectcolor2(bool isred){
  if (isred == true){
    return (Optical.hue() < 300 and Optical.hue() > 90);
  }
  return (Optical.hue() > 340 or Optical.hue() < 60);
}
void intake_color_red(){
  bool wrong_color=false;
  Optical.setLight(ledState::on);
  Optical.setLightPower(100);
  bool isred=true;
  while(1) {
        Optical.setLightPower(100);
        if (Optical.isNearObject() && detectcolor2(isred)) {
          wrong_color = true;
        }else if (Optical.isNearObject()){
          wrong_color=false;
        }
        if (distance_sensor.objectDistance(distanceUnits::mm) < 50 && wrong_color) {
          Sort.set(true);
          wait(300, msec);
          Sort.set(false);
          wrong_color = false;
        }
    // very important
    wait(10, msec);
  }
}

void intake_color_blue(){
  bool wrong_color=false;
  Optical.setLight(ledState::on);
  Optical.setLightPower(100);
  bool isred=false;
  while(1) {
        Optical.setLightPower(100);
        if (Optical.isNearObject() && detectcolor2(isred)) {
          wrong_color = true;
        }else if (Optical.isNearObject()){
          wrong_color=false;
        }
        if (distance_sensor.objectDistance(distanceUnits::mm) < 50 && wrong_color) {
          Sort.set(true);
          //wait(300, msec);
          //Sort.set(false);
          //wrong_color = false;
        }
    // very important
    wait(10, msec);
  }
}

void mogo_release(){
  wait(14500, msec);
  mogo_mech.set(false);
}

void intake_arm_without_outake() {
  intake(12);
  while(true) {
    if(Optical.isNearObject()) {
      break;
    }
    wait(10, msec);
  }
  intake(6);
  while(true) {
    if(distance_sensor.objectDistance(distanceUnits::mm) < 50) {
      break;
    }
    wait(10, msec);
  }
  intake(4);
  wait(80, msec);
  intake_stop(hold);
} 

void r1095r(){
  xpos=0;
  ypos=0;
  InertialA.setRotation(0, degrees);
  MoveToPoint(xpos,ypos-25,-1,4000,true,4);
  wait(100, msec);
  ChassisControl(-2,-2);
  mogo_mech.set(true);
  Stop();
  wait(300, msec);
  TurnToAngle(90,1000,true,12);
  intake(12);
  MoveToPoint(xpos+15,ypos,1,1000,true,12);
  wait(1000, msec);
  TurnToAngle(-90,1000,true,12);
  MoveToPoint(xpos-15,ypos,1,1000,true,12);
  mogo_mech.set(false);
  MoveToPoint(xpos-10,ypos,1,1000,false,3);
  wait(2000, msec);
  intake_stop();
  Stop();

}

void AwpStake(bool isright){
  // start
  InertialA.setRotation(0, degrees);
  xpos=0,ypos=0;
  dirchangestart = false;
  dirchangeend = false;
  clipper.set(true);
  arm_motor.setVelocity(100,rpm);
  arm_motor.spinFor(200,degrees,false);
  intakeraise.set(true);
  thread intakethread=thread(intake_th);
  intake(12);
  MoveToPoint(5, 10, 1, 1500, true, 8);
  intakeraise.set(false);
  MoveToPoint(2.5, 6, -1, 2500, true, 6);
  TurnToAngle(-60,1000,true,8);
  mogo_mech.set(true);
  MoveToPoint(0, 7.5, 1, 1000, true, 5);
  clipper.set(false);
  arm_motor.spinFor(420,degrees,false);
  task::sleep(1000);
  arm_motor.spinFor(-400,degrees,false);
  MoveToPoint(19,-1.5,-1,3000,false,8);
  MoveToPoint(29,-7,-1,3000,true,4);
  mogo_mech.set(false);
  task::sleep(100);
  TurnToAngle(180,1000,true,12);
  intake(12);
  MoveToPoint(32,-32,1,1000,false,10);
  Stop();
  task::sleep(200);
  TurnToAngle(-130,1000,true,10);
  MoveToPoint(-4,-47,1,2000,false,10);
  Stop();
  task::sleep(1000);
  MoveToPoint(5,-38,-1,1000,true,10);
  TurnToAngle(45,1000,true,10);
  intake(0);
  mogo_mech.set(true);
  MoveToPoint(32,10,1,1000,false,12);
  MoveToPoint(32,15,1,1000,false,4);
  







  // MoveToPoint((isright) ? 5.5 :-5.5 ,11,1,2000,true,4);

  // arm_motor.spinFor(fwd, -230+arm_motor.position(degrees), degrees, 100, rpm, true);
  // MoveToPoint((isright) ? -7 : 7,-5,-1,2000,true,8);
  // task::sleep(200);
  // arm_motor.spinFor(reverse, arm_motor.position(degrees)+30, degrees, 100, rpm, true);
  // arm_motor.stop(coast);
  // intakeraise.set(true);
  // TurnToAngle((isright) ? 90:-90,1000,true,8);
  // intake(12);
  // MoveToPoint((isright) ? 5 : -5,-2,1,2000,true,6);

  // task::sleep(300);
  // intakeraise.set(false);
  // MoveToPoint((isright) ? -9 : 9,-4,-1,2000,true,8);
  // TurnToAngle(0,1000,true,10);
  // MoveToPoint((isright) ? -11 : 11,-25,-1,4000,true, 4);
  // ChassisControl(-2,-2);
  // mogo_mech.set(false);  
  // Stop();
  // intakethread.interrupt();
  // task::sleep(300);
  // TurnToAngle((isright) ? -90:90,1000,true,8);
  // intake(12);
  // MoveToPoint((isright) ? -30 : 30 ,-25,1,2000,true,10);
  // task::sleep(1000);
  // TurnToAngle((isright) ? 90:-90,1000,true,8);
  // task::sleep(500);
  // mogo_mech.set(true);
  // task::sleep(10);
  // MoveToPoint((isright) ? -3 : 3,-25,1,2000,true,12);
  // MoveToPoint((isright) ? 4 : -4,-25,1,3000,false,3);
  // intake_stop();
}

void SigSoloAWP() {
  dirchangestart = false;
  dirchangeend = false;
  mogo_mech.set(true);
  if(isRed) {
    //thread ir = thread(intake_color_red);
  } else {
    thread ib = thread(intake_color_blue);
  }
  intakeraise.set(true);
  clipper.set(true);
  arm_angle_target = 200;
  thread at = thread(arm_thread);
  thread it = thread(intake_thread);
  MoveToPoint(-5.5, 10, 1, 2000, true, 6);
  intakeraise.set(false);
  MoveToPoint(-2, 7, -1, 1500, true, 3);
  TurnToAngle(55, 800);
  at.interrupt();
  arm(12);
  DriveTo(1.25, 700);
  Stop(coast);
  wait(600, msec);
  DriveTo(-3, 800, false);
  clipper.set(false);
  arm_angle_target = arm_store_target;
  thread at2 = thread(arm_thread);
  MoveToPoint(-16, -6, -1, 2000, false);
  dirchangestart = true;
  dirchangeend = true;
  MoveToPoint(-27, -8, -1, 2000, false, 5);
  dirchangestart = false;
  dirchangeend = false;
  mogo_mech.set(false);
  wait(200, msec);
  TurnToAngle(135, 800, false);
  it.interrupt();
  intake(12);
  MoveToPoint(-34, -28, 1, 2500, true);
  wait_intake();
  wait(200, msec);
  intake(0);
  TurnToAngle(110, 800, false);
  //intake(12);
  MoveToPoint(-16, 0, 1, 3000, false);
  intake(12);
  mogo_mech.set(true);
  numrings = 2;
  thread it2 = thread(intake_thread);
  MoveToPoint(-11, 40, 1, 3000, false, 6);
  /*
  MoveToPoint(-12, 50, 1, 2000, false, 8);
  Swing(-45, 1, 800, false);
  correct_angle = NormalizeTarget(0);
  DriveTo(-6, 1200, false);
  */
  dirchangestart = true;
  dirchangeend = true;
  MoveToPoint(-30, 39, -1, 2500, false, 5);
  dirchangestart = false;
  dirchangeend = false;
  mogo_mech.set(false);
  wait(200, msec);
  TurnToAngle(30, 800, false);
  it2.interrupt();
  intake(12);
  MoveToPoint(-32, 60, 1, 2500);
  wait_intake_thread();
  MoveToPoint(-32, 43, -1, 2500, false);
  intake(0);
  TurnToAngle(-50, 800, false);
  intake(12);
  arm(-12);
  MoveToPoint(-50, 38, 1, 2000);
}

void doink_release() {
  wait(600, msec);
  Doinker.set(true);
}

void arm_reset() {
  arm_motor.setPosition(0, deg);
}

/*
void GoalRush() {
  double start_time = Brain.timer(msec);
  dirchangestart = false;
  dirchangeend = false;
  mogo_mech.set(true);
  if(isRed) {
    thread ir = thread(intake_color_red);
  } else {
    thread ib = thread(intake_color_blue);
  }
  arm_angle_target = 465;
  thread at = thread(arm_thread);
  //thread dr = thread(doink_release);
  clipper.set(true);
  //MoveToPoint(-9, 34, 1, 3000, true);
  MoveToPointEarly(-12, 50.25, 1, 1500, true);
  Controller1.Screen.print(Brain.timer(msec)-start_time);
  //Stop(hold);
  //wait(300, msec);

  //Doinker.set(true);
  at.interrupt();
  arm(12);
  //wait(100, msec);
  correct_angle = NormalizeTarget(10);
  DriveTo(-12, 1500, false);
  arm(-12);
  clipper.set(false);
  TurnToAngle(45, 800, false);
  MoveToPoint(-30, 20, -1, 2000, false);
  ChassisControl(-3, -3);
  wait(500, msec);
  mogo_mech.set(false);
  wait(200, msec);
  intake(12);
  correct_angle = NormalizeTarget(135);
  DriveTo(3, 800, false);
  arm_angle_target = 80;
  thread at2 = thread(arm_thread);
  correct_angle = NormalizeTarget(90);
  MoveToPoint(-17, 35, 1, 2000);
  correct_angle = NormalizeTarget(10);
  DriveTo(-4, 800, false);
  TurnToAngle(-110, 800, false);
  MoveToPoint(-30, 0, 1, 2000, false);
  TurnToAngle(140, 800, false);
  correct_angle = NormalizeTarget(120);
  DriveTo(1000, 2000, false, 8);
  correct_angle = NormalizeTarget(135);
  DriveTo(-3, 800, false);
  intakeraise.set(true);
  DriveTo(1000, 1000, false, 8);
  DriveTo(-8, 1200, false);
  TurnToAngle(-20, 800, false);
  mogo_mech.set(true);
  correct_angle = NormalizeTarget(-45);
  DriveTo(-1000, 600, false);
  Swing(-60, 800, false);
  arm_pid_target = 42;
  thread apl = thread(arm_pid_loop);
  intakeraise.set(true);
  MoveToPoint(-55, 0, 1, 2000);
  MoveToPoint(-52, -2, -1, 2000);
  TurnToAngle(-135, 800);
  clipper.set(true);
  intake(-12);
  wait(100, msec);
  intake_stop();
  wait(100, msec);
  Stop(coast);
  arm(-12);
}
*/

void SetupGoalRush() {
  //TurnToAngleSetup(-rushsetupangle);
  TurnToAngleSetup(rushsetupangle);
}

void GoalRushStraight() {
  mogo_mech.set(true);
  dirchangestart = false;
  dirchangeend = false;
  correct_angle = NormalizeTarget(-rushsetupangle);
  arm_motor.setPosition(0, deg);
  arm_angle_target = 1500;
  thread at = thread(arm_thread);
  DriveTo(22, 1500, false);
  DriveTo(-20, 2500, false);
  at.interrupt();
  arm_pid_target = arm_store_target - 20;
  thread apl = thread(arm_pid_loop);
  //arm(-12);
  MoveToPoint(-20, 18, -1, 2000, false);
  arm(0);
  dirchangestart = true;
  dirchangeend = true;
  MoveToPoint(-30, 20, -1, 2000, false, 4);
  mogo_mech.set(false);
  wait(200, msec);
  intake(12);
  correct_angle = NormalizeTarget(90);
  DriveTo(1, 800, false);
  MoveToPoint(-24, 34, 1, 2000, true, 8);
  wait_intake_thread();
  correct_angle = NormalizeTarget(10);
  DriveTo(-7, 1000, false);
  TurnToAngle(80, 800, false);
  MoveToPoint(-3, 6, 1, 2000, false);
  apl.interrupt();
  arm_motor.stop(hold);
  intake(12);
  MoveToPoint(5, -12, 1, 1500);
  correct_angle = NormalizeTarget(135);
  DriveTo(2, 800, false);
  wait_intake_thread();
  DriveTo(-4, 1000, false);
  Stop(hold);
  /*
  intakeraise.set(true);
  wait(300, msec);
  DriveTo(5, 1000, false);
  wait_intake_thread();
  DriveTo(-4, 1000, false);
  Stop(hold);
  intake(-12);
  wait(200, msec);
  intake(12);
  wait(700, msec);
  */
  Doinker.set(true);
  wait(1000, msec);
  intake(0);
  DriveTo(7, 1000, false);
  TurnToAngle(0, 800, false);
  TurnToAngle(-45, 500);
  mogo_mech.set(true);
  Doinker.set(false);
  wait(200, msec);
  MoveToPoint(-30, 30, 1, 2000);
  Stop(hold);
}

void skills() {
  mogo_mech.set(true);
  arm(12);
  DriveTo(-2, 800);
  wait(1000, msec);
  arm_motor.stop(coast);
  DriveTo(-3, 800, false);
  arm(-12);
  TurnToAngle(60, 800, false);
  MoveToPoint(-20, -8, -1, 2500, false, 4);
  mogo_mech.set(false);
  wait(200, msec);
  intake(12);
  MoveToPoint(-25, -35, 1, 2000, false);
  TurnToAngle(-140, 800, false);
  correct_angle = NormalizeTarget(-135);
  DriveTo(10, 1500, false);
  MoveToPoint(-46, -86, 1, 2500, true);
  wait_intake();
  thread al = thread(arm_load);
  MoveToPoint(-46, -100, 1, 2500, true);
  wait_intake();
  wait(800, msec);
  TurnToAngle(-60, 800, false);
  al.interrupt();
  arm_pid_target = arm_store_target;
  thread apl2 = thread(arm_pid_loop);
  intake(12);
  MoveToPoint(-54, -59.5, 1, 2500, true, 8);
  TurnToAngle(-90, 800);
  apl2.interrupt();
  arm_angle_target = 390;
  thread att = thread(arm_thread);
  DriveTo(1000, 800, false, 6);
  Stop(coast);
  wait(300, msec);
  att.interrupt();
  arm(0);
  DriveTo(-5, 1000, false);
  clipper.set(false);
  arm_pid_target = arm_store_target;
  thread apll = thread(arm_pid_loop);
  TurnToAngle(-60, 800, false);
  MoveToPoint(-42, 1, 1, 3000, true, 5);
  wait_intake();
  TurnToAngle(-90, 800, false);
  correct_angle = NormalizeTarget(135);
  DriveTo(-1000, 1500, false, 8);
  mogo_mech.set(true);
  DriveTo(3, 800, false);
  thread it = thread(intake_thread);
  TurnToAngle(180, 800, false);
  MoveToPoint(-52, -6, 1, 2000, false);
  correct_angle = NormalizeTarget(-170);
  DriveTo(7, 1500, false, 8);
  Swing(-70, 1, 1000, false);
  Stop();
  apll.interrupt();
  arm(-12);
  wait_intake_thread();

  //second quarter
  MoveToPoint(3, -8.5, -1, 2500, false);
  MoveToPoint(24, -8.5, -1, 2500, false, 5);
  mogo_mech.set(false);
  wait(200, msec);
  it.interrupt();
  intake(12);
  MoveToPoint(26, -34, 1, 2000, false, 8);
  MoveToPoint(47, -34, 1, 2000, false, 8);
  /*
  MoveToPoint(-5, -60, 1, 2000, true);
  wait_intake();
  thread it2 = thread(intake_thread);
  MoveToPoint(-30, -86, 1, 2500, true);
  wait_intake();
  it2.interrupt();
  intake(0);
  MoveToPoint(20, -34, -1, 2500, false);
  */
  arm_motor.stop(coast);
  MoveToPoint(50, -14, 1, 2500, false, 8);
  MoveToPoint(52, 3, 1, 2500, true, 6);
  wait_intake();
  correct_angle = NormalizeTarget(60);
  DriveTo(-8, 1000, false);
  thread al2 = thread(arm_load);
  MoveToPoint(63, -11, 1, 2000, true);
  wait_intake_thread();
  TurnToAngle(180, 800, false);
  correct_angle = NormalizeTarget(-135);
  DriveTo(-1000, 1500, false, 8);
  mogo_mech.set(true);
  DriveTo(3, 800, false);
  al2.interrupt();
  arm_pid_target = arm_score_target;
  thread apl4 = thread(arm_pid_loop);
  intake(0);
  MoveToPoint(59, -66, 1, 2000, true, 8);
  MoveToPoint(52, -56.5, -1, 2000, true, 8);
  TurnToAngle(90, 700);
  DriveTo(1000, 800, false, 6);
  Stop(coast);
  apl4.interrupt();
  arm_angle_target = 390;
  arm_thread();
  DriveTo(-3, 800, false);
  clipper.set(false);
  arm_pid_target = arm_store_target;
  thread apl6 = thread(arm_pid_loop);
  TurnToAngle(180, 800, false);
  thread itt = thread(intake_thread);
  MoveToPoint(24, -80, 1, 2000, false);
  TurnToAngle(180, 800, false);
  TurnToAngle(90, 800, false);
  MoveToPoint(1, -105, -1, 3000, false, 4);
  mogo_mech.set(false);
  wait(200, msec);
  itt.interrupt();
  intake(12);
  MoveToPoint(48, -86, 1, 2000, false);
  MoveToPoint(64, -79, 1, 2000, true, 8);
  wait_intake();
  TurnToAngle(135, 800, false);
  apl6.interrupt();
  thread al3 = thread(arm_load);
  MoveToPoint(61, -105, 1, 2500, true);
  wait_intake();
  wait(800, msec);
  al3.interrupt();
  arm_pid_target = arm_score_target;
  thread apl7 = thread(arm_pid_loop);
  intake(12);
  TurnToAngle(-150, 800, false);
  MoveToPoint(51, -104, 1, 1400, false, 10);
  MoveToPoint(51, -120, 1, 1600, false);
  Stop(hold);
  Doinker.set(true);
  wait_intake();
  TurnToAngle(45, 800, false, 8);
  correct_angle = NormalizeTarget(-45);
  Doinker.set(false);
  DriveTo(3, 800, false);
  Stop(hold);
  mogo_mech.set(true);
  wait(300, msec);
  DriveTo(-1000, 1500, false);
  //mogo_mech.set(true);
  wait(200, msec);
  DriveTo(5, 1000, false, 6);
  intake_motor.stop(coast);
  MoveToPoint(10, -116, 1, 2500, true, 8);
  TurnToAngle(180, 800);
  Stop(coast);
  apl7.interrupt();
  arm(12);
  wait(800, msec);
  DriveTo(-2, 800, false);
  clipper.set(false);
  arm(-12);
  TurnToAngle(100, 800, false);
  correct_angle = NormalizeTarget(30);
  DriveTo(-4, 800, false);
  correct_angle = NormalizeTarget(70);
  DriveTo(-1000, 2500, false);
}

void NegativeElim() {
  mogo_mech.set(true);
  DriveTo(-10, 1500, false);
  MoveToPoint(-4, -30, -1, 2000, false, 6);
  mogo_mech.set(false);
  wait(200, msec);
  TurnToAngle(80, 800, false);
  intake(12);
  MoveToPoint(4, -49, 1, 2000, false);
  MoveToPoint(26, -49, 1, 3000, false, 4);
  correct_angle = NormalizeTarget(95);
  DriveTo(-10, 1500, false);
  MoveToPoint(-5, -31, -1, 2000, false);
  MoveToPoint(15, -31, 1, 2000, false);
  MoveToPoint(12, -18, -1, 2000, false);
  TurnToAngle(90, 800, false);
  correct_angle = NormalizeTarget(45);
  DriveTo(1000, 1500, false, 6);
  DriveTo(-4, 1000, false, 6);
  intakeraise.set(true);
  DriveTo(1000, 1000, false, 6);
  wait_intake();
  correct_angle = NormalizeTarget(60);
  DriveTo(-5, 1000, false);
  TurnToAngle(-30, 800, false);
  arm_pid_target = arm_load_target;
  thread apl6 = thread(arm_pid_loop);
  MoveToPoint(-28, -10, 1, 2500, true);
  MoveToPoint(-18, -7, -1, 1500, true);
  TurnToAngle(-35, 600);
  arm(12);
  DriveTo(3, 800);
  wait(500, msec);
  arm_motor.stop(coast);
  DriveTo(-3, 800);
  arm(-12);
  wait(1000, msec);
  arm_motor.stop(coast);
}

void testturn() {
  //MoveToPoint(0, 90, 1, 5000, false, 4);
  DriveTo(75, 5000, false, 4);
  Stop(coast);
  //MoveToPoint(90, 90, 1, 3000, true, 8);
  //MoveToPoint(90, 0, 1, 3000, true, 8);
  wait(1000, msec);
  //MoveToPoint(0, 0, -1, 5000, true, 4);
  DriveTo(-78, 5000, true, 4);
  TurnToAngle(0, 1000);
  Controller1.Screen.print("X: %0.1f Y: %0.1f arm: %d", xpos, ypos);
}