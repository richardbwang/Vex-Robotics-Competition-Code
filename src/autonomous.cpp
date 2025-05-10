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

void color_sort(){
  Optical.setLight(ledState::on);
  Optical.setLightPower(100);
  while(true) {
    if (Optical.isNearObject() && Optical.color() == (isRed ? color(blue) : color(red))) {
      sorting = true;
      intake(10);
      while(distance_sensor.objectDistance(mm) > 30) {
        wait(10, msec);
      } 
      while(distance_sensor.objectDistance(mm) < 40) {
        wait(10, msec);
      } 
      intake(-12);
      wait(150, msec);  
      intake(12);
      sorting = false;
    }
    // very important
    wait(10, msec);
  }
}

/*
void intakeStuck() {
  intake(-12);
  wait(40, msec);
  intake(12);
}
*/

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
    while(!Optical.isNearObject()/*distance_sensor.objectDistance(mm) > 50*/){
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
  targetIntakeVolts = 0;
}

void intake_th(){
  Optical.setLight(ledState::on);
  Optical.setLightPower(100);
  while(!Optical.isNearObject() and distance_sensor.objectDistance(mm) > 50){
    wait(10, msec);
  }
  intake_stop(hold);
  targetIntakeVolts = 0;
}

bool detectcolor2(bool isred){
  if (isred == true){
    return (Optical.hue() < 300 and Optical.hue() > 90);
  }
  return (Optical.hue() > 340 or Optical.hue() < 60);
}

void intake_color_red(){
  //bool wrong_color=false;
  Optical.setLight(ledState::on);
  Optical.setLightPower(100);
  while(1) {
        Optical.setLightPower(100);
        if (Optical.isNearObject() && Optical.color() == color(blue)) {
          sorting = true;
          wait(40, msec);
          intake(0);
          wait(400, msec);
          sorting = false;
          intake(12);
        }
    // very important
    wait(10, msec);
  }
}

void intake_color_blue(){
  bool wrong_color=false;
  Optical.setLight(ledState::on);
  Optical.setLightPower(100);
  while(1) {
        Optical.setLightPower(100);
        if (Optical.isNearObject() && Optical.color() == color(red)) {
          sorting = true;
          wait(40, msec);
          intake(0);
          wait(400, msec);
          sorting = false;
          intake(12);
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

// void AwpStake(bool isright){
//   // start
//   InertialA.setRotation(0, degrees);
//   xpos=0,ypos=0;
//   dirchangestart = false;
//   dirchangeend = false;
//   clipper.set(true);
//   arm_motor.setVelocity(100,rpm);
//   arm_motor.spinFor(260,degrees,false);
//   intakeraise.set(true);
//   thread intakethread=thread(intake_th);
//   intake(12);
//   MoveToPoint(5, 10, 1, 1500, true, 8);
//   intakeraise.set(false);
//   MoveToPoint(2.5, 6, -1, 2500, true, 6);
//   TurnToAngle(-60,1000,true,8);
//   mogo_mech.set(true);
//   MoveToObject(aiVisionArmRed,redStakeColor,160,82,1,1000,true,6);
//   // MoveToPoint(0, 7.5, 1, 1000, true, 5);
//   clipper.set(false);
//   arm(12);
//   arm_motor.setStopping(coast);
//   task::sleep(600);
//   arm_motor.stop();
//   MoveToPoint(19,-1.5,-1,3000,false,8);
//   arm_motor.spinFor(-400,degrees,false);
//   MoveToPoint(29,-7,-1,3000,true,4);
//   mogo_mech.set(false);
//   task::sleep(100);
//   TurnToAngle(180,1000,true,12);
//   intake(12);
//   MoveToPoint(32,-32,1,1000,false,10);
//   Stop();
//   task::sleep(200);
//   TurnToAngle(-130,1000,true,10);
//   MoveToPoint(-4,-47,1,2000,false,10);
//   Stop();
//   task::sleep(1000);
//   MoveToPoint(5,-38,-1,1000,true,10);
//   TurnToAngle(45,1000,true,10);
//   intake(0);
//   set(true);



  // MoveToPoint(32,10,1,1000,false,12);
  // MoveToPoint(32,15,1,1000,false,4);
  







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
// }

void dropmogo(){
  while(distance_sensor.objectDistance(mm)>50){
    wait(10, msec);
  }
  wait(200, msec);
  mogo_mech.set(false);
}

void intakewait(){
  task::sleep(300);
  intake(12);
}


void SigSoloAWP() {
  thread is = thread(intakeStuck);
  correct_angle = isRed ? InertialA.rotation() : -InertialA.rotation();
  double start_time = Brain.timer(msec);
  // 10.92+1.625-7.5
  // 3.04+7.1875-8.5
  thread colorSort = thread(color_sort);
  arm_motor.setPosition(arm_load_target - 7, deg);
  dirchangestart = false;
  dirchangeend = false;
  arm_angle_target = 600;
  thread arm_loop2 = thread(arm_thread);
  wait(500, msec);
  DriveTo(-3, 800, false);
  arm_loop2.interrupt();
  arm_pid_target = arm_store_target;
  thread arm_loop = thread(arm_pid_loop);
  dirchangestart = true;
  dirchangeend = true;
  MoveToPoint(-16, (isRed ? -11 : -11), -1, 2000, false, 10);
  MoveToPoint(-31, (isRed ? -10 : -10), -1, 2000, true, 4);
  //ChassisControl(-0.5, -0.5);
  dirchangestart = false;
  dirchangeend = false;
  mogo_mech.set(true);
  task::sleep(100);
  TurnToAngle(160, 800, false);
  intake(12);
  MoveToPoint(-46.5, (isRed ? -24 : -22), 1, 1500,true, 8);
  wait(400, msec);
  MoveToPoint(-32, -10, -1,1000, true, 8);
  MoveToPoint(-32, -30, 1, 2500, true, 8);
  thread ittt = thread(intake_th); 
  wait(100, msec);
  TurnToAngle(110, 800, false);
  // intakeraise.set(true);
  MoveToPoint(-14, -4, 1, 3000, false, 12);
  ittt.interrupt();
  intake(11);
  // mogo_mech.set(false);
  MoveToPoint(-7, 33, 1, 3000, true, 3);
  wait(200, msec);
  mogo_mech.set(false);
  wait(50, msec);
  intake(12);
  thread it = thread(intake_th);
  DriveTo(4, 1500, false, 6);
  arm_loop.interrupt();
  arm(-12);
  TurnToAngle(30, 800, false);
  MoveToPoint(-28, isRed ? 41 : 39, -1, 2500, true, 5);
  dirchangestart = false;
  dirchangeend = false;
  mogo_mech.set(true);
  wait(100, msec);
  // wait(200, msec);
  TurnToAngle(60, 800, false);
  it.interrupt();
  thread iw = thread(intakewait);
  MoveToPoint(-31, 63, 1, 2500);
  dirchangestart = true;
  dirchangeend = true;
  MoveToPoint(-31, 45, -1, 2500, false, 10);
  dirchangestart = false;
  dirchangeend = false;
  intake(0);
  TurnToAngle(-50, 800, false);
  intake(12);
  MoveToPoint(-49, 36, 1, 2000);
  Controller1.Screen.print(Brain.timer(msec) - start_time);
}

void NegativeAWP() {
  double start_time = Brain.timer(msec);
  thread cs = thread(color_sort);
  dirchangestart = true;
  dirchangeend = true;
  //InertialA.setRotation((isRed) ? 33 : -33, degrees);
  arm_motor.setPosition(arm_load_target - 7, deg);
  arm_angle_target = 600;
  thread arm_loop2 = thread(arm_thread);
  wait(400, msec);
  MoveToPoint(-19, (isRed ? -11 : -10), -1, 2000, false, 12);
  arm_loop2.interrupt();
  arm_pid_target = arm_store_target;
  thread apl = thread(arm_pid_loop);
  MoveToPoint(-30, (isRed ? -11 : -10), -1, 2000, true, 4);
  mogo_mech.set(true);
  task::sleep(100);
  TurnToAngle(160, 800, false);
  intake(12);
  MoveToPoint(-46, -22, 1, 2000, false, 5);
  MoveToPoint(-48, -39, 1, 1500, false, 4);
  MoveToPoint(-36, -11, -1, 2000, true);
  MoveToPoint(-31, -29, 1, 2000, false, 8);
  apl.interrupt();
  arm_pid_target = 580;
  thread apl2 = thread(arm_pid_loop);
  correct_angle = NormalizeTarget(180);
  DriveTo(-3, 800, false, 8);
  MoveToPoint(2, -41, 1, 2500, false);
  correct_angle = NormalizeTarget(135);
  intake(12);
  thread it1 = thread(intake_th);
  DriveTo(1000, 600, false, 4);
  Stop(coast);
  wait(400, msec);
  DriveTo(-5, 1200, true, 2);
  it1.interrupt();
  //intakeStuck();
  wait(500, msec);
  thread it2 = thread(intake_th);
  DriveTo(1000, 600, false, 4);
  Stop(coast);
  wait(400, msec);
  DriveTo(-3, 1000, true, 2);
  it2.interrupt();
  //intakeStuck();
  wait(500, msec);
  thread it = thread(intake_th);
  DriveTo(1000, 600, false, 4);
  Stop(coast);
  wait(500, msec);
  if(distance_sensor.objectDistance(mm) > 50) {
    intake(8);
  }
  apl2.interrupt();
  arm(-12);
  DriveTo(-4, 800, false, 12);
  TurnToAngle(60, 800, false);
  it.interrupt();
  //thread is = thread(intakeStuck);
  apl2.interrupt();
  MoveToPoint(-28, 7, 1, 2000, false, 8);
  Controller1.Screen.print(Brain.timer(msec) - start_time);
}

void doink_release() {
  wait(600, msec);
  Doinker.set(true);
}

void arm_reset() {
  arm_motor.setPosition(0, deg);
}

void rush_clamp() {
  while(clamp_distance.objectDistance(mm) > 70) {
    wait(10, msec);
  }
  intakeraise.set(true);
  Doinker.set(true);
}

void GoalRush() {
  thread is = thread(intakeStuck);
  dkp -= 0.2;
  xpos = (isRed ? 0 : 25.375);
  arm_motor.setPosition(arm_load_target - 7, deg);
  InertialA.setRotation(rushsetupangle, deg);
  correct_angle = NormalizeTarget(isRed ? rushsetupangle: -rushsetupangle);
  arm_pid_target = arm_store_target;
  thread al = thread(arm_pid_loop);
  thread cs = thread(color_sort);
  thread rc = thread(rush_clamp);
  intake(12);
  thread it = thread(intake_th);
  Doinker.set(true);
  DriveTo(35, 1100, true);
  dkp += 0.2;
  wait(100, msec);
  //DriveTo(29, 1000, false);
  /*
  while(ypos > 12) {
    ChassisControl(-12, -12);
    wait(10, msec);
  }
  */
  MoveToPoint((isRed ? -3 : -21), 10, -1, 15000, false);
  Stop(hold);
  rc.interrupt();
  Doinker.set(true);
  intakeraise.set(false);
  wait(300, msec);
  correct_angle = NormalizeTarget(isRed ? -15 : 15);
  DriveTo(3, 800, true, 6);
  DriveTo(-5, 1000, true);
  Doinker.set(false);
  wait(300, msec);
  if(isRed) {
    TurnToAngle(-90, 800, false);
    MoveToPoint(-4, 28, -1, 2000, true, 5);
    mogo_mech.set(true);
    wait(200, msec);
    intake(12);
    MoveToPoint(-7, 8, 1, 2000, true);
    wait(300, msec);
    intake(-12);
    MoveToPoint(-16, -4, 1, 1500, false);
    /*
    TurnToAngle(-135, 800, false);
    mogo_mech.set(false);
    correct_angle = NormalizeTarget(-90);
    DriveTo(-3, 800, false, 6);
    */
    correct_angle = NormalizeTarget(-125);
    mogo_mech.set(false);
    DriveTo(-15, 1700, true);
    correct_angle = NormalizeTarget(-90);
    DriveTo(5, 1000, false);
    intake(-12);
    TurnToAngle(180, 800, false);
    intake(0);
    MoveToPoint(-34, 24, -1, 2000, false, 5);
    mogo_mech.set(true);
    al.interrupt();
    arm_pid_target = arm_store_target + 300;
    thread al5 = thread(arm_pid_loop);
    MoveToPoint(-5, -6, 1, 2000, false, 10);
    intake(12);
    correct_angle = NormalizeTarget(135);
    targetIntakeVolts = 0;
    DriveTo(1000, 800, false, 4);
    targetIntakeVolts = 12;
    Stop(coast);
    wait(300, msec);
    if(isRed) {
      int i = 0;
      while(InertialA.rotation() > NormalizeTarget(30) && i < 100) {
        ChassisControl(-5, 9);
        wait(10, msec);
        i++;
      }
    }
    correct_angle = NormalizeTarget(150);
    DriveTo(-14, 1700, false);
    al5.interrupt();
    arm_pid_target = arm_score_target - 80;
    thread al2 = thread(arm_pid_loop);
    /*
    //Doinker.set(true);
    DriveTo(-8, 1000, false);
    DriveTo(4, 1200, true, 4);
    wait(800, msec);
    intake(0);
    correct_angle = NormalizeTarget(145);
    DriveTo(1000, 500, false, 4);
    TurnToAngle(90, 800, false);
    //Doinker.set(false);
    */
    TurnToAngle(60, 800, false);
    MoveToPoint(3, 37, 1, 2000, true, 8);
    TurnToAngle(42, 700);
    ChassisControl(1, 1);
    wait(400, msec);
    al2.interrupt();
    arm_motor.spin(fwd, 2, volt);
  } else {
    TurnToAngle(90, 800, false);
    MoveToPoint(-37, 24, -1, 2000, true, 5);
    mogo_mech.set(true);
    wait(200, msec);
    intake(12);
    MoveToPoint(-7, 6, 1, 2000, true);
    intake(-12);
    MoveToPoint(-11, -8, 1, 1500, false);
    /*
    TurnToAngle(-135, 800, false);
    mogo_mech.set(false);
    correct_angle = NormalizeTarget(-90);
    DriveTo(-3, 800, false, 6);
    */
    correct_angle = NormalizeTarget(-135);
    mogo_mech.set(false);
    DriveTo(-13, 1700, true);
    correct_angle = NormalizeTarget(-90);
    DriveTo(5, 1000, false);
    intake(-12);
    TurnToAngle(180, 800, false);
    intake(0);
    MoveToPoint(-24, 30, -1, 2000, false, 5);
    mogo_mech.set(true);
    al.interrupt();
    arm_pid_target = arm_store_target + 300;
    thread al5 = thread(arm_pid_loop);
    MoveToPoint(-5, -6, 1, 2000, false, 10);
    intake(12);
    correct_angle = NormalizeTarget(135);
    targetIntakeVolts = 0;
    DriveTo(1000, 1000, false, 6);
    targetIntakeVolts = 12;
    Stop(coast);
    wait(300, msec);
    /*
    if(!isRed) {
      int i = 0;
      while(InertialA.rotation() < NormalizeTarget(-30) && i < 100) {
        ChassisControl(11, -5);
        wait(10, msec);
        i++;
      }
    }
    */
    correct_angle = NormalizeTarget(120);
    DriveTo(-10, 2000, false);
    al5.interrupt();
    arm_pid_target = arm_score_target - 80;
    thread al2 = thread(arm_pid_loop);
    /*
    //Doinker.set(true);
    DriveTo(-8, 1000, false);
    DriveTo(4, 1200, true, 4);
    wait(800, msec);
    intake(0);
    correct_angle = NormalizeTarget(145);
    DriveTo(1000, 500, false, 4);
    TurnToAngle(90, 800, false);
    //Doinker.set(false);
    */
    TurnToAngle(30, 800, false);
    intake(12);
    MoveToPoint(0.5, 35.5, 1, 2000, true, 8);
    TurnToAngle(41, 700);
    ChassisControl(1.5, 1.5);
    wait(200, msec);
    al2.interrupt();
    arm_motor.spin(fwd, 2, volt);
    wait(400, msec);
  }
  // if(isRed) {
  //   MoveToPoint(-6, -8, -1, 1500, false);
  //   MoveToPoint(-14, -10, -1, 1500, false);
  // } else {
  //   TurnToAngle(80, 800, false);
  // }
  // /*
  // TurnToAngle(30, 800, false);
  // correct_angle = NormalizeTarget(60);
  // DriveTo(-13, 1500, true, 8);
  // correct_angle = NormalizeTarget(90);
  // DriveTo(6, 1200, true, 8);
  // */
  // MoveToPoint(isRed ? -36 : -40, 22, -1, 3000, true, 5);
  // mogo_mech.set(true);
  // wait(200, msec);
  // intake(12);
  // wait(400, msec);
  // mogo_mech.set(false);
  // DriveTo(3, 800, false);
  // if(isRed) {
  //   intake(0);
  //   MoveToPoint(-10, 6, 1, 2000, true);
  // }
  // intake(0);
  // MoveToPoint(isRed ? -7 : -26, 26, -1, 2500, true, 5);
  // mogo_mech.set(true);
  // wait(200, msec);
  // //TurnToAngle(180, 800, false);
  // MoveToPoint(isRed ? -10 : -5, -1, 1, 2000, false);
  // intake(12);
  // correct_angle = NormalizeTarget(isRed ? 120 : 150);
  // DriveTo(1000, 1000, false, 6);
  // Stop(coast);
  // wait(500, msec);
  // DriveTo(-10, 1200, false);
  // TurnToAngle(90, 800, false);
  // MoveToPoint(3, isRed ? 38: 37, 1, 2000, true, 8);
  // TurnToAngle(38, 700);
  // al.interrupt();
  // arm_pid_target = arm_score_target - 80;
  // thread al2 = thread(arm_pid_loop);
  // ChassisControl(1, 1);
  // wait(400, msec);
  // al2.interrupt();
  // arm_motor.spin(fwd, 2, volt);
}

void SetupGoalRush() {
  TurnToAngleSetup(rushsetupangle);
}

void GoalRushStraight() {
  InertialA.setRotation((isRed) ? -rushsetupangle : rushsetupangle, degrees);
  dirchangestart = false;
  dirchangeend = false;
  correct_angle = NormalizeTarget(-rushsetupangle);
  arm_motor.setPosition(arm_load_target - 7, deg);
  arm_angle_target = 500;
  thread at = thread(arm_thread);
  DriveTo(32, 1500, false);
  at.interrupt();
  arm(4);
  DriveTo(-15, 2000, false);
  TurnToAngle(90, 800, false);
  MoveToPoint(-20, 15, -1, 2000, false);
  arm_pid_target = arm_store_target - 20;
  thread apl = thread(arm_pid_loop);
  arm(0);
  dirchangestart = true;
  dirchangeend = true;
  MoveToPoint(-34, 22, -1, 2000, false, 4);
  mogo_mech.set(true);
  wait(200, msec);
  intake(12);
  correct_angle = NormalizeTarget(90);
  DriveTo(1, 800, false);
  MoveToPoint(-24, 38, 1, 2000, true, 8);
  wait_intake_thread();
  correct_angle = NormalizeTarget(10);
  DriveTo(-5, 1000, false);
  TurnToAngle(80, 800, false);
  MoveToPoint(-6, 10, 1, 2000, false);
  apl.interrupt();
  arm_motor.stop(hold);
  intake(12);
  MoveToPoint(-2, -8, 1, 1500);
  correct_angle = NormalizeTarget(135);
  intake(12);
  DriveTo(1000, 1300, false, 3);
  DriveTo(-1, 500, false, 1);
  intakeraise.set(true);
  DriveTo(-2.5, 1000, false, 1);
  Stop(hold);
  intake(-12);
  wait(50, msec);
  intake(12);
  wait(200, msec);
  DriveTo(1000, 800, false);
  intakeraise.set(false);
  DriveTo(-10, 1400, false);
  Stop(hold);
  Doinker.set(true);
  intake(-12);
  wait(50, msec);
  intake(12);
  wait(600, msec);
  intake(0);
  DriveTo(10, 1200, false, 4);
  TurnToAngle(180, 800, false);
  TurnToAngle(-90, 500, false);
  correct_angle = NormalizeTarget(0);
  DriveTo(12, 1500, false);
  mogo_mech.set(false);
  Doinker.set(false);
  MoveToPoint(-15, 25, -11, 2000);
  TurnToAngle(-180, 800);
  Stop(hold);
}

void skills() {
  thread is = thread(intakeStuck);
  thread color_sort = thread(color_sort);
  mogo_mech.set(false);
  arm_motor.setPosition(arm_load_target - 7, deg);
  arm_angle_target = 600;
  thread arm_loop2 = thread(arm_thread);
  correct_angle = InertialA.rotation();
  wait(400, msec);
  DriveTo(-7, 1400, false, 5);
  arm_loop2.interrupt();
  arm_pid_target = arm_store_target;
  thread aplt = thread(arm_pid_loop);
  mogo_mech.set(true);
  wait(200, msec);
  intake(12);
  MoveToPoint(-25, -35, 1, 2000, false);
  intake(0);
  TurnToAngle(-140, 800, false);
  intake(12);
  correct_angle = NormalizeTarget(-150);
  DriveTo(6, 1500, false);
  MoveToPoint(-44, -85, 1, 2500, true);
  wait_intake();
  aplt.interrupt();
  thread al = thread(arm_load);
  MoveToPoint(-44, -98, 1, 2500, true);
  wait_load();
  al.interrupt();
  intake_motor.stop(coast);
  arm_pid_target = arm_store_target;
  thread apl2 = thread(arm_pid_loop);
  /*
  wait(200, msec);
  intake(12);
  MoveToPoint(-53, -102, 1, 2000, true);
  wait_intake_thread();
  correct_angle = NormalizeTarget(-90);
  DriveTo(-4, 800, false);
  TurnToAngle(-60, 800, false);
  thread ita = thread(intake_thread);
  MoveToPoint(-55, -60, 1, 2500, true, 8);
  */
  MoveToPoint(-36, -60, -1, 2500, true, 8);
  TurnToAngle(-90, 800);
  apl2.interrupt();
  // arm_pid_target = arm_score_target;
  // thread apll = thread(arm_pid_loop);
  //DriveTo(1000, 800, false, 6);
  ChassisControl(6, 6);
  wait(500, msec);
  correct_angle = NormalizeTarget(InertialA.rotation());
  Stop(coast);
  //apll.interrupt();
  arm_angle_target = arm_score_target;
  thread at = thread(arm_thread);
  wait(600, msec);
  DriveTo(-6, 1000, false);
  at.interrupt();
  // clipper.set(false);
  Stop(hold);
  arm(0);
  arm_pid_target = arm_load_target;
  thread apll2 = thread(arm_pid_loop);
  wait(400, msec);
  //ita.interrupt();
  wait_load();
  apll2.interrupt();
  // arm_pid_target = arm_score_target;
  // thread apll3 = thread(arm_pid_loop);
  //DriveTo(1000, 600, false, 8);
  ChassisControl(6, 6);
  wait(500, msec);
  correct_angle = NormalizeTarget(-90);
  Stop(coast);
  // apll3.interrupt();
  intake_motor.stop(coast);
  arm_angle_target = arm_score_target;
  thread at2 = thread(arm_thread);
  wait(700, msec);
  DriveTo(-5, 1000, false);
  at2.interrupt();
  // clipper.set(false);
  arm(0);
  arm_pid_target = arm_store_target;
  thread aplll = thread(arm_pid_loop);
  TurnToAngle(-60, 800, false);
  intake(12);
  MoveToPoint(-44, 1, 1, 3000, true, 5);
  wait_intake();
  //wait_intake();
  TurnToAngle(-20, 800, false);
  correct_angle = NormalizeTarget(-70);
  DriveTo(-10, 1500, false);
  //TurnToAngle(-90, 800, false);
  MoveToPoint(-56, -9, 1, 2000, true);
  wait_intake();
  TurnToAngle(-160, 800, false);
  correct_angle = NormalizeTarget(160);
  intake(-12);
  mogo_mech.set(false);
  DriveTo(-1000, 1500, false, 8);
  DriveTo(3, 800, false);
  thread it = thread(intake_thread);
  TurnToAngle(-160, 800, false);
  //MoveToPoint(-56, -9, 1, 2000, false);
  // correct_angle = NormalizeTarget(-170);
  // DriveTo(7, 1500, false, 8);
  // Swing(-70, 1, 1000, false);
  //Stop();
  aplll.interrupt();
  arm(-12);
  //wait_intake_thread();

  //second quarter
  MoveToPoint(0, -11, -1, 2500, false);
  MoveToPoint(26, -11, -1, 2500, false, 5);
  mogo_mech.set(true);
  wait(200, msec);
  it.interrupt();
  TurnToAngle(-135, 800, false);
  intake(12);
  MoveToPoint(30, -33, 1, 2000, false, 8);
  MoveToPoint(50, -32, 1, 2000, false, 8);
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
  TurnToAngle(60, 800, false);
  MoveToPoint(51, -14, 1, 2500, false, 8);
  MoveToPoint(52, 1, 1, 2500, true, 6);
  wait_intake();
  correct_angle = NormalizeTarget(60);
  DriveTo(-8, 1000, false);
  thread al2 = thread(arm_load);
  MoveToPoint(65, -9, 1, 2000, true);
  wait_load();
  TurnToAngle(180, 800, false);
  correct_angle = NormalizeTarget(-135);
  DriveTo(-1000, 1500, false, 8);
  mogo_mech.set(false);
  DriveTo(3, 800, false);
  intake(0);
  MoveToPoint(62, -66, 1, 2000, true, 8);
  MoveToPoint(53, -58, -1, 2000, true, 8);
  TurnToAngle(90, 700);
  //DriveTo(1000, 800, false, 6);
  ChassisControl(6, 6);
  wait(500, msec);
  correct_angle = NormalizeTarget(InertialA.rotation());
  Stop(coast);
  al2.interrupt();
  intake_motor.stop(coast);
  arm_angle_target = arm_score_target;
  thread at3 = thread(arm_thread);
  wait(800, msec);
  DriveTo(-5, 1000, false);
  at3.interrupt();
  // clipper.set(false);
  arm_pid_target = arm_store_target;
  thread apl6 = thread(arm_pid_loop);
  TurnToAngle(150, 800, false);
  thread itt = thread(intake_thread);
  MoveToPoint(28, -80, 1, 2000, false);
  TurnToAngle(180, 800, false);
  TurnToAngle(60, 800, false);
  MoveToPoint(3, -100, -1, 3000, false, 4);
  mogo_mech.set(true);
  wait(200, msec);
  itt.interrupt();
  intake(12);
  MoveToPoint(48, -80, 1, 2000, false);
  MoveToPoint(62, -77, 1, 2000, true, 8);
  wait_intake();
  TurnToAngle(135, 800, false);
  apl6.interrupt();
  thread al3 = thread(arm_load);
  MoveToPoint(62, -101, 1, 2500, true);
  wait(200, msec);
  wait_load();
  al3.interrupt();
  intake_motor.stop(coast);
  arm_pid_target = arm_store_target;
  thread apl7 = thread(arm_pid_loop);
  wait(200, msec);
  intake(12);
  TurnToAngle(-150, 800, false);
  MoveToPoint(53, -104, 1, 1400, false, 10);
  MoveToPoint(51, -115, 1, 1600, false);
  Stop(hold);
  Doinker.set(true);
  wait_intake();
  TurnToAngle(45, 800, false, 8);
  correct_angle = NormalizeTarget(-45);
  Doinker.set(false);
  DriveTo(3, 800, false);
  Stop(hold);
  mogo_mech.set(false);
  wait(300, msec);
  DriveTo(-1000, 1500, false);
  //mogo_mech.set(false);
  wait(200, msec);
  DriveTo(10, 1000, false, 8);
  intake_motor.stop(coast);
  MoveToPoint(9, -110, 1, 2500, true, 8);
  TurnToAngle(180, 800);
  //DriveTo(1000, 800, false, 6);
  ChassisControl(6, 6);
  wait(500, msec);
  correct_angle = NormalizeTarget(InertialA.rotation());
  DriveTo(-4, 800, true);
  Stop(coast);
  // MoveToObject(aiVisionArmBlue, blueStakeColor, 160, 82, 1, 1000, true, 8);
  apl7.interrupt();
  arm(12);
  wait(800, msec);
  DriveTo(-2, 800, false);
  // clipper.set(false);
  arm(-12);
  TurnToAngle(100, 800, false);
  correct_angle = NormalizeTarget(45);
  DriveTo(-4, 800, false);
  correct_angle = NormalizeTarget(70);
  DriveTo(-1000, 1600, false);
  correct_angle = NormalizeTarget(30);
  arm_pid_target = arm_store_target;
  thread apl8 = thread(arm_pid_loop);
  DriveTo(20, 1500, false);
  TurnToAngle(170, 800, false);
  correct_angle = NormalizeTarget(-135);
  DriveTo(-1000, 3000, false);
}

void intakeThird() {
  intake(-12);
  wait(50, msec);
  intake(12);
  wait(300, msec);
  intakeraise.set(true);
}

void NegativeElim() {
  dirchangestart = true;
  dirchangeend = true;
  //InertialA.setRotation((isRed) ? 33 : -33, degrees);
  arm_motor.setPosition(arm_load_target - 7, deg);
  arm(12);
  wait(500, msec);
  arm(0);
  MoveToPoint(-18, -6, -1, 2000, false);
  arm_pid_target = arm_store_target;
  thread apl = thread(arm_pid_loop);
  MoveToPoint(-25, -5, -1, 1500, true, 3);
  mogo_mech.set(true);
  wait(200, msec);
  TurnToAngle(180, 800, false);
  intake(12);
  MoveToPoint(-46, -22, 1, 2000, false, 5);
  MoveToPoint(-48, -38, 1, 1500, false, 4);
  MoveToPoint(-37, -13, -1, 2000, true);
  MoveToPoint(-30, -31, 1, 2000, false, 8);
  apl.interrupt();
  arm_pid_target = 580;
  thread apl2 = thread(arm_pid_loop);
  correct_angle = NormalizeTarget(180);
  DriveTo(-3, 800, false, 8);
  MoveToPoint(0, -43, 1, 2500, false);
  correct_angle = NormalizeTarget(135);
  intake(12);
  DriveTo(1000, 1300, false, 3);
  DriveTo(-1, 500, false, 1);
  intakeraise.set(true);
  DriveTo(-2.5, 1000, false, 1);
  Stop(hold);
  intake(-12);
  wait(50, msec);
  intake(12);
  wait(200, msec);
  thread it = thread(intake_th);
  DriveTo(1000, 800, false);
  intakeraise.set(false);
  DriveTo(-3, 800, false);
  TurnToAngle(60, 800, false);
  it.interrupt();
  thread ithird = thread(intakeThird);
  MoveToPoint(-7, 12, 1, 3500, true, 6);
  DriveTo(-4, 800, false);
  intakeraise.set(false);
  //thread is = thread(intakeStuck);
  Stop(hold);
}

void OldNegativeElim() {
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

void SetupAwp() {
  MoveToPoint(0, -5.5, 1, 1000, true, 4);
  TurnToAngle(-33, 1000, true, 4);
  DriveTo(8, 1000, true, 3);
}

void SetupNegativeElim() {
  DriveTo(-3, 3000, true, 4);
  TurnToAngle(33, 2000, true, 4);
  DriveTo(7, 3000, true, 2);
}

void AwpPositive(){
  // start
  // InertialA.setRotation(isRed ? -33: 33, degrees);
  // InertialA.setRotation(180,degrees);
  arm_motor.setPosition(arm_load_target - 7, deg);
  xpos = 0, ypos = 0;
  InertialA.setRotation(0,degrees);
  MoveToPoint(0,71,1,5000,true,8);
  // TurnToAngle(90,1000,true,8);
  // MoveToPoint(90,90,1,5000,true,8);
  // TurnToAngle(180,1000,true,8);
  // MoveToPoint(90,0,1,5000,true,8);
  // TurnToAngle(-90,1000,true,8);
  // MoveToPoint(0,0,1,5000,true,8);
  // TurnToAngle(0,1000,true,8);
  task::sleep(500);
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("pos: %.2f, %.2f", xpos, ypos);
  // mogo_mech.set(true);



  // MoveToPoint(0,26,-1,2000,true,6);
  // task::sleep(200);
  // MoveToPoint(0,27,-1,1000,false,2);
  // mogo_mech.set(false);
  // Stop();
  // task::sleep(100);
  // TurnToAngle(90,1000,true,8);
  // intake(12);
  // MoveToPoint(26,28,1,1000,true,10);
  // task::sleep(1000);
  // TurnToAngle(-30,1000,true,8);
  // intake_stop();
  // mogo_mech.set(true);
  // MoveToPoint(26,33,1,1000,true,6);
  // TurnToAngle(180,1000,true,8);
  // MoveToPoint(23.5,40,-1,1000,true,4);




  // arm(10);
  // wait(500, msec);
  // MoveToPoint(12, -10, -1, 1000, true, 8);
  // arm_motor.spinToPosition(arm_store_target, deg, 50, velocityUnits::pct, false);
  // intakeraise.set(true);

  // MoveToPoint(14, 3, 1, 1500, true, 4);
  // intakeraise.set(false);
  // thread intakethread=thread(intake_th);
  // intake(6);
  // task::sleep(500);
  // TurnToAngle(-55, 1000, true, 8);
  // MoveToPoint(36, -11, -1, 3000, true, 5);
  // task::sleep(100);
  // mogo_mech.set(false);
  // task::sleep(100);

  // TurnToAngle(180, 1000, true, 12);
  // intake(12);
  // MoveToPoint(39, -36, 1, 1000, true, 10);
  // task::sleep(100);
  // MoveToPoint(39, -33, 1, 1000, true, 10);

  // TurnToAngle(240, 1000, true, 12);
  // MoveToPoint(-15, -52, 1, 2000, true, 6);
  // task::sleep(1000);
  // intake(0);
  // arm_motor.spinToPosition(0, deg, 50, velocityUnits::pct, false);

  // MoveToPoint(32, -36, -1, 1000, true, 6);
  // TurnToAngle(0, 1000, true, 12);
  // MoveToPoint(32, 2, 1, 4000, false, 3);
}

void testPID() {
  thread is = thread(intakeStuck);
  intake(12);
  /*
  DriveTo(80, 5000);
  TurnToAngle(90, 2000);
  TurnToAngle(135, 2000);
  TurnToAngle(160, 2000);
  TurnToAngle(175, 2000);
  TurnToAngle(180, 2000);
  DriveTo(24, 5000);
  DriveTo(-24, 5000);
  TurnToAngle(0, 2000);
  DriveTo(-80, 5000);
  */
  /*
  MoveToPoint(50, 50, 1, 4000, true, 6);
  MoveToPoint(20, 20, 1, 4000, true, 6);
  MoveToPoint(50, 50, 1, 4000, true, 6);
  MoveToPoint(0, 0, 1, 5000, true, 8);
  Controller1.Screen.print(xpos);
  Controller1.Screen.print("abc");
  Controller1.Screen.print(ypos);
  */
}