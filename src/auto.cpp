#include "vex.h"
#include "motor-control.h"
#include "math.h"
#include "auto.h"

#include <string>

void chassis_reset() {
  left_chassis1.resetRotation();
  right_chassis1.resetRotation();
}
void run(int Lpower ,int Rpower){
  left_chassis1.spin(fwd, 0.128 * Lpower, voltageUnits::volt);
  right_chassis1.spin(fwd, 0.128 * Rpower, voltageUnits::volt);
}

void auto_shoot_disk() {
  liftDown(100);
  wait(100, msec);

  while (LimitSwitchA.pressing() == 0) {
    liftDown(-50);
    wait(50, msec);
  }

  if (LimitSwitchA.pressing() == 1) {
    lift_down.resetRotation();
    liftDown(0);
  } else {
    liftDown(-50);
    wait(100, msec);
    lift_down.resetRotation();
    liftDown(0);
  }
}

//================================Vision Sensor============================================//
double AimingGoal(ROLLER roller_color) {
  int found_num = roller_color == ROLLER_RED ? 
                  VisionA.takeSnapshot(VisionA__HIGH_GOAL_RED) :
                  VisionA.takeSnapshot(VisionA__HIGH_GOAL_BLUE);
  if (found_num > 0) {
    int x = VisionA.largestObject.centerX;
      // int y = VisionA.largestObject.centerY;
      // int h = VisionA.largestObject.height;
      // int w = VisionA.largestObject.width;
      // int a = VisionA.largestObject.angle;
      // int ox = VisionA.largestObject.originX;
      // int oy = VisionA.largestObject.originY;
    if ((x <= 150) and (x >= 145)) {
        // Aiming is good, shoot
    } else {
        double degree = (double(x) - 150.00) * 30 / 100;
        TurnForAngle(degree, 200);
        return degree;
    }
    return 0.0;
  }

  return 0.0;
}

void auto_long_shot(int num) {
  for (int i = 0; i < num; i++) {
     FwSpinFor(450, 450, 200);
     wait(1000, msec);
     auto_shoot_disk();
  }
  fw_rpm(0, 0);
}

void auto_roll_roller(ROLLER roller) {
  if (roller == ROLLER_RED) {
    grab(100);
  } else {
    grab(-100);
  }
  grab (0);
}
