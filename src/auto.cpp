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
  if (LimitSwitchA.pressing() == 0) {
    liftDown(-50);
  }

  liftDown(100);
  wait(150, msec);
  liftDown(-100);

  if (LimitSwitchA.pressing() == 1) {
    liftDown(0);
  }
  wait(150, msec);
  liftDown(-50);
}

void auto_long_shot(int num) {
  for (int i = 0; i < num; i++) {
     fw_pid_rpm_with_time_limit(75, 75, 200);
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
