#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor left_fw_motor = motor(PORT5, ratio6_1, false);
motor left_chassis1 = motor(PORT3, ratio18_1, false);
motor right_chassis1 = motor(PORT2, ratio18_1, true);
limit LimitSwitchA = limit(Brain.ThreeWirePort.A);
motor lift_down = motor(PORT9, ratio18_1, true);
motor right_fw_motor = motor(PORT8, ratio6_1, true);
motor left_chassis2 = motor(PORT4, ratio18_1, false);
motor right_chassis2 = motor(PORT1, ratio18_1, true);
motor grab_motor = motor(PORT6, ratio18_1, false);
inertial InertialA = inertial(PORT10);
/*vex-vision-config:begin*/
signature VisionA__HIGH_GOAL_RED = signature (1, 771, 9973, 5372, -1813, 253, -780, 0.9, 0);
signature VisionA__HIGH_GOAL_BLUE = signature (2, -2445, -637, -1541, 1549, 6885, 4217, 1.4, 0);
signature VisionA__HIGH_GOAL_RED_GRANT = signature (3, 0, 0, 0, 0, 0, 0, 0, 0);
signature VisionA__SIG_4 = signature (4, 0, 0, 0, 0, 0, 0, 3, 0);
signature VisionA__SIG_5 = signature (5, 0, 0, 0, 0, 0, 0, 3, 0);
signature VisionA__SIG_6 = signature (6, 0, 0, 0, 0, 0, 0, 3, 0);
signature VisionA__SIG_7 = signature (7, 0, 0, 0, 0, 0, 0, 3, 0);
vision VisionA = vision (PORT19, 37, VisionA__HIGH_GOAL_RED, VisionA__HIGH_GOAL_BLUE, VisionA__HIGH_GOAL_RED_GRANT, VisionA__SIG_4, VisionA__SIG_5, VisionA__SIG_6, VisionA__SIG_7);
/*vex-vision-config:end*/
/*vex-vision-config:begin*/
vision Vision7 = vision (PORT7, 50);
/*vex-vision-config:end*/

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}