#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor left_fw_motor = motor(PORT8, ratio6_1, true);
motor left_chassis1 = motor(PORT3, ratio18_1, false);
motor right_chassis1 = motor(PORT2, ratio18_1, true);
limit LimitSwitchA = limit(Brain.ThreeWirePort.H);
motor lift_down = motor(PORT9, ratio18_1, true);
motor right_fw_motor = motor(PORT5, ratio6_1, false);
motor left_chassis2 = motor(PORT4, ratio18_1, false);
motor right_chassis2 = motor(PORT1, ratio18_1, true);
motor grab_motor = motor(PORT6, ratio18_1, false);
inertial InertialA = inertial(PORT10);
digital_out DigitalOutG = digital_out(Brain.ThreeWirePort.G);
/*vex-vision-config:begin*/
signature VisionA__HIGH_GOAL_RED = signature (1, 4441, 7203, 5822, -1921, -409, -1165, 1.4, 0);
signature VisionA__HIGH_GOAL_BLUE = signature (2, -2771, 219, -1276, 1959, 6677, 4318, 1.1, 0);
vision VisionA = vision (PORT19, 50, VisionA__HIGH_GOAL_RED, VisionA__HIGH_GOAL_BLUE);
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