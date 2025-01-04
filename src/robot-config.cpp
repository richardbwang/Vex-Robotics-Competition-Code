#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor intake_motor = motor(PORT19, ratio18_1, true);
motor right_chassis1 = motor(PORT15, ratio6_1, false);
motor right_chassis2 = motor(PORT16, ratio6_1, true);
motor left_chassis1 = motor(PORT6, ratio6_1, true);
motor left_chassis2 = motor(PORT9, ratio6_1, false);
motor right_chassis3 = motor(PORT20, ratio6_1, false);
motor left_chassis3 = motor(PORT21, ratio6_1, true);
motor catapult = motor(PORT11, ratio36_1, false);
motor awp_motor = motor(PORT1, ratio18_1, false);

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