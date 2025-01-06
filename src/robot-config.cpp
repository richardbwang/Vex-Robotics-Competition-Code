#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor intake_motor = motor(PORT20, ratio6_1, true);
motor right_chassis1 = motor(PORT3, ratio6_1, false);
motor right_chassis2 = motor(PORT4, ratio6_1, true);
motor left_chassis1 = motor(PORT1, ratio6_1, true);
motor left_chassis2 = motor(PORT9, ratio6_1, false);
motor right_chassis3 = motor(PORT5, ratio6_1, false);
motor left_chassis3 = motor(PORT2, ratio6_1, true);
inertial InertialA = inertial(PORT13);
motor catapult_motor = motor(PORT18, ratio18_1, false);
motor awp_motor = motor(PORT19, ratio18_1, false);
distance Distance14 = distance(PORT14);

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