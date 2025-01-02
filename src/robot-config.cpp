#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor leftintake_motor = motor(PORT6, ratio6_1, false);
motor left_chassis1 = motor(PORT4, ratio18_1, false);
motor right_chassis1 = motor(PORT3, ratio18_1, true);
limit LimitSwitchA = limit(Brain.ThreeWirePort.A);
motor lift_down = motor(PORT8, ratio18_1, true);
motor righttintake_motor = motor(PORT7, ratio6_1, true);
motor left_chassis2 = motor(PORT2, ratio18_1, false);
motor right_chassis2 = motor(PORT1, ratio18_1, true);
motor xi_motor = motor(PORT5, ratio18_1, false);

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