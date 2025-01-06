#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor intake_motor = motor(PORT18, ratio6_1, true);
motor right_chassis1 = motor(PORT19, ratio6_1, false);
motor right_chassis2 = motor(PORT14, ratio6_1, true);
motor left_chassis1 = motor(PORT2, ratio6_1, false);
motor left_chassis2 = motor(PORT1, ratio6_1, true);
motor right_chassis3 = motor(PORT16, ratio6_1, false);
motor left_chassis3 = motor(PORT3, ratio6_1, true);
inertial InertialA = inertial(PORT15);
motor catapult_motor = motor(PORT6, ratio18_1, true);
motor hang_motor = motor(PORT5, ratio18_1, false);
distance Distance13 = distance(PORT17);
digital_out DigitalOutF = digital_out(Brain.ThreeWirePort.B);
digital_out DigitalOutA = digital_out(Brain.ThreeWirePort.A);
digital_out DigitalOutB = digital_out(Brain.ThreeWirePort.C);

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