#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
limit LimitSwitchA = limit(Brain.ThreeWirePort.A);
motor catapult_motor = motor(PORT11, ratio18_1, false);
motor intake_motor = motor(PORT8, ratio6_1, false);
motor left_chassis1 = motor(PORT19, ratio18_1, false);
motor left_chassis2 = motor(PORT12, ratio18_1, true);
motor right_chassis1 = motor(PORT17, ratio18_1, true);
motor right_chassis2 = motor(PORT16, ratio18_1, false);
motor left_chassis3 = motor(PORT13, ratio18_1, false);
motor right_chassis3 = motor(PORT20, ratio18_1, true);
inertial InertialA = inertial(PORT18);
digital_out DigitalOutB = digital_out(Brain.ThreeWirePort.B);
digital_out DigitalOutC = digital_out(Brain.ThreeWirePort.C);

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