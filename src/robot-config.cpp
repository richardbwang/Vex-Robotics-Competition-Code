#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor left_chassis1 = motor(PORT14, ratio6_1, true);
motor left_chassis2 = motor(PORT13, ratio6_1, false);
motor left_chassis3 = motor(PORT12, ratio6_1, true);
motor left_chassis4 = motor(PORT11, ratio18_1, true);
motor right_chassis1 = motor(PORT20, ratio6_1, false);
motor right_chassis2 = motor(PORT19, ratio6_1, false);
motor right_chassis3 = motor(PORT18, ratio18_1, false);
motor right_chassis4 = motor(PORT17, ratio18_1, true);
motor intake_motor = motor(PORT15, ratio6_1, true);
inertial InertialA = inertial(PORT1);
digital_out DigitalOutA = digital_out(Brain.ThreeWirePort.B);
digital_out DigitalOutB = digital_out(Brain.ThreeWirePort.A);
distance Distance13 = distance(PORT16);
digital_out DigitalOutH = digital_out(Brain.ThreeWirePort.C);

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