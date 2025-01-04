#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor hang_motor = motor(PORT2, ratio18_1, false);
motor intake_motor = motor(PORT7, ratio6_1, true);
motor right_chassis1 = motor(PORT17, ratio6_1, false);
motor right_chassis2 = motor(PORT18, ratio6_1, false);
motor left_chassis1 = motor(PORT12, ratio6_1, true);
motor left_chassis2 = motor(PORT13, ratio6_1, true);
motor right_chassis3 = motor(PORT19, ratio6_1, false);
motor left_chassis3 = motor(PORT14, ratio6_1, true);
inertial InertialA = inertial(PORT15);
motor arm_motor = motor(PORT8, ratio18_1, false);
motor catapult_motor = motor(PORT4, ratio36_1, false);
distance Distance5 = distance(PORT5);
digital_out DigitalOutA = digital_out(Brain.ThreeWirePort.A);
digital_out DigitalOutB = digital_out(Brain.ThreeWirePort.B);

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