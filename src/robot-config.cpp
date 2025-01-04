#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
limit LimitSwitchA = limit(Brain.ThreeWirePort.A);
motor catapult_motor = motor(PORT16, ratio18_1, true);
motor intake_motor = motor(PORT2, ratio6_1, false);
motor left_chassis1 = motor(PORT14, ratio6_1, true);
motor left_chassis2 = motor(PORT13, ratio6_1, false);
motor right_chassis1 = motor(PORT18, ratio6_1, false);
motor right_chassis2 = motor(PORT17, ratio6_1, true);
motor left_chassis3 = motor(PORT12, ratio6_1, false);
motor right_chassis3 = motor(PORT19, ratio6_1, true);
inertial InertialA = inertial(PORT4);
digital_out DigitalOutC = digital_out(Brain.ThreeWirePort.C);
distance DistanceA = distance(PORT3);
digital_out DigitalOutD = digital_out(Brain.ThreeWirePort.D);
digital_out DigitalOutE = digital_out(Brain.ThreeWirePort.E);

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