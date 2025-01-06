#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor left_chassis1 = motor(PORT13, ratio6_1, true);
motor left_chassis2 = motor(PORT14, ratio6_1, false);
motor left_chassis3 = motor(PORT15, ratio6_1, true);
motor right_chassis1 = motor(PORT19, ratio6_1, false);
motor right_chassis2 = motor(PORT6, ratio6_1, true);
motor right_chassis3 = motor(PORT18, ratio6_1, false);
motor right_intake = motor(PORT2, ratio18_1, false);
inertial inertial_sensor = inertial(PORT5);
motor left_intake = motor(PORT1, ratio18_1, true);
digital_out goal_clamp = digital_out(Brain.ThreeWirePort.A);
digital_out PTO = digital_out(Brain.ThreeWirePort.D);
optical optical_sensor = optical(PORT21);
distance distance_sensor_arm = distance(PORT9);
digital_out hang_deploy = digital_out(Brain.ThreeWirePort.F);
digital_out filter = digital_out(Brain.ThreeWirePort.E);
distance distance_sensor = distance(PORT17);
digital_out Sort = digital_out(Brain.ThreeWirePort.B);

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