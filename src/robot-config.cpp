#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor left_chassis1 = motor(PORT17, ratio6_1, true);
motor left_chassis2 = motor(PORT19, ratio6_1, false);
motor left_chassis3 = motor(PORT20, ratio6_1, true);
motor right_chassis1 = motor(PORT11, ratio6_1, false);
motor right_chassis2 = motor(PORT12, ratio6_1, true);
motor right_chassis3 = motor(PORT13, ratio6_1, false);
motor intake_motor = motor(PORT9, ratio18_1, true);
inertial InertialA = inertial(PORT14);
digital_out mogo_mech = digital_out(Brain.ThreeWirePort.E);
digital_out clipper = digital_out(Brain.ThreeWirePort.B);
optical Optical = optical(PORT15);
digital_out intakeraise = digital_out(Brain.ThreeWirePort.A);
digital_out Doinker = digital_out(Brain.ThreeWirePort.C);
distance distance_sensor = distance(PORT2);
motor arm_motor = motor(PORT21, ratio36_1, false);
rotation X = rotation(PORT1, false);
rotation Y = rotation(PORT10, false);
bumper arm_stop = bumper(Brain.ThreeWirePort.F);
digital_out Sort = digital_out(Brain.ThreeWirePort.H);

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