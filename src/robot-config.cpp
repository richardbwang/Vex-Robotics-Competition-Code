#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;
controller Controller1 = controller(primary);

// VEXcode device constructors
////////////////////////////////
// State
///////////////////////////////
motor left_chassis1 = motor(PORT12, ratio6_1, true);
motor left_chassis2 = motor(PORT5, ratio6_1, true);
motor left_chassis3 = motor(PORT20, ratio6_1, true);
motor right_chassis1 = motor(PORT11, ratio6_1, false);
motor right_chassis2 = motor(PORT19, ratio6_1, false);
motor right_chassis3 = motor(PORT13, ratio6_1, false);
motor arm_motor1 = motor(PORT4, ratio18_1, true);
motor arm_motor2 = motor(PORT2, ratio18_1, false);
motor_group arm_motor = motor_group(arm_motor1, arm_motor2);
motor intake_motor = motor(PORT3, ratio18_1, false);
inertial InertialA = inertial(PORT21);
digital_out mogo_mech = digital_out(Brain.ThreeWirePort.C);
optical Optical = optical(PORT7);
digital_out intakeraise = digital_out(Brain.ThreeWirePort.G);
digital_out Doinker = digital_out(Brain.ThreeWirePort.E);
distance distance_sensor = distance(PORT8);
distance clamp_distance = distance(PORT6);
distance front_distance = distance(PORT1);
rotation X = rotation(PORT10, false);
rotation Y = rotation(PORT9, true);
bumper arm_stop = bumper(Brain.ThreeWirePort.H);

////////////////////////////////
// Norcal Sig
///////////////////////////////
// motor left_chassis1 = motor(PORT1, ratio6_1, true);
// motor left_chassis2 = motor(PORT2, ratio6_1, true);
// motor left_chassis3 = motor(PORT3, ratio6_1, true);
// motor right_chassis1 = motor(PORT8, ratio6_1, false);
// motor right_chassis2 = motor(PORT9, ratio6_1, false);
// motor right_chassis3 = motor(PORT10, ratio6_1, false);
// motor intake_motor = motor(PORT13, ratio18_1, true);
// inertial InertialA = inertial(PORT4);
// digital_out mogo_mech = digital_out(Brain.ThreeWirePort.C);
// optical Optical = optical(PORT14);
// digital_out intakeraise = digital_out(Brain.ThreeWirePort.A);
// digital_out Doinker = digital_out(Brain.ThreeWirePort.H);
// distance distance_sensor = distance(PORT5);
// distance front_distance = distance(PORT7);
// motor arm_motor1 = motor(PORT18, ratio18_1, false);
// motor arm_motor2 = motor(PORT19, ratio18_1, true);
// motor_group arm_motor = motor_group(arm_motor1, arm_motor2);
// rotation X = rotation(PORT11, false);
// rotation Y = rotation(PORT12, true);
// bumper arm_stop = bumper(Brain.ThreeWirePort.B);

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