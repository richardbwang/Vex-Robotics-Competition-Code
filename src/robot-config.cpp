#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;
controller Controller1 = controller(primary);

// VEXcode device constructors
////////////////////////////////
// Worlds
///////////////////////////////
motor left_chassis1 = motor(PORT5, ratio6_1, true);
motor left_chassis2 = motor(PORT6, ratio6_1, true);
motor left_chassis3 = motor(PORT7, ratio6_1, false);
motor right_chassis1 = motor(PORT3, ratio6_1, false);
motor right_chassis2 = motor(PORT4, ratio6_1, false);
motor right_chassis3 = motor(PORT11, ratio6_1, true);
motor arm_motor1 = motor(PORT2, ratio18_1, true);
motor arm_motor2 = motor(PORT1, ratio18_1, false);
motor_group arm_motor = motor_group(arm_motor1, arm_motor2);
motor intake_motor = motor(PORT8, ratio18_1, true);
inertial InertialA = inertial(PORT9);
digital_out mogo_mech = digital_out(Brain.ThreeWirePort.E);
optical Optical = optical(PORT18);
digital_out intakeraise = digital_out(Brain.ThreeWirePort.F);
digital_out Doinker = digital_out(Brain.ThreeWirePort.G);
digital_out cage = digital_out(Brain.ThreeWirePort.H);
distance distance_sensor = distance(PORT20);
distance clamp_distance = distance(PORT12);
rotation X = rotation(PORT13, true);
rotation Y = rotation(PORT19, true);
bumper arm_stop = bumper(Brain.ThreeWirePort.D);

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