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
inertial InertialA = inertial(PORT3);
digital_out mogo_mech = digital_out(Brain.ThreeWirePort.E);
digital_out clipper = digital_out(Brain.ThreeWirePort.B);
optical Optical = optical(PORT15);
digital_out intakeraise = digital_out(Brain.ThreeWirePort.A);
digital_out Doinker = digital_out(Brain.ThreeWirePort.C);
distance distance_sensor = distance(PORT2);
distance clip_sensor = distance(PORT5);
motor arm_motor = motor(PORT21, ratio18_1, false);
rotation X = rotation(PORT1, false);
rotation Y = rotation(PORT10, false);
bumper arm_stop = bumper(Brain.ThreeWirePort.F);
vision::signature redStakeSig = signature(1, 8757, 10341, 9549, -993, 1, -496, 7.200, 0);
vision Vision1 = vision(PORT7, 90, redStakeSig);
aivision::colordesc wallStakeColor = aivision::colordesc(1, 83, 127, 31, 23, 1);
aivision::colordesc redStakeColor = aivision::colordesc(2, 223, 31, 85, 18, 1);
aivision::colordesc blueStakeColor = aivision::colordesc(3, 30, 119, 181, 35, 1);
aivision aiVisionArmWall = aivision(PORT8, wallStakeColor);
aivision aiVisionArmRed = aivision(PORT8, redStakeColor);
aivision aiVisionArmBlue = aivision(PORT8, blueStakeColor);

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