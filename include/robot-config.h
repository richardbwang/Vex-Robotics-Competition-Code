using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor intake_motor;
extern motor right_chassis1;
extern motor right_chassis2;
extern motor left_chassis1;
extern motor left_chassis2;
extern motor right_chassis3;
extern motor left_chassis3;
extern motor catapult;
extern motor awp_motor;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );