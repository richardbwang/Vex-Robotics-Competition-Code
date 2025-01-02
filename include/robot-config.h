using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor leftintake_motor;
extern motor left_chassis1;
extern motor right_chassis1;
extern limit LimitSwitchA;
extern motor lift_down;
extern motor righttintake_motor;
extern motor left_chassis2;
extern motor right_chassis2;
extern motor xi_motor;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );