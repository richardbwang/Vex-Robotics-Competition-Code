using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor puncher_motor;
extern motor intake_motor;
extern motor left_chassis1;
extern motor left_chassis2;
extern motor right_chassis1;
extern motor right_chassis2;
extern motor left_chassis3;
extern motor right_chassis3;
extern inertial InertialA;
extern digital_out DigitalOutA;
extern distance Distance9;
extern digital_out DigitalOutD;
extern digital_out DigitalOutE;
extern digital_out DigitalOutB;
extern digital_out DigitalOutC;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );