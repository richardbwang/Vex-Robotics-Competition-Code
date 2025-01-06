using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor left_chassis1;
extern motor left_chassis2;
extern motor left_chassis3;
extern motor right_chassis1;
extern motor right_chassis2;
extern motor right_chassis3;
extern motor right_intake;
extern inertial inertial_sensor;
extern motor left_intake;
extern digital_out goal_clamp;
extern digital_out PTO;
extern optical optical_sensor;
extern distance distance_sensor_arm;
extern digital_out hang_deploy;
extern digital_out filter;
extern distance distance_sensor;
extern digital_out Sort;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );