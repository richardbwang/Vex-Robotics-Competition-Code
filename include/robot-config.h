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
extern motor intake_motor;
extern inertial InertialA;
extern digital_out mogo_mech;
extern optical Optical;
extern digital_out intakeraise;
extern digital_out Doinker;
extern distance distance_sensor;
extern distance clamp_distance;
extern distance front_distance;
extern motor arm_motor1;
extern motor arm_motor2;
extern motor_group arm_motor;
extern rotation X;
extern rotation Y;
extern bumper arm_stop;
extern digital_out cage;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );