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
extern digital_out clipper;
extern optical Optical;
extern digital_out intakeraise;
extern digital_out Doinker;
extern distance distance_sensor;
extern distance clip_sensor;
extern motor arm_motor;
extern rotation X;
extern rotation Y;
extern bumper arm_stop;
extern vision Vision1;
extern vision::signature redStakeSig;
extern aivision aiVisionArmWall;
extern aivision aiVisionArmRed;
extern aivision aiVisionArmBlue;
extern aivision::colordesc wallStakeColor;
extern aivision::colordesc redStakeColor;
extern aivision::colordesc blueStakeColor;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );