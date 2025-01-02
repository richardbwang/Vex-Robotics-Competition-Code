using namespace vex;

extern brain Brain;

using signature = vision::signature;

// VEXcode devices
extern controller Controller1;
extern motor left_fw_motor;
extern motor left_chassis1;
extern motor right_chassis1;
extern limit LimitSwitchA;
extern motor lift_down;
extern motor right_fw_motor;
extern motor left_chassis2;
extern motor right_chassis2;
extern motor grab_motor;
extern inertial InertialA;
extern signature VisionA__HIGH_GOAL_RED;
extern signature VisionA__HIGH_GOAL_BLUE;
extern signature VisionA__HIGH_GOAL_RED_GRANT;
extern signature VisionA__SIG_4;
extern signature VisionA__SIG_5;
extern signature VisionA__SIG_6;
extern signature VisionA__SIG_7;
extern vision VisionA;
extern signature Vision7__SIG_1;
extern signature Vision7__SIG_2;
extern signature Vision7__SIG_3;
extern signature Vision7__SIG_4;
extern signature Vision7__SIG_5;
extern signature Vision7__SIG_6;
extern signature Vision7__SIG_7;
extern vision Vision7;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );