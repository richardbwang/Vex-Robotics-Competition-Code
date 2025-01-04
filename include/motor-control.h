#include <string>

void ChassisControl(float left_power, float right_power);
void ChassisControlPercent(float left_velocity, float right_velocity);

double GetInertialHeading();
double NormalizeAngle(double angle);
double TurnForAngle(float turn_angle, float time_limit_msec);

// New turning function using PID class.
double TurnSmallNoBool(float turn_angle, float time_limit_msec);
double TurnSmall(float turn_angle, float time_limit_msec);
double TurnMedium(float turn_angle, float time_limit_msec);
double TurnBig(float turn_angle, float time_limit_msec);

void DriveTo(float distance_in, float max_output, float time_limit_msec);
void DriveFor(float distance_in, float time_limit_msec);
void DriveFor(float distance_in, float time_limit_msec, bool catapult);
void Grab(float power);

void Stop(vex::brakeType type = vex::brake);
void chassis_reset();
void run(float lPower, float rPower);
void PullCatapult(void);
void FireCatapult(void);
void PullCatapultStop(void);
void CatapultStop(void);
void AngleStabilization(void);
void GrabCheck(void);