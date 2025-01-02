#include <string>

void ChassisControl(float lPower, float rPower);
void ChassisControlPercent(float lPower, float rPower);
void ResetChassis();
double GetLeftRotationDegree();
double GetRightRotationDegree();

double TurnForAngle(float turnAngle, float timeLimit_msec);
void DriveFor(float distance_in, float time_limit_msec);
void DriveForTest(float distance_in);

void Stop(vex::brakeType type = vex::brake);

void liftDown(float power);

void fw(float lPower, float rPower);
void fw_rpm(double lRpm, double rRpm);
void fw_pid_rpm_with_time_limit(double lRpm, double rRpm, int time_limt_msec);
bool AimingGoal();

// Grab the disk and also spin the roller.
void grab(float power);