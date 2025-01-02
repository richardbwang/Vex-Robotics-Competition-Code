#include <string>

void ChassisControl(float lPower, float rPower);
void ChassisControlPercent(float lPower, float rPower);
void ResetChassis();
double GetLeftRotationDegree();
double GetRightRotationDegree();

double TurnForAngle(float turnAngle, float timeLimit_msec);
void DriveFor(float distance_in, float time_limit_msec);

void Stop(vex::brakeType type = vex::brake);

void liftDown(float power);



void fw(float lPower, float rPower);
void fw_rpm(double lRpm, double rRpm);
void fw_pid_rpm(double lRpm, double rRpm);
void FwSpinFor(double l_rpm, double r_rpm, int time_limt_msec);
void fw_pid_rpm_with_time_limit(double lRpm, double rRpm, int time_limt_msec);

// Grab the disk and also spin the roller.
void grab(float power);