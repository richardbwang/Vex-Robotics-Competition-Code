#include <string>
extern bool isturning;
extern bool usevelocity;
extern double correct_angle;

const double wheel_distance_in = (36.0 / 60.0) * 3.25 * 3.14159;

void ChassisControl(double left_power, double right_power);
void ChassisControlReverse(double left_power, double right_power);
void ChassisControl1(double left_power, double right_power, double left_only = false, double right_only = false);

double GetInertialHeading(bool normalize = false);
double NormalizeAngle(double angle);

// New turning function using PID class.
double TurnSmallNoBool(double turn_angle, double time_limit_msec);
double TurnSmall(double turn_angle, double time_limit_msec);
double TurnMedium(double turn_angle, double time_limit_msec);
double TurnBig(double turn_angle, double time_limit_msec);
void TurnToAngle(double turn_angle, double time_limit_msec);

void DriveTo(double distance_in, double time_limit_msec, double max_output = 12);
void DriveToNew(double distance_in, double time_limit_msec, double max_output = 12);
void CurveCircle(double result_angle_deg, double center_radius, double time_limit_msec, double max_output = 12);
void Grab(double power);

void Stop(vex::brakeType type = vex::brake);
void ResetChassis();
double GetLeftRotationDegree();
double GetRightRotationDegree();

void PullCatapult();
void FireCatapult();
void GrabCheck();
void GrabCheckFast();
void DriveToSeperate(double l_distance_in, double r_distance_in, double time_limit_msec, double max_output = 0);
void WiggleForward(double distance_in, double wiggle_amplifier, double wiggle_frequency, double forward_speed, double wiggle_time, double drive_time_limit_msec);
void distance_check();
void grab_wait();
void heading_correction();