#include <string>
#include <cmath>
extern bool isturning;
extern bool usevelocity;
extern bool headingcorrection;
extern double xpos, ypos;
extern double correct_angle, cx, cy;
extern double distance_value;

const double wheel_distance_in = (36.0 / 84.0) * 4 * M_PI;

void DriveControl(double left_power, double right_power);
void ChassisControl(double left_power, double right_power);

double GetInertialHeading(bool normalize = false);
double NormalizeAngle(double angle);
double NormalizeTarget(double angle);

void TurnToAngle(double turn_angle, double time_limit_msec);
void DriveTo(double distance_in, double time_limit_msec, double max_output = 12);
void CurveCircle(double result_angle_deg, double center_radius, double time_limit_msec, bool exit = true, double max_output = 12);
void Swing(double swing_angle, double drive_direction, double time_limit_msec, double max_output = 12);
void Grab(double power);
void Arm(double power);
void pullcatapult();
void catapultlaunch(int times, int interval_msec);

void Stop(vex::brakeType type = vex::brake);
void ResetChassis();
double GetLeftRotationDegree();
double GetRightRotationDegree();
void heading_correction();
void trackodom();
void TurnToPoint(double x, double y, int d, double time_limit_msec);
void MoveToPoint(double x, double y, int dir, double time_limit_msec, bool exit = true, double max_output = 12);
void boomerang(double x, double y, double a, double dlead, double time_limit_msec, bool exit = true, double max_output = 12);