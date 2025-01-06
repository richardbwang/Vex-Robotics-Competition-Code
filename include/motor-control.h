#include <string>
#include <cmath>
extern bool isturning;
extern bool usevelocity;
extern bool headingcorrection;
extern bool spinfw;
extern bool dirchangestart;
extern bool dirchangeend;
extern double xpos, ypos;
extern double correct_angle, cx, cy;
extern double distance_value;
extern double hangangletarget;
extern double hangangletimelimit;
extern double v;

const double wheel_distance_in = (48.0 / 84.0) * 4.2 * M_PI;

void DriveControl(double left_power, double right_power);
void ChassisControl(double left_power, double right_power);

double GetInertialHeading(bool normalize = false);
double NormalizeAngle(double angle);
double NormalizeTarget(double angle);

void TurnToAngle(double turn_angle, double time_limit_msec, bool exit = true, double max_output = 12);
void DriveTo(double distance_in, double time_limit_msec, bool exit = true, double max_output = 12);
void DriveToGoal(double distance_in, int dir, double time_limit_msec);
void CurveCircle(double result_angle_deg, double center_radius, double time_limit_msec, bool exit = true, double max_output = 12);
void Swing(double swing_angle, double drive_direction, double time_limit_msec, bool exit = true, double max_output = 12);
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
void MoveToPoint(double x, double y, int dir, double time_limit_msec, bool exit = true, double max_output = 12, bool overturn = false);
void boomerang(double x, double y, double a, double dlead, double time_limit_msec, int dir = 1, bool exit = true, double max_output = 12, bool overturn = false);
void hangangle();
void ArmRelease();