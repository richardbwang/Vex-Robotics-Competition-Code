#include <string>
extern bool isturning;
extern bool usevelocity;
extern bool headingcorrection;
extern bool launch;
extern double correct_angle;
extern double distance_value;

const double wheel_distance_in = (36.0 / 72.0) * 4 * 3.14159;

void DriveControl(double left_power, double right_power);
void ChassisControl(double left_power, double right_power);

double GetInertialHeading(bool normalize = false);

double NormalizeAngle(double angle);

void TurnToAngle(double turn_angle, double time_limit_msec);
void DriveTo(double distance_in, double time_limit_msec, double max_output = 12);
void CurveCircle(double result_angle_deg, double center_radius, double time_limit_msec, bool exit = true, double max_output = 12);
void Swing(double swing_angle, double drive_direction, double time_limit_msec, double max_output = 12);
void Grab(double power);
void Arm(double power);
void pullcatapult();
void catapultlaunch(int interval_msec);

void Stop(vex::brakeType type = vex::brake);
void ResetChassis();
double GetLeftRotationDegree();
double GetRightRotationDegree();
void heading_correction();