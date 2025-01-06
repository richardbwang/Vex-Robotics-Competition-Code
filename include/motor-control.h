#include <string>
#include <cmath>
extern bool isRed;
extern bool isturning;
extern bool usevelocity;
extern bool headingcorrection;
extern bool spinfw;
extern bool dirchangestart;
extern bool dirchangeend;
extern double xpos, ypos;
extern double correct_angle, cx, cy;
extern double distance_value;
extern double arm_angle_target, arm_pid_target, arm_load_target, arm_store_target, arm_score_target;
extern double v;
extern double rushsetupangle;

const double wheel_distance_in = (48.0 / 84.0) * 4.2 * M_PI;

void ChassisControl(double left_power, double right_power);
void intake(double inpower);

double GetInertialHeading(bool normalize = false);
double NormalizeAngle(double angle);
double NormalizeTarget(double angle);

void TurnToAngle(double turn_angle, double time_limit_msec, bool exit = true, double max_output = 12);
void TurnToAngleSetup(double turn_angle, double max_output = 12);
void DriveTo(double distance_in, double time_limit_msec, bool exit = true, double max_output = 12);
void CurveCircle(double result_angle_deg, double center_radius, double time_limit_msec, bool exit = true, double max_output = 12);
void Swing(double swing_angle, double drive_direction, double time_limit_msec, bool exit = true, double max_output = 12);
void Grab(double power);
void pullcatapult();
void catapultlaunch(int times, int interval_msec);

void Stop(vex::brakeType type = vex::brake);
void intake_stop(vex::brakeType type = vex::brake);
void ResetChassis();
double GetLeftRotationDegree();
double GetRightRotationDegree();
void heading_correction();
void trackodom();
void trackodomwheel();
void TurnToPoint(double x, double y, int d, double time_limit_msec);
void MoveToPoint(double x, double y, int dir, double time_limit_msec, bool exit = true, double max_output = 12, bool overturn = false);
void MoveToPointEarly(double x, double y, int dir, double time_limit_msec, bool exit = true, double max_output = 12, bool overturn = false);
void boomerang(double x, double y, double a, double dlead, double time_limit_msec, int dir = 1, bool exit = true, double max_output = 12, bool overturn = false);
void hangangle();
void ArmReleaseLeft();
void ArmReleaseRight();
void BarCross();
void friction_test();
void arm(double armpower);
void arm_thread();
void arm_pid(double arm_target);
void arm_pid_loop();
void wait_intake();
void wait_intake_thread();
void arm_load();