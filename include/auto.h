#include <string>

enum ROLLER {ROLLER_RED, ROLLER_BLUE};

void run(int Lpower ,int Rpower);
void chassis_reset();
void auto_shoot_disk();
void auto_long_shot(int num);
void auto_roll_roller(ROLLER);
double AimingGoal(ROLLER);