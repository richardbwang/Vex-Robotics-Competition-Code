void ChassisControl(float lPower, float rPower);
void Stop(vex::brakeType type = vex::brake);
void intake(float lPower, float rPower);
void liftDown(float power);
void intake_rpm(float lPower, float rPower);
void fw_adjustable_rpm(float lRpm, float rRpm);
void intake_pid(float lPower, float rPower);
void xi(float power);