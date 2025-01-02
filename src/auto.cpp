#include "vex.h"
#include "motor-control.h"
#include "math.h"

void chassis_reset()
{
  left_chassis1.resetRotation();
 
  right_chassis1.resetRotation();
 


}
void run(int Lpower ,int Rpower)
{
  left_chassis1. spin(fwd, 0.128 * Lpower, voltageUnits::volt);
 

  right_chassis1.spin(fwd, 0.128 * Rpower, voltageUnits::volt);
  
}
