/*Test program for stepper motor with DRV8825 driver*/
#include "mbed.h"
#include "mStepper.h"
#define dirPin D2
#define stepPin D3
#define motorInterfaceType 1
mStepper stepper(mStepper::DRIVER, D3,D2); // step, dir 
int32_t pos[2]={0,200};
uint8_t index=0;
int main() {  
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(3000);
    while(1) {
      stepper.run();
      if (stepper.distanceToGo() == 0)  {
        stepper.disableOutputs();
        wait(1);
        int32_t posDesired = pos[index];
        stepper.moveTo(posDesired);
        index++;
        index%=2;
      }
    }
}
