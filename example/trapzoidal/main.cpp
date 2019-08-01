/*Test program for stepper motor with DRV8825 driver*/
// Trapzoidal velocity profile
#include "mbed.h"
#include "mStepper.h"
#define dirPin D2
#define  stepPin D3

mStepper stepper(mStepper::DRIVER,stepPin,dirPin); // step, dir 
int32_t pos[2]={0,12800};
uint8_t index=0;
int main() {  
 stepper.setMaxSpeed(8000); //8000
 //stepper. setSpeed(500);
 stepper.setAcceleration(8000); // constant speed within 1s
   while(1) {
     stepper.run();
     if (stepper.distanceToGo() == 0)  {
       stepper.disableOutputs(); // to save power
       wait(1);
       int32_t targetPos = pos[index];
       stepper.moveTo(targetPos);
       index++;   index%=2;  // make 0-1-0-1
     }
   }
}

