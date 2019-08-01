/*Test program for stepper motor with DRV8825 driver*/
// Trapzoidal velocity profile
#include "mbed.h"
#include "mStepper.h"
#define dirPin D2
#define stepPin D3
InterruptIn bot(BUTTON1);
Timeout to;             // for debounce
AnalogIn pot(A0);       // for pot

mStepper stepper(mStepper::DRIVER,stepPin,dirPin); // step, dir 

volatile bool f_run=false, f_first=true;
void decide(){     // decision after 100ms
    f_run=!f_run;
    f_first=true;
    to.detach();
}
void runStop(){    // button callback
    if (f_first){
        to.attach_us(decide, 100000);
        f_first=false;
    }
}
int main() {
  bot.fall(runStop);
  stepper.setMaxSpeed(10000);
  stepper.setSpeed(5000);   
    while(1) {
        if (f_run){
            stepper.setSpeed((long)(50000*pot));
            stepper.runSpeed();
        }
    }
}
