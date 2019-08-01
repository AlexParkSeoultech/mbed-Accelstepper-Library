// mStepper.cpp
// Modified by Alex Park
// Copyright (C) 2009-2013 Mike McCauley
// $Id: mStepper.cpp,v 1.19 2014/10/31 06:05:27 mikem Exp mikem $

#include "mStepper.h"

void mStepper::moveTo(long absolute){   //move to absolute position
    if (_targetPos != absolute)     {
        _targetPos = absolute;
        computeNewSpeed();
    }
}

void mStepper::move(long relative) {  //move to absolute position
    moveTo(_currentPos + relative);
}

// Implements steps according to the current _stepInterval
// You must call this at least once per step
// returns true if a step occurred
bool mStepper::runSpeed(){
    // Dont do anything unless we actually have a step interval
    if (!_stepInterval)  // if dt==0
        return false;
    if (t.read_us() >= _stepInterval)     {
        if (_direction == DIRECTION_CW)     {       // Clockwise
            _currentPos += 1;
        }
        else    { // Anticlockwise          
            _currentPos -= 1;
        }
        step(_currentPos);       // make physically one pulse
        t.reset();
        return true;
    }
    else    {
        return false;
    }
}

long mStepper::distanceToGo() {   //find out distance to target
    return _targetPos - _currentPos;
}

long mStepper::targetPosition()  { // return Target pos
    return _targetPos;
}

long mStepper::currentPosition()  { //return current pos
    return _currentPos;
}


void mStepper::setCurrentPosition(long position) {  // Assign current pos
    _targetPos = _currentPos = position;
    _n = 0;
    _stepInterval = 0;
}

void mStepper::computeNewSpeed(){
    long distanceTo = distanceToGo(); // +ve is clockwise from curent location
    long stepsToStop = (long)((_speed * _speed) / (2.0f * _acceleration));    // Equation 16
    // step numbers to the end of acceleration 

    if (distanceTo == 0 && stepsToStop <= 1)    { // We are at the target and its time to stop        
        _stepInterval = 0;
        _speed = 0.0;
        _n = 0;
        return;
    }

    if (distanceTo > 0)     {  // need to turn CW to reach target
        if (_n > 0)         {
            // Currently accelerating, need to decel now? Or maybe going the wrong way?
            if ((stepsToStop >= distanceTo) || _direction == DIRECTION_CCW)
                _n = -stepsToStop; // Start deceleration
        }
        else if (_n < 0)        {
            // Currently decelerating, need to accel again?
            if ((stepsToStop < distanceTo) && _direction == DIRECTION_CW)
                _n = -_n; // Start accceleration
        }
    }
    else if (distanceTo < 0)    { // need to turn CCW
        if (_n > 0)        {
            // Currently accelerating, need to decel now? Or maybe going the wrong way?
            if ((stepsToStop >= -distanceTo) || _direction == DIRECTION_CW)
             _n = -stepsToStop; // Start deceleration
        }
        else if (_n < 0)        {
            // Currently decelerating, need to accel again?
            if ((stepsToStop < -distanceTo) && _direction == DIRECTION_CCW)
             _n = -_n; // Start accceleration
        }
    }

    // Need to accelerate or decelerate
    if (_n == 0)   {
        // First step from stopped
        _cn = _c0;
        _direction = (distanceTo > 0) ? DIRECTION_CW : DIRECTION_CCW;
    }
    else    {
        // Subsequent step. Works for accel (n is +_ve) and decel (n is -ve).
        _cn = _cn - ((2.0f * _cn) / ((4.0f * _n) + 1)); // Equation 13
        _cn = max(_cn, _cmin); 
    }
    _n++;
    _stepInterval = _cn;
    _speed = 1000000.0f / _cn;
    if (_direction == DIRECTION_CCW)
        _speed = -_speed;

}

// Run the motor to implement speed and acceleration in order to proceed to the target position
// You must call this at least once per step, preferably in your main loop
// If the motor is in the desired position, the cost is very small
// returns true if the motor is still running to the target position.
bool mStepper::run() {
    if (runSpeed())
      computeNewSpeed();
    return _speed != 0.0f || distanceToGo() != 0;
}


 mStepper::mStepper(std::uint8_t interface, PinName pin1, PinName pin2, PinName pin3, PinName pin4, bool enable): _bo(pin1,pin2,pin3,pin4){
    _interface = interface;
    _currentPos = 0;
    _targetPos = 0;
    _speed = 0.0;
    _maxSpeed = 1.0;
    _acceleration = 0.0;
    _stepInterval = 0;
    _minPulseWidth = 1;
    _lastStepTime = 0;
    _n = 0;
    _c0 = 0.0;
    _cn = 0.0;
    _cmin = 1.0;
    _direction = DIRECTION_CCW;
   // _enablePin=NC;
    int i;
    for (i = 0; i < 4; i++)
    _pinInverted[i] = 0;
    setAcceleration(1); 
    t.start();
 }

void mStepper::setMaxSpeed(float speed) {
    if (_maxSpeed != speed)   {
        _maxSpeed = speed;
        _cmin = 1000000.0f / speed;
        // Recompute _n from current speed and adjust speed if accelerating or cruising
        if (_n > 0) {
            _n = (long)((_speed * _speed) / (2.0f * _acceleration)); // Equation 16
            computeNewSpeed();
        }
    }
}

void mStepper::setAcceleration(float acceleration){
    if (acceleration == 0.0f)
        return;
    if (_acceleration != acceleration)    {
        // Recompute _n per Equation 17
        _n = _n * (_acceleration / acceleration);
        // New c0 per Equation 7, with correction per Equation 15
        _c0 = 0.676f * sqrt(2.0f / acceleration) * 1000000.0f; // Equation 15
        _acceleration = acceleration;
        computeNewSpeed();
    }
}

void mStepper::setSpeed(float speed) {
    if (speed == _speed)
        return;
    speed = constrain(speed, -_maxSpeed, _maxSpeed);
    if (speed == 0.0f)
        _stepInterval = 0;
    else    {
        _stepInterval = fabs(1000000.0f / speed);
        _direction = (speed > 0.0f) ? DIRECTION_CW : DIRECTION_CCW;
    }
    _speed = speed;
}

float mStepper::speed() {
    return _speed;
}

// Subclasses can override
void mStepper::step(long step){
    switch (_interface)    {
    case FUNCTION:
        step0(step);
        break;

    case DRIVER:   //works
        step1(step);
        break;
    
    case FULL2WIRE:
        step2(step);
        break;
    
    case FULL3WIRE:
        step3(step);
        break;  

    case FULL4WIRE:  //works
        step4(step);
        break;  

    case HALF3WIRE:
        step6(step);
        break;  
        
    case HALF4WIRE:  //works
        step8(step);
        break;  
    }
}

// You might want to override this to implement eg serial output
// ....
void mStepper::setOutputPins(uint8_t mask) {
    _bo=mask;
}

// 0 pin step function (ie for functional usage)
void mStepper::step0(long step) {
  if (_speed > 0)
    _forward();
  else
    _backward();
}

// 1 pin step function (ie for stepper drivers)
// This is passed the current step number (0 to 7)
// Subclasses can override
void mStepper::step1(long step) {
    setOutputPins(_direction ? 0b10 : 0b00); // Set direction first else get rogue pulses
    setOutputPins(_direction ? 0b11 : 0b01); // step HIGH
    wait_us(_minPulseWidth);
    setOutputPins(_direction ? 0b10 : 0b00); // step LOW
}

// 2 pin step function
// This is passed the current step number (0 to 7)
// Subclasses can override
void mStepper::step2(long step) {
    switch (step & 0x3)    {
    case 0: /* 01 */
        setOutputPins(0b10);    break;
    case 1: /* 11 */
        setOutputPins(0b11);    break;
    case 2: /* 10 */ 
        setOutputPins(0b01);    break;
    case 3: /* 00 */
        setOutputPins(0b00);    break;
    }
}
// 3 pin step function
// This is passed the current step number (0 to 7)
// Subclasses can override
void mStepper::step3(long step) {
    switch (step % 3)     {
    case 0:    // 100
        setOutputPins(0b100);    break;
    case 1:    // 001
        setOutputPins(0b001);    break;
    case 2:    //010
        setOutputPins(0b010);    break;     
    }
}

// 4 pin step function for half stepper
// This is passed the current step number (0 to 7)
// Subclasses can override
void mStepper::step4(long step) {
    switch (step & 0x3)     {
    case 0:    // 0101
        setOutputPins(0b0101);      break;

    case 1:    // 0110
        setOutputPins(0b0110);      break;

    case 2:    //1010
        setOutputPins(0b1010);      break;

    case 3:    //1001
        setOutputPins(0b1001);      break;
    }
}

// 3 pin half step function
// This is passed the current step number (0 to 7)
// Subclasses can override
void mStepper::step6(long step)
{
    switch (step % 6)
    {
    case 0:    // 100
        setOutputPins(0b100);
            break;
        
        case 1:    // 101
        setOutputPins(0b101);
            break;
        
    case 2:    // 001
        setOutputPins(0b001);
            break;
        
        case 3:    // 011
        setOutputPins(0b011);
            break;
        
    case 4:    // 010
        setOutputPins(0b010);
            break;
        
    case 5:    // 011
        setOutputPins(0b110);
            break;
        
    }
}

// 4 pin half step function
// This is passed the current step number (0 to 7)
// Subclasses can override
void mStepper::step8(long step)
{
    switch (step & 0x7)
    {
    case 0:    // 1000
        setOutputPins(0b0001);
            break;
        
        case 1:    // 1010
        setOutputPins(0b0101);
            break;
        
    case 2:    // 0010
        setOutputPins(0b0100);
            break;
        
        case 3:    // 0110
        setOutputPins(0b0110);
            break;
        
    case 4:    // 0100
        setOutputPins(0b0010);
            break;
        
        case 5:    //0101
        setOutputPins(0b1010);
            break;
        
    case 6:    // 0001
        setOutputPins(0b1000);
            break;
        
        case 7:    //1001
        setOutputPins(0b1001);
            break;
    }
}
    
// Prevents power consumption on the outputs
void   mStepper::disableOutputs() {   
    if (! _interface) return;
    setOutputPins(0); // Handles inversion automatically
   if (*_enablePin)    {
        _enablePin->write(0^ _enableInverted );
    }
//
}

void   mStepper::enableOutputs() {
    if (! _interface) 
    return;
   if (*_enablePin)    {
        _enablePin->write(1^ _enableInverted );
    }

}

void mStepper::setMinPulseWidth(unsigned int minWidth){
    _minPulseWidth = minWidth;
}


void mStepper::setEnablePin(PinName enablePin){
    _enablePin = new DigitalOut(enablePin);
    // This happens after construction, so init pin now.
    if (*_enablePin)    {
        _enablePin->write(1^ _enableInverted );
    }
}

void mStepper::setPinsInverted(bool directionInvert, bool stepInvert, bool enableInvert){
    _pinInverted[0] = stepInvert;
    _pinInverted[1] = directionInvert;
    _enableInverted = enableInvert;
}

void mStepper::setPinsInverted(bool pin1Invert, bool pin2Invert, bool pin3Invert, bool pin4Invert, bool enableInvert){    
    _pinInverted[0] = pin1Invert;
    _pinInverted[1] = pin2Invert;
    _pinInverted[2] = pin3Invert;
    _pinInverted[3] = pin4Invert;
    _enableInverted = enableInvert;
}

// Blocks until the target position is reached and stopped  blocking!!!
void mStepper::runToPosition(){
    while (run()){}
}

bool mStepper::runSpeedToPosition(){
    if (_targetPos == _currentPos)
        return false;
    if (_targetPos >_currentPos)
        _direction = DIRECTION_CW;
    else
        _direction = DIRECTION_CCW;
    return runSpeed();
}

// Blocks until the new target position is reached
void mStepper::runToNewPosition(long position) {    //  blocking!!!
    moveTo(position);
    runToPosition();
}

void mStepper::stop(){
    if (_speed != 0.0f)     {    
        long stepsToStop = (long)((_speed * _speed) / (2.0f * _acceleration)) + 1; // Equation 16 (+integer rounding)
        if (_speed > 0)
            move(stepsToStop);
        else
            move(-stepsToStop);
    }
}

