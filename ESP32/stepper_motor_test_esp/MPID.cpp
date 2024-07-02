#include "MPID.h"
#include "Arduino.h"

MPID::MPID(double* input, double* output, double* setpoint, double Kp, double Ki, double Kd) {
  this->_input = input;
  this->_output = output;
  this->_setpoint = setpoint;

  // Set PID coefficients
  _kp = Kp;
  _ki = Ki;
  _kd = Kd;

  // Call init to set up initial conditions and defaults
  init();
}

void MPID::init() {
  // Initialize PID controller variables
  _lastInput = *_input;
  _integral = 0;
  _outputSum = 0; // Reset the output sum to 0 for a clean start
  _previousError = 0;
  
  // Set default output limits to match Arduino PWM limits
  setOutputLimits(0, 255);
  
  // Set default Controller Sample Time to 0.1 seconds (100 milliseconds)
  _sampleTime = 100;
  
  // Initialize _lastTime to ensure the first call to calculate() runs immediately
  _lastTime = millis() - _sampleTime;
  
  // Ensure the initial output and integral are within the specified limits
  if (*_output > _outMax) *_output = _outMax;
  else if (*_output < _outMin) *_output = _outMin;
  
  if (_outputSum > _outMax) _outputSum = _outMax;
  else if (_outputSum < _outMin) _outputSum = _outMin;
}

bool MPID::calculate() {
  if ((millis() - _lastTime) >= _sampleTime) {

    double input = *this->_input;
    double error = *this->_setpoint - input;

    // integral
    _integral += _ki * error * (_sampleTime / 1000.0); // Integrate error over time

    // Prevent integral windup
    if (_integral > _outMax) _integral = _outMax; 
    else if (_integral < _outMin) _integral = _outMin;

    double dInput = input - _lastInput;
    /*
    *   Proportional = _kp * error
    *   Integral = _ki * error * (_sampleTime / 1000.0)   [using += for acumulation]
    *   Derivative = _kd * dInput / (_sampleTime / 1000.0)
    */
    double output = _kp * error + _integral - _kd * dInput / (_sampleTime / 1000.0);

    if (output > _outMax) output = _outMax; // Clamp output
    else if (output < _outMin) output = _outMin;

    *this->_output = output;

    _lastInput = input;
    _lastTime = millis();

    // Serial.print("min: ");
    // Serial.print(_outMin);
    // Serial.print("   max: ");
    // Serial.println(_outMax);
    return true; // Indicate a new output value is computed
  }
  return false; // No computation done
}

void MPID::setConstants(double Kp, double Ki, double Kd) {
  double sampleTimeInSec = ((double)_sampleTime) / 1000;
  _kp = Kp;
  _ki = Ki * sampleTimeInSec;
  _kd = Kd / sampleTimeInSec;
}

void MPID::setSampleTime(int NewSampleTime) {
  if (NewSampleTime > 0) {
    double ratio = (double)NewSampleTime / (double)_sampleTime;
    _ki *= ratio;
    _kd /= ratio;
    _sampleTime = (unsigned long)NewSampleTime;
  }
}

void MPID::setOutputLimits(double Min, double Max) {
  if (Min >= Max) return;
  _outMin = Min;
  _outMax = Max;

  if (*_output > _outMax) *_output = _outMax;
  else if (*_output < _outMin) *_output = _outMin;

  if (_outputSum > _outMax) _outputSum = _outMax;
  else if (_outputSum < _outMin) _outputSum = _outMin;
}
