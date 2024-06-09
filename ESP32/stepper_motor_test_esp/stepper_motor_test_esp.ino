#include <AccelStepper.h>

// Define the stepper motor pins
const int stepPin = 12;  // Step pin
const int dirPin = 14;   // Direction pin
const int enPin = 27;    // Enable pin

// Create an instance of the AccelStepper class
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

void setup() {
  // Set the enable pin as an output
  pinMode(enPin, OUTPUT);
  
  // Enable the stepper motor
  digitalWrite(enPin, HIGH);
  
  // Set the maximum speed and acceleration
  stepper.setMaxSpeed(1000);  // Set the maximum speed in steps per second
  stepper.setAcceleration(500);  // Set the acceleration in steps per second^2
}

void loop() {
  // Move the stepper motor in the positive direction
  stepper.moveTo(2000);  // Set the target position in steps
  stepper.runToPosition();  // Move the motor to the target position
  
  // Delay for a short period
  delay(1000);
  
  // Move the stepper motor in the negative direction
  stepper.moveTo(-2000);  // Set the target position in steps
  stepper.runToPosition();  // Move the motor to the target position
  
  // Delay for a short period
  delay(1000);
  
  // Change the acceleration
  stepper.setAcceleration(1000);  // Set a new acceleration value
  
  // Move the stepper motor in the positive direction with the new acceleration
  stepper.moveTo(4000);  // Set the target position in steps
  stepper.runToPosition();  // Move the motor to the target position
  
  // Delay for a short period
  delay(1000);
  
  // Move the stepper motor in the negative direction with the new acceleration
  stepper.moveTo(-4000);  // Set the target position in steps
  stepper.runToPosition();  // Move the motor to the target position
  
  // Delay for a short period
  delay(1000);
}
