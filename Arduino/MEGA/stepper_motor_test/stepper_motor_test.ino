#include <AccelStepper.h>

// Define the pins for the TB6600 driver
#define DIR_PIN 16
#define PUL_PIN 17
#define EN_PIN 18

// Define motor interface type
#define motorInterfaceType 1

// Create an instance of the AccelStepper class
AccelStepper stepper(motorInterfaceType, PUL_PIN, DIR_PIN);

void setup() {
  // Set the enable pin as an output
  pinMode(EN_PIN, OUTPUT);
  
  // Enable the motor driver
  digitalWrite(EN_PIN, HIGH);

  // Set the maximum speed and acceleration
  stepper.setMaxSpeed(1000);  // Steps per second
  stepper.setAcceleration(500);  // Steps per second^2
}

void loop() {
  // Set the target position
  stepper.moveTo(2000);  // Move 2000 steps forward
  stepper.runToPosition();  // Blocking call, wait until motion is complete

  delay(1000);  // Wait for a second

  // Move back to the original position
  stepper.moveTo(0);  // Move 2000 steps backward
  stepper.runToPosition();  // Blocking call, wait until motion is complete

  delay(1000);  // Wait for a second
}