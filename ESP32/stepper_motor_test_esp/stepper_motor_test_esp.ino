// ESP32 Version

#include <AccelStepper.h>
#include <Wire.h>
#include "MPID.h"

#define AS5600_ADDRESS 0x36
#define AS5600_REG_RAW_ANGLE 0x0C

// Define the stepper motor pins
const int stepPin = 12;  // Step pin
const int dirPin = 14;   // Direction pin
const int enPin = 27;    // Enable pin

// AS5600 Magnetic sensor pins
const int I2C_SDA = 21;
const int I2C_SCL = 22;    

// Create an instance of the AccelStepper class
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

const int MIN_POS = 0;
const int MAX_POS = 2500;
int currentPosition = 0;

double currentAngle = 0;
double angleSetPoint = 180;
float angleOffset = 0;

// PID constants
double Kp = 50.0, Ki = 0.2, Kd = 0.0;
double outputPID = 0;
// Instantiate the PID controller
MPID pendulumPID(&currentAngle, &outputPID, &angleSetPoint, Kp, Ki, Kd);

bool reset = false;

void setup() {

  

  Serial.begin(9600);

  Serial.println("\n---- INVERTED PENDULUM PROGRAM ----\n");

  Wire.begin(I2C_SDA, I2C_SCL);  // Initialize I2C with specified SDA and SCL pins

  // Set the enable pin as an output
  pinMode(enPin, OUTPUT);
  
  // Enable the stepper motor
  digitalWrite(enPin, LOW);
  
  // Set the maximum speed and acceleration
  stepper.setMaxSpeed(15000);  // Set the maximum speed in steps per second
  stepper.setAcceleration(50000);  // Set the acceleration in steps per second^2

  // Initialize PID controller
  pendulumPID.setSampleTime(50);       // Sample time of 100ms
  pendulumPID.setOutputLimits(-300, 300);  // Output limits to match Arduino PWM range

  reset = true;

  calibrateAngle();

}

void loop() {


  if(reset){

    int midPosition = (MAX_POS - MIN_POS)/2;
    // Move the stepper motor in the positive direction
    stepper.moveTo(midPosition);  // Set the target position in steps
    stepper.runToPosition();  // Move the motor to the target position
    currentPosition = midPosition;
    reset = false;
    delay(1000);
  }


  getCurrentAngle();

  if(pendulumPID.calculate()){

    runPID();
  }


  
}

void runPID(){

  Serial.print("currentAngle: ");
  Serial.print(currentAngle);
  Serial.print(", output [steps]: ");
  Serial.println(outputPID);

  if(currentAngle > angleSetPoint && currentAngle < 225){
    Serial.println("    0 -->");

    _move(-outputPID);

  }else if(currentAngle < angleSetPoint && currentAngle > 135){
    Serial.println("<-- 0 ");
    _move(-outputPID);
  }
}

void run(){

  Serial.print("currentAngle: ");
  Serial.print(currentAngle);
  Serial.print(", angleSetPoint: ");
  Serial.print(angleSetPoint);
  Serial.println(" degrees");

  if(currentAngle > angleSetPoint && currentAngle < 225){
    Serial.println("    0 -->");

    _move(100);

  }else if(currentAngle < angleSetPoint && currentAngle > 135){
    Serial.println("<-- 0 ");
    _move(-100);
  }
}

void _move(int steps){

  Serial.println(steps);

  int newPosition = currentPosition + steps;
  if(newPosition > MIN_POS && newPosition < MAX_POS){
    // Move the stepper motor in the positive direction
    stepper.moveTo(newPosition);  // Set the target position in steps
    stepper.runToPosition();  // Move the motor to the target position
    currentPosition = newPosition;
  }else{
    reset = true;
  }

}


void getCurrentAngle(){
  // Request 2 bytes from the AS5600 raw angle register
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(AS5600_REG_RAW_ANGLE);
  Wire.endTransmission();
  
  Wire.requestFrom(AS5600_ADDRESS, 2);
  while (Wire.available() < 2);
  int highByte = Wire.read();
  int lowByte = Wire.read();

  // Combine the two bytes to get the raw angle value
  int rawAngle = (highByte << 8) | lowByte;

  // Convert raw angle to degrees (0 to 360) and Apply offset and ensure the angle stays within 0-360 degrees
  float adjustedAngle = (rawAngle * 360.0) / 4096.0 - angleOffset;
  if (adjustedAngle >= 360.0) {
    currentAngle = adjustedAngle - 360.0;
  } else if (adjustedAngle < 0.0) {
    currentAngle = adjustedAngle + 360.0;
  }else{
    currentAngle = adjustedAngle;
  }

  // Serial.print("Raw Angle: ");
  // Serial.print(rawAngle);
  // Serial.print(", Angle: ");
  // Serial.print(currentAngle);
  // Serial.println(" degrees");
}

void calibrateAngle(){
  // Request 2 bytes from the AS5600 raw angle register
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(AS5600_REG_RAW_ANGLE);
  Wire.endTransmission();
  
  Wire.requestFrom(AS5600_ADDRESS, 2);
  while (Wire.available() < 2);
  int highByte = Wire.read();
  int lowByte = Wire.read();

  // Combine the two bytes to get the raw angle value
  int rawAngle = (highByte << 8) | lowByte;
  
  // Convert raw angle to degrees (0 to 360)
  angleOffset = (rawAngle * 360.0) / 4096.0;

  

  Serial.print(" Angle Offset: ");
  Serial.println(angleOffset);
}

void testStepperMotor(){
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
  stepper.setAcceleration(100000);  // Set a new acceleration value
  
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
