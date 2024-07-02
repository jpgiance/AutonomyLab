// ESP32 Version

#include <AccelStepper.h>
#include <Wire.h>

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

void setup() {

  Serial.begin(9600);
  Wire.begin(I2C_SDA, I2C_SCL);  // Initialize I2C with specified SDA and SCL pins

  // Set the enable pin as an output
  pinMode(enPin, OUTPUT);
  
  // Enable the stepper motor
  digitalWrite(enPin, HIGH);
  
  // Set the maximum speed and acceleration
  stepper.setMaxSpeed(1000);  // Set the maximum speed in steps per second
  stepper.setAcceleration(50000);  // Set the acceleration in steps per second^2
}

void loop() {
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
  float angle = (rawAngle * 360.0) / 4096.0;

  Serial.print("Raw Angle: ");
  Serial.print(rawAngle);
  Serial.print(", Angle: ");
  Serial.print(angle);
  Serial.println(" degrees");

  delay(100);
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
