#include <Wire.h>

#define AS5600_ADDRESS 0x36

#define AS5600_REG_RAW_ANGLE 0x0C

void setup() {
  Serial.begin(9600);
  Wire.begin();
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

  // Serial.print("Raw Angle: ");
  // Serial.print(rawAngle);
  // Serial.print(", Angle: ");
  Serial.print(angle);
  Serial.println(" ,");

  delay(100);
}

