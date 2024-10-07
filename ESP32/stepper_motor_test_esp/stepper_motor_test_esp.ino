// ESP32 Version

#include <AccelStepper.h>
#include <Wire.h>
#include <MPID.h>

#define AS5600_ADDRESS 0x36
#define AS5600_REG_RAW_ANGLE 0x0C

// Define the stepper motor pins
const int stepPin = 12;  // Step pin
const int dirPin = 14;   // Direction pin
const int enPin = 27;    // Enable pin

const int calibrateTargetAngle = 26;  //
const int disableMotor = 25;  // 

const int trigPin = 2;
const int echoPin = 4;

// AS5600 Magnetic sensor pins
const int I2C_SDA = 21;
const int I2C_SCL = 22;    

// Create an instance of the AccelStepper class
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

const int microStepping = 1;
const int MIN_POS = 0;
const int MAX_POS = 2800*microStepping;
int midPosition = (MAX_POS - MIN_POS)/2;


double currentAngle = 0;
double angleSetPoint = 180;
float angleOffset = 0;

// PID Angle constants
double Kp = 20.0, Ki = 0.0, Kd = 0.0;
double outputPID = 0;

// Instantiate the PID controller
MPID pendulumPID(&currentAngle, &outputPID, &angleSetPoint, Kp, Ki, Kd);

// PID Position constants
// double Kp_pos = 0.1, Ki_pos = 0.01, Kd_pos = 0.001;
// double outputPID_pos = 0;

// Instantiate the PID controller
// MPID positionPID(&currentPosition, &outputPID_pos, &positionSetPoint, Kp_pos, Ki_pos, Kd_pos);

static volatile bool reset = false;
static volatile bool canRun = false;








int readTimeBuff[1000];
int timeBuffIndex = 0;
int count = 0;
int countSensor = 0;

unsigned long startMillis = 0;


QueueHandle_t stepsToMoveQueue;
QueueHandle_t currentPositionQueue;








void moveMotor(void *parameters){

  int currentPosition;
  int stepsToMove;

  // Loop forever
  while (1) {


    if(reset){

      Serial.println("------- Start Reset --------");
      
      // Move the stepper motor in the positive direction
      stepper.moveTo(midPosition);  // Set the target position in steps
      stepper.runToPosition();  // Move the motor to the target position
      xQueueOverwrite(currentPositionQueue, &midPosition);

      // int currentPos;

      // if(xQueuePeek(currentPositionQueue, &currentPos, 0) == pdTRUE){

      //   int stepsNeededToMoveAtMidPosition;

      //   if(currentPos > midPosition){
      //     stepsNeededToMoveAtMidPosition = currentPos - midPosition;
      //   }else{
      //     stepsNeededToMoveAtMidPosition = midPosition - currentPos;
      //   }

      //   xQueueOverwrite(stepsToMoveQueue, &stepsNeededToMoveAtMidPosition);

      //   reset = false;
      //   delay(1000);

      // }

      reset = false;
      delay(1000);

      Serial.println("------- End Reset --------");


    }


    if(xQueuePeek(stepsToMoveQueue, &stepsToMove, 0) == pdTRUE && xQueuePeek(currentPositionQueue, &currentPosition, 0) == pdTRUE && canRun) {
      // Use the value

      int newPosition = currentPosition + stepsToMove;

      if(newPosition != currentPosition){
        
        if(newPosition > MIN_POS && newPosition < MAX_POS){
          // Move the stepper motor in the positive direction
          stepper.moveTo(newPosition);  // Set the target position in steps
          stepper.runToPosition();  // Move the motor to the target position

          xQueueOverwrite(currentPositionQueue, &newPosition);
          
        }else{
          reset = true;
        }

      }
      
            
    }

    vTaskDelay(1);

  }

}







void setup() {

  

  Serial.begin(115200);

  Serial.println("\n---- INVERTED PENDULUM PROGRAM ----\n");

  pinMode(calibrateTargetAngle, INPUT_PULLUP);
  pinMode(disableMotor, INPUT_PULLUP);

  Wire.begin(I2C_SDA, I2C_SCL);  // Initialize I2C with specified SDA and SCL pins

  // Set the enable pin as an output
  pinMode(enPin, OUTPUT);
  
  // Enable the stepper motor
  digitalWrite(enPin, LOW);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  digitalWrite(trigPin, LOW);
  
  // Set the maximum speed and acceleration
  stepper.setMaxSpeed(4000*microStepping);  // Set the maximum speed in steps per second    // 4000
  stepper.setAcceleration(40000*microStepping);  // Set the acceleration in steps per second^2   //40000

  // Initialize PID controller
  pendulumPID.setSampleTime(1);       // Sample time of 100ms
  pendulumPID.setOutputLimits(-800, 800);  // Output limits to match Arduino PWM range

  // positionPID.setSampleTime(100);       // Sample time of 100ms
  // positionPID.setOutputLimits(-50, 50);  // Output limits to match Arduino PWM range

  reset = true;

  calibrateAngle0();

  stepsToMoveQueue = xQueueCreate(1, sizeof(int));
  currentPositionQueue = xQueueCreate(1, sizeof(int));

  xTaskCreatePinnedToCore(moveMotor,
                        "Move motor",
                        1024,
                        NULL,
                        1,
                        NULL,
                        0);

}

void loop() {

  // if(timeBuffIndex > 998){
  //   timeBuffIndex = 0;

  //   for (int i = 0; i<1000; i++) {
  //     Serial.println(readTimeBuff[i]);
  //   }
  // }

  unsigned long currentMillis = millis();

  if (currentMillis - startMillis >= 100) {
      // Read the state of the buttons
    int buttonState25 = digitalRead(disableMotor);
    int buttonState26 = digitalRead(calibrateTargetAngle);

    if(!buttonState25){
      calibrateAngle180();
    }

    startMillis = currentMillis;
    Serial.print(buttonState25);
    Serial.print(", ");
    Serial.print(buttonState26);
    Serial.print(", ");
    Serial.print(currentAngle);
    Serial.print(", ");
    Serial.print(count);
    Serial.print(", ");
    Serial.println(countSensor);
    // Serial.print("main loop on core:  ");
    // Serial.println(xPortGetCoreID());
    count = 0;
    countSensor = 0;

    readDistance();
  }




  getCurrentAngle();

  // if(pendulumPID.calculate() || positionPID.calculate()){
  if(pendulumPID.calculate() ){

    count++;

    if(canRun){
      runPID();
    }

  }


  
}

void runPID(){
  

  // Serial.print("currentAngle: ");
  // Serial.print(angleSetPoint);
  // Serial.print(", ");
  // Serial.println(currentAngle);
  // Serial.print(", output [steps]: ");
  // Serial.println(outputPID);



  // if(currentAngle > angleSetPoint && currentAngle < 205){

    // _move(-(outputPID*microStepping)-(outputPID_pos*microStepping));
    _move(-(outputPID*microStepping));

  // }else if(currentAngle < angleSetPoint && currentAngle > 155){
  //   _move(-outputPID);
  // }
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

  xQueueOverwrite(stepsToMoveQueue, &steps);


  // int newPosition = currentPosition + steps;
  // if(newPosition > MIN_POS && newPosition < MAX_POS){
  //   // Move the stepper motor in the positive direction
  //   stepper.moveTo(newPosition);  // Set the target position in steps
  //   stepper.runToPosition();  // Move the motor to the target position
  //   currentPosition = newPosition;
  // }else{
  //   reset = true;
  // }

}


void readDistance(){

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  long distance = duration * 0.034 / 2;

  Serial.print("Distance:");
  Serial.println(distance);
}


void getCurrentAngle(){

  // startMillis = millis();
  
  // Request 2 bytes from the AS5600 raw angle register
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(AS5600_REG_RAW_ANGLE);
  Wire.endTransmission();
  
  Wire.requestFrom(AS5600_ADDRESS, 2);
  while (Wire.available() < 2);
  int highByte = Wire.read();
  int lowByte = Wire.read();

  countSensor++;

  // readTimeBuff[timeBuffIndex++] = millis() - startMillis;

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

  currentAngle = currentAngle;     // -7.5

  // Serial.print("Raw Angle: ");
  // Serial.print(rawAngle);
  // Serial.print(", Angle: ");
  // Serial.print(currentAngle);
  // Serial.println(" degrees");

  // Serial.print(angleSetPoint);
  // Serial.print(", ");
  // Serial.println(currentAngle);

  if(abs(angleSetPoint - currentAngle) < 2){
    canRun = true;
  }

  if(abs(angleSetPoint - currentAngle) > 25){
    canRun = false;
  }
}

void calibrateAngle0(){
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

void calibrateAngle180(){
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
  angleOffset = (rawAngle * 360.0) / 4096.0 - 180;

  

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
