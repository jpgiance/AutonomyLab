
#include "MPID.h"

#define ENCODER_CPR 646

volatile long encoderValueA = 0;
volatile long encoderValueB = 0;
double rpmA = 0;
double rpmB = 0;
int dirA = 0;
int dirB = 0;
double setPointA = 50;
double setPointB = 0;
double outputA = 0;
double outputB = 0;

// Motor A connections
int enA = 6;
int in1 = 4;
int in2 = 5;
// Motor A encoder
int encoderAPin1 = 10;
int encoderAPin2 = 2;
// Motor B connections
int enB = 11;
int in3 = 8;
int in4 = 9;
// Motor B encoder
int encoderBPin1 = 7;
int encoderBPin2 = 3;

// PID constants
double Kp = 2.0, Ki = 0.2, Kd = 0.0;

// Instantiate the PID controller
MPID motorAPID(&rpmA, &outputA, &setPointA, Kp, Ki, Kd);


void setup() {
  Serial.begin(9600);
  Serial.println("Encoder Test");


  // Encoders pins setup
  pinMode(encoderAPin2, INPUT_PULLUP);
  pinMode(encoderBPin1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderAPin2), updateEncoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderBPin2), updateEncoderB, RISING);

  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  setupTimer1();


  // Initialize PID controller
  motorAPID.setSampleTime(100);       // Sample time of 100ms
  motorAPID.setOutputLimits(0, 255);  // Output limits to match Arduino PWM range
}

void loop() {



  if (motorAPID.calculate()) {
    // Apply the PID output to the motor
    setMotorsSpeed(1, 1);

    // For debugging purposes
    // Serial.print("Setpoint: ");
    Serial.print(setPointA);
    Serial.print(" ");
    // Serial.print("\t Input: ");
    Serial.print(rpmA);
    Serial.print(" ");
    // Serial.print("\t Output: ");
    Serial.println(outputA);
  }














  // // Serial.print(dirA);
  // Serial.print(rpmA);
  // Serial.print("\t");
  // // Serial.print(dirB);
  // Serial.println(rpmB);

  // delay(500);
}

// This function tests both motors
void testMotors() {
  directionControl();
  delay(1000);
  speedControl();
  delay(1000);
}

void setMotorsSpeed(int directionA, int directionB) {

  analogWrite(enA, outputA);
  analogWrite(enB, outputB);

  if (directionA > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }

  if (directionB > 0) {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  } else {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
}

// This function lets you control spinning direction of motors
void directionControl() {
  // Set motors to maximum speed
  // For PWM maximum possible values are 0 to 255
  analogWrite(enA, 0);
  analogWrite(enB, 0);

  // Turn on motor A & B
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  delay(2000);

  // Now change motor directions
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  delay(2000);

  // Turn off motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

// This function lets you control speed of the motors
void speedControl() {
  // Turn on motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  // Accelerate from zero to maximum speed
  for (int i = 0; i < 256; i++) {
    analogWrite(enA, i);
    analogWrite(enB, i);
    delay(20);
  }

  // Decelerate from maximum speed to zero
  for (int i = 255; i >= 0; --i) {
    analogWrite(enA, i);
    analogWrite(enB, i);
    delay(20);
  }

  // Now turn off motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void calculateSpeed() {

  // Convert revolutions to RPM.
  rpmA = encoderValueA * 300 / ENCODER_CPR;
  encoderValueA = 0;

  rpmB = encoderValueB * 300 / ENCODER_CPR;
  encoderValueB = 0;
}

void setupTimer1() {
  noInterrupts();  // Disable all interrupts

  // Clear registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  // Set compare match register to desired timer count:
  // F_CPU / Prescaler / Desired frequency - 5
  // Example for 5Hz (5 times per second interval) with a 256 prescaler:
  // 16000000 / 256 / 5Hz - 1 = 12499
  OCR1A = 12499;  // Set compare match register
  // OCR1A = 62499; // Set compare match register

  // Turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12, CS11 and CS10 bits for 256 prescaler
  TCCR1B |= (1 << CS12);
  // Enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  interrupts();  // Enable all interrupts
}

ISR(TIMER1_COMPA_vect) {
  // Your periodic task here
  // Note: Keep this function short and efficient
  calculateSpeed();
}

void updateEncoderA() {
  int a = digitalRead(encoderAPin1);

  if (a > 0) {
    dirA = 1;
  } else {
    dirA = -1;
  }
  // increment value for each pulse
  encoderValueA++;
}

void updateEncoderB() {
  int a = digitalRead(encoderBPin1);

  if (a > 0) {
    dirB = 1;
  } else {
    dirB = -1;
  }

  // increment value for each pulse
  encoderValueB++;
}
