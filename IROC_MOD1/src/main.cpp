#include<Arduino.h>
// Motor control pins for the L298N motor driver
const int motorPin1 = 18; // IN1 for L298N
const int motorPin2 = 19; // IN2 for L298N
const int enablePin = 5;  // Enable pin for L298N (used for speed control)

// Encoder pins
const int encoderPinA = 32; // Encoder A pin
const int encoderPinB = 33; // Encoder B pin

// PID parameters
float proportional = 0.5; // Proportional gain
float integral = 0.00001; // Integral gain
float derivative = 0.005; // Derivative gain

// PID control variables
float errorIntegral = 0; // Accumulated integral error
float previousError = 0; // Previous error value for derivative calculation
unsigned long previousTime = 0; // For calculating time difference
float controlSignal = 0; // Resulting control signal

// Motor control and setpoint
int targetPulses = 0; // Desired pulse count (setpoint)
bool motorRunning = false; // Motor status indicator
int stopThreshold = 20; // Threshold for stopping motor (deadband)
int minPWM = 30; // Minimum PWM value to ensure motor moves
int maxPWM = 200; // Maximum PWM value to prevent overspeeding

// Encoder position and direction
volatile int encoderPosition = 0;
volatile bool encoderDirection = true; // True for CW, False for CCW

void encoderISR();
void calculatePID(int currentPulses, int targetPulses);
void applyControlSignal();
void startMotor();
void stopMotor();

void setup() {
  Serial.begin(9600);

  // Set motor control pins as output
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);

  // Initialize motor control to off
  stopMotor();

  // Set encoder pins as input with interrupt
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), encoderISR, CHANGE);
}

void encoderISR() {
  // Determine direction based on encoder signals
  if (digitalRead(encoderPinA) == digitalRead(encoderPinB)) {
    encoderDirection = true; // CW
  } else {
    encoderDirection = false; // CCW
  }

  // Update encoder position based on direction
  if (encoderDirection) {
    encoderPosition++;
  } else {
    encoderPosition--;
  }
}

void loop() {
  if (motorRunning) {
    // Read the current position from the encoder
    int currentPulses = encoderPosition;

    // Determine the control signal using PID logic
    calculatePID(currentPulses, targetPulses);

    // Apply the control signal to the motor
    applyControlSignal();

    // Check if the motor has reached or surpassed the target
    if (abs(currentPulses - targetPulses) <= stopThreshold) { // If within deadband
      stopMotor(); // Stop the motor
      Serial.print("Target reached: ");
      Serial.println(currentPulses);
    }
  }

  // Check for new target pulses from Serial input
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // Read the input string
    int newTargetPulses = input.toInt(); // Convert the input to an integer

    if (newTargetPulses != 0) { // If it's non-zero
      encoderPosition = 0; // Reset encoder count
      targetPulses = newTargetPulses; // Set the new target
      startMotor(); // Start the motor
      Serial.print("Motor started. Target pulses: ");
      Serial.println(targetPulses);
    }
  }
}

void calculatePID(int currentPulses, int targetPulses) {
  // Time calculation
  unsigned long currentTime = micros(); // Current time in microseconds
  float deltaTime = (currentTime - previousTime) / 1000000.0; // Time difference in seconds
  previousTime = currentTime; // Update the previous time

  // Error calculation
  float error = targetPulses - currentPulses;

  // Proportional term
  float proportionalTerm = proportional * error;

  // Integral term
  errorIntegral += error * deltaTime;
  float integralTerm = integral * errorIntegral;

  // Derivative term
  float derivativeTerm = derivative * ((error - previousError) / deltaTime);
  previousError = error; // Store the error for next derivative calculation

  // Compute the control signal
  controlSignal = proportionalTerm + integralTerm + derivativeTerm;

  // Clamp control signal to avoid excessive PWM values
  if (controlSignal > maxPWM) {
    controlSignal = maxPWM;
  } else if (controlSignal < -maxPWM) {
    controlSignal = -maxPWM;
  }
}

void applyControlSignal() {
  int pwmValue = abs((int)controlSignal); // Get absolute PWM value
  if (pwmValue < minPWM) {
    pwmValue = minPWM; // Ensure minimum PWM to avoid stalling
  }

  if (controlSignal > 0) { // Forward direction
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
  } else { // Reverse direction
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
  }

  analogWrite(enablePin, pwmValue); // Apply PWM to control motor speed
}

void startMotor() {
  motorRunning = true; // Motor running status
  applyControlSignal(); // Apply the initial control signal
}

void stopMotor() {
  motorRunning = false; // Motor stopped
  digitalWrite(motorPin1, LOW); // Disable the motor
  digitalWrite(motorPin2, LOW);
  digitalWrite(enablePin, LOW); // Disable PWM output
}