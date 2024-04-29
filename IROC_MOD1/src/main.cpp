#include <Arduino.h>
const int motorPin1 = 25; // IN1 for L298N
const int motorPin2 = 26;  // IN2 for L298N
const int enablePin = 27; // Enable pin for L298N (used for speed control)

// Encoder pins
const int encoderPinA = 32; // Encoder A pin
const int encoderPinB = 33; // Encoder B pin

volatile long counter = 0; // Motor pulse counter
volatile long targetPulses = 0; // Target pulse count
bool motorRunning = false; // Motor running status

void encoder_isr();
void moveMotor(bool forward);
void stopMotor();

void setup() {
  Serial.begin(115200); // Initialize serial communication
  
  // Set up encoder pins with pull-up resistors
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  
  // Attach interrupts for encoder
  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoder_isr, CHANGE);

  // Set motor control pins as output
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);

  stopMotor(); // Ensure motor is stopped initially
}

void loop() {
  // Monitor serial input for new target pulses
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // Read the input string
    targetPulses = input.toInt(); // Convert the input to an integer
    
    // Determine motor direction and reset counter
    if (targetPulses > 0) {
      counter = 0; // Reset counter
      moveMotor(true); // Move motor forward
      motorRunning = true; // Motor is running
    } else if (targetPulses < 0) {
      counter = 0; // Reset counter
      moveMotor(false); // Move motor backward
      motorRunning = true; // Motor is running
    } else {
      stopMotor(); // Stop the motor if the target is zero
    }

    Serial.println(counter);
  }

  // Check if motor should be stopped
  if (motorRunning) {
    if ((targetPulses > 0 && counter >= targetPulses) ||
        (targetPulses < 0 && counter <= targetPulses)) {
      stopMotor(); // Stop the motor
      motorRunning = false; // Set motor running flag to false
    }
  }
}

void encoder_isr() {
  // ISR for handling encoder state changes
  if (digitalRead(encoderPinA) == digitalRead(encoderPinB)) {
    counter--; // Encoder moving backward
  } else {
    counter++; // Encoder moving forward
  }
}

void moveMotor(bool forward) {
  // Move the motor in the specified direction
  if (forward) {
    digitalWrite(motorPin1, HIGH); // Set direction
    digitalWrite(motorPin2, LOW); 
  } else {
    digitalWrite(motorPin1, LOW); 
    digitalWrite(motorPin2, HIGH); // Set opposite direction
  }

  analogWrite(enablePin, 255); // Set motor speed to maximum
}

void stopMotor() {
  // Stop the motor and disable the enable pin
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  digitalWrite(enablePin, LOW);
}