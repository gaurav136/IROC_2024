#include <Arduino.h>
#include <Wire.h>
#include <ArduinoJson.h>

#define IROC_MOD3 0x02

// #define MOTOR_PWM_PIN 5
// #define MOTOR_DIRECTION_PIN 4

void receiveEvent(int numBytes) {
  StaticJsonDocument<200> doc;

  // Allocate a buffer for the received data
  uint8_t buffer[numBytes];
  int i = 0;
  
  // Read data from I2C bus into the buffer
  while (Wire.available()) {
    buffer[i] = Wire.read();
    i++;
  }

  // Parse the received JSON from the buffer
  DeserializationError error = deserializeJson(doc, buffer);
  
  if (error) {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }
  
  

  int servoAngle = doc["Angle"];
  int ObjDis= doc["Distance"];
  Serial.print("Angel :");
  Serial.println(servoAngle);
  Serial.print("Distance :");
  Serial.println(ObjDis);

  // // Set PWM value for motor
  // analogWrite(MOTOR_PWM_PIN, pwmValue);
  
  // // Set direction for motor
  // digitalWrite(MOTOR_DIRECTION_PIN, direction);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(IROC_MOD3);
  Wire.onReceive(receiveEvent);

  // pinMode(MOTOR_PWM_PIN, OUTPUT);
  // pinMode(MOTOR_DIRECTION_PIN, OUTPUT);
}

void loop() {
  delay(100);
}
