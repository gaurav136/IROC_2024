
#include <Arduino.h>
#include <Wire.h>

#define I2C_DEV_ADDR 0x55

#define m1_dir1 32
#define m1_pwm1 33
#define m1_dir2 25
#define m1_pwm2 26

#define m2_dir1 27
#define m2_pwm1 14
#define m2_dir2 13
#define m2_pwm2 23

#define m3_dir1 19
#define m3_pwm1 18
#define m3_dir2 17
#define m3_pwm2 16

uint32_t i = 0;
int speed = 0;

void onRequest()
{
  Wire.print(i++);
  Wire.print(" Packets.");
  Serial.println("onRequest");
}

void onReceive(int len)
{
  Serial.printf("onReceive[%d]: ", len);
  while (Wire.available())
  {
    Serial.write(Wire.read());
  }
  Serial.println();
}

void forward()
{
  digitalWrite(m1_dir1, LOW);
  analogWrite(m1_pwm1, speed);
  digitalWrite(m1_dir2, HIGH);
  analogWrite(m1_pwm2, speed);

  digitalWrite(m2_dir1, HIGH);
  analogWrite(m2_pwm1, speed);
  digitalWrite(m2_dir2, LOW);
  analogWrite(m2_pwm2, speed);

  digitalWrite(m3_dir1, LOW);
  analogWrite(m3_pwm1, speed);
  digitalWrite(m3_dir2, HIGH);
  analogWrite(m3_pwm2, speed);
}

void backward()
{
  digitalWrite(m1_dir1, HIGH);
  analogWrite(m1_pwm1, speed);
  digitalWrite(m1_dir2, LOW);
  analogWrite(m1_pwm2, speed);

  digitalWrite(m2_dir1, LOW);
  analogWrite(m2_pwm1, speed);
  digitalWrite(m2_dir2, HIGH);
  analogWrite(m2_pwm2, speed);

  digitalWrite(m3_dir1, HIGH);
  analogWrite(m3_pwm1, speed);
  digitalWrite(m3_dir2, LOW);
  analogWrite(m3_pwm2, speed);
}

void rotate_left()
{
  digitalWrite(m1_dir1, HIGH);
  analogWrite(m1_pwm1, speed);
  digitalWrite(m1_dir2, HIGH);
  analogWrite(m1_pwm2, speed);

  digitalWrite(m2_dir1, LOW);
  analogWrite(m2_pwm1, speed);
  digitalWrite(m2_dir2, LOW);
  analogWrite(m2_pwm2, speed);

  digitalWrite(m3_dir1, HIGH);
  analogWrite(m3_pwm1, speed);
  digitalWrite(m3_dir2, HIGH);
  analogWrite(m3_pwm2, speed);
}

void rotate_right()
{
  digitalWrite(m1_dir1, LOW);
  analogWrite(m1_pwm1, speed);
  digitalWrite(m1_dir2, LOW);
  analogWrite(m1_pwm2, speed);

  digitalWrite(m2_dir1, HIGH);
  analogWrite(m2_pwm1, speed);
  digitalWrite(m2_dir2, HIGH);
  analogWrite(m2_pwm2, speed);

  digitalWrite(m3_dir1, LOW);
  analogWrite(m3_pwm1, speed);
  digitalWrite(m3_dir2, LOW);
  analogWrite(m3_pwm2, speed);
}

void setup()
{

  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);
  Wire.begin((uint8_t)I2C_DEV_ADDR);

#if CONFIG_IDF_TARGET_ESP32
  char message[64];
  snprintf(message, 64, "%lu Packets.", i++);
  Wire.slaveWrite((uint8_t *)message, strlen(message));
#endif

  pinMode(m1_dir1, OUTPUT);
  pinMode(m1_pwm1, OUTPUT);
  pinMode(m1_dir2, OUTPUT);
  pinMode(m1_pwm2, OUTPUT);

  pinMode(m2_dir1, OUTPUT);
  pinMode(m2_pwm1, OUTPUT);
  pinMode(m2_dir2, OUTPUT);
  pinMode(m2_pwm2, OUTPUT);

  pinMode(m3_dir1, OUTPUT);
  pinMode(m3_pwm1, OUTPUT);
  pinMode(m3_dir2, OUTPUT);
  pinMode(m3_pwm2, OUTPUT);
}

void loop()
{
  forward();
  delay(5000);
  backward();
  delay(5000);
  rotate_left();
  delay(5000);
  rotate_right();
  delay(5000);
}