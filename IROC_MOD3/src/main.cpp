
#include <Arduino.h>

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




int speed=150;

void setup() {
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

void loop() {
  digitalWrite(m1_dir1,LOW);
  analogWrite(m1_pwm1, speed);
  digitalWrite(m1_dir2,HIGH);
  analogWrite(m1_pwm2, speed);

  digitalWrite(m2_dir1,HIGH);
  analogWrite(m2_pwm1, speed);
  digitalWrite(m2_dir2,LOW);
  analogWrite(m2_pwm2, speed);

  digitalWrite(m3_dir1,LOW);
  analogWrite(m3_pwm1, speed);
  digitalWrite(m3_dir2,HIGH);
  analogWrite(m3_pwm2, speed);

}