#include <Arduino.h>
#include <Wire.h>
#include <ArduinoJson.h>


#define IROC_MOD2 0x02
#define IROC_MOD3 0x03

void setup() {
  Serial.begin(115200);
  Wire.begin();
}

void loop() {
  StaticJsonDocument<200> doc1;
  StaticJsonDocument<200> doc2;

  doc1["PWM"] = 255;
  doc1["Direction"] = 1;

  doc2["Angle"] = 90;
  doc2["Distance"] =10;

  // JsonArray data = doc.createNestedArray("data");
  // data.add(48.756080);
  // data.add(2.302038);

  size_t len1 = measureJson(doc1);
  size_t len2 = measureJson(doc2);
  uint8_t buffer1[len1];
  uint8_t buffer2[len2];

  serializeJson(doc1, buffer1, len1);
  serializeJson(doc2, buffer2, len2);

  Wire.beginTransmission(IROC_MOD2);
  Wire.write(buffer1, len1);
  Wire.endTransmission();

  Wire.beginTransmission(IROC_MOD3);
  Wire.write(buffer2, len2);
  Wire.endTransmission();

  delay(1000);
}
