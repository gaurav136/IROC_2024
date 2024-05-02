#include <Arduino.h>
#include <Wire.h>
#include <ArduinoJson.h>

#define I2C_DEV_ADDR 0x55

void setup() {
  Serial.begin(115200);
  Wire.begin();
}

void loop() {
  StaticJsonDocument<200> doc;

  doc["sensor"] = "temperature";
  doc["value"] = random(20, 30);

  // JsonArray data = doc.createNestedArray("data");
  // data.add(48.756080);
  // data.add(2.302038);

  size_t len = measureJson(doc);
  uint8_t buffer[len];
  serializeJson(doc, buffer, len);

  Wire.beginTransmission(I2C_DEV_ADDR);
  Wire.write(buffer, len);
  Wire.endTransmission();

  delay(1000);
}
