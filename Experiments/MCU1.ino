#include "Wire.h"

#define I2C_DEV_ADDR 0x55

uint32_t i = 0;
uint32_t x = 0;

void setup() {
  Serial.begin(115200);
  //Serial.setDebugOutput(true);
  Wire.begin();
}

void loop() {
  delay(5000);

  //Write message to the slave
  Wire.beginTransmission(I2C_DEV_ADDR);
  Wire.printf("Hello World! %u", i++);
  uint8_t error = Wire.endTransmission(true);
  Serial.printf("endTransmission: %u\n", x++);
  
}
