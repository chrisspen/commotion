#include <Wire.h>

#define COMMOTION_ADDR 0x1e

void setup() {
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output
}

void loop() {
  Wire.requestFrom(COMMOTION_ADDR, 1);    // request 6 bytes from slave device

  while (Wire.available()) { // slave may send less than requested
    byte c = Wire.read(); // receive a byte as character
    Serial.println(c);         // print the character
  }

  delay(500);
}
