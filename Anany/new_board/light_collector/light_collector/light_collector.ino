// Acknowledgments

// Creator: Anany Sharma at the University of Florida working under NSF grant. 2405373

// This material is based upon work supported by the National Science Foundation under Grant No. 2405373. 
// Any opinions, findings, and conclusions or recommendations expressed in this material are those of the authors and do not necessarily reflect the views of the National Science Foundation.


#include "Adafruit_APDS9960.h"
Adafruit_APDS9960 apds;

void setup() {
  Serial.begin(115200);

  if(!apds.begin()){
    Serial.println("failed to initialize device! Please check your wiring.");
  }
  else Serial.println("Device initialized!");

  //enable color sensign mode
  apds.enableColor(true);
}

void loop() {
  uint16_t r, g, b, c;

  // Wait for color data to be ready
  while (!apds.colorDataReady()) {
    delay(5);
  }

  apds.getColorData(&r, &g, &b, &c);

  // Avoid division by zero
  if (c == 0) {
    return; // Skip this frame
  }

  // Normalize the RGB values
  float r_norm = (float)r / c;
  float g_norm = (float)g / c;
  float b_norm = (float)b / c;

  // Output normalized RGB for Edge Impulse
  Serial.print(r_norm, 4); Serial.print(",");
  Serial.print(g_norm, 4); Serial.print(",");
  Serial.print(b_norm, 4);Serial.print(",");
  Serial.print(c, 4);
  Serial.println();

  delay(20); // Adjust as needed
}
