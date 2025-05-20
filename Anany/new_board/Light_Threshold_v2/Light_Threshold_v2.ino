// Acknowledgments

// Creator: Anany Sharma at the University of Florida working under NSF grant. 2405373

// This material is based upon work supported by the National Science Foundation under Grant No. 2405373. 
// Any opinions, findings, and conclusions or recommendations expressed in this material are those of the authors and do not necessarily reflect the views of the National Science Foundation.


// Importing the necessary libraries
#include "Adafruit_APDS9960.h"
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789

#define TFT_CS   33  // Chip Select control pin
#define TFT_DC    25  // Data/Command select pin
#define TFT_RST   26  // Reset pin (or connect to RST, see below)

#define BLACK   0x0000
#define WHITE   0xFFFF
#define BLUE    0x001F
#define GREEN   0x07E0
#define RED     0xF800

//GyverOLED<SSH1106_128x64> display;
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

Adafruit_APDS9960 apds;

void setup() {

  Serial.begin(115200);
  tft.init(170,320);
  tft.setRotation(3);
  tft.fillScreen(BLACK);  


  if(!apds.begin()){
    Serial.println("failed to initialize device! Please check your wiring.");
  }
  else Serial.println("Device initialized!");

  //enable color sensing mode
  apds.enableColor(true);
}
int currentDisplayedLightState = -1; // Use -1 to indicate no state is initially displayed

void loop() {
  uint16_t r, g, b, c;

  // Wait for color data to be ready
  while (!apds.colorDataReady()) {
    delay(5);
  }

  apds.getColorData(&r, &g, &b, &c);

  float r_norm = (float)r / c;
  float g_norm = (float)g / c; 
  float b_norm = (float)b / c;

  if (c > 0) {
      Serial.print(r_norm, 4); Serial.print(",");
      Serial.print(g_norm, 4); Serial.print(",");
      Serial.print(b_norm, 4);Serial.print(",");
      Serial.print(c, 4);
      Serial.println();
  } else {
       Serial.println("c=0"); // Indicate very low light state
  }


  // --- Determine the current light state based on c ---
  // We'll use integers to represent the states:
  // 0: DARK (< 3)
  // 1: MEDIUM LIGHT (3 to 9)
  // 2: FULL LIGHT (>= 10)
  int newLightState;

  if (c < 3) {
    newLightState = 0; // DARK
  } else if (c >= 3 && c < 10) {
    newLightState = 1; // MEDIUM LIGHT
  } else {
    newLightState = 2; // FULL LIGHT
  }


  // --- Check if the displayed state needs to be updated ---
  if (newLightState != currentDisplayedLightState) {
    // The light state has changed, update the display

    // First, clear the area where the previous text was displayed.
    tft.fillRect(0, 0, tft.width(), 35, BLACK);
    tft.setTextSize(4);
    tft.setTextColor(WHITE);
    tft.setCursor(0, 0);

    // Print the message corresponding to the new state
    switch (newLightState) {
      case 0:
        tft.println("DARK");
        break;
      case 1:
        tft.println("MEDIUM LIGHT");
        break;
      case 2:
        tft.println("FULL LIGHT");
        break;
    }

    // Update the global variable to remember the state we just displayed
    currentDisplayedLightState = newLightState;
  }

  // Small delay is fine here as the display update is conditional
  delay(1);
}