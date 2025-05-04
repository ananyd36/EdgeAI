#include <Wire.h>
#include <GyverOLED.h>
#include <math.h> // Needed for sin()

// OLED display object
GyverOLED<SSH1106_128x64> display;

// Constants for the display
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// Input Pins
#define POT_PIN 27        // Analog input for potentiometer
#define LEFT_SWITCH_PIN 36 // Digital input for left switch
#define RIGHT_SWITCH_PIN 39 // Digital input for right switch

// ADC Resolution (for ESP32 default)
#define ADC_MAX 4095.0

// --- Global Variable ---
// Define the Y coordinate for the base digital level (both switches OFF)
// and the center of the analog wave. Derived from the original digital code.
const int BASE_Y_LEVEL = SCREEN_HEIGHT - 15;

void setup() {
  display.init();
  Serial.begin(115200);

  // Initialize inputs
  pinMode(POT_PIN, INPUT);
  pinMode(LEFT_SWITCH_PIN, INPUT_PULLUP); // Use internal pull-up resistor
  pinMode(RIGHT_SWITCH_PIN, INPUT_PULLUP); // Use internal pull-up resistor

  display.clear();
  display.setScale(1); // Use default text scale
  display.print("Analog/Digital Demo");
  display.update();
  delay(1000);
}

// Function to read the potentiometer value and map it to a desired range
float readPotentiometer(float minVal, float maxVal) {
  float sensorValue = analogRead(POT_PIN);
  // Map the ADC value (0-4095) to the desired range (minVal-maxVal)
  // Handle potential inversion if needed based on wiring (assuming higher voltage = higher value here)
   float mappedValue = minVal + (sensorValue / ADC_MAX) * (maxVal - minVal);
  // If your pot reads "backwards" (max value when you expect min), uncomment the line below
  // float mappedValue = minVal + ((ADC_MAX - sensorValue) / ADC_MAX) * (maxVal - minVal);
  return mappedValue;
}

// Function to draw a sinusoidal wave centered at a specific Y coordinate
// Amplitude here is the peak deviation from the center Y
void drawSineWave(float amplitude, float frequency, int centerY) {
  // Ensure amplitude isn't too large to go off-screen
  if (amplitude > centerY) amplitude = centerY;
  if (amplitude > (SCREEN_HEIGHT - 1 - centerY)) amplitude = SCREEN_HEIGHT - 1 - centerY;
  if (amplitude < 1) amplitude = 1; // Minimum visible amplitude

  for (int x = 0; x < SCREEN_WIDTH; x++) {
    // Calculate the y-value for the sine wave relative to the center Y
    float yValue = amplitude * sin(2.0 * PI * frequency * (float)x / SCREEN_WIDTH) + centerY;

    // Draw the point if it's within screen bounds
    if (yValue >= 0 && yValue < SCREEN_HEIGHT) {
      display.dot(x, (int)round(yValue), 1);
    }
  }
}

// Function to draw the digital step function line and associated text
void drawDigitalStep(bool leftState, bool rightState) {
  int level = 0; // 0 = Both OFF (base level handled by analog wave)
  if (leftState && !rightState) level = 1;  // Left ON only
  if (!leftState && rightState) level = 2;  // Right ON only
  if (leftState && rightState) level = 3;  // Both ON

  // Calculate the Y coordinate for the step function line based on level
  int yCoordinate;
  String levelText = "OFF";

  switch (level) {
    case 1:
      yCoordinate = SCREEN_HEIGHT - 15; // Level 1 slightly higher than base
      levelText = "LOW";
      break;
    case 2:
      yCoordinate = SCREEN_HEIGHT - 25; // Level 2 higher
      levelText = "MID"; // Or just "HIGH" if only two levels used?
      break;
    case 3:
      yCoordinate = SCREEN_HEIGHT - 35; // Level 3 highest
      levelText = "HIGH";
      break;
    default:
       // This case should technically not be hit if called correctly,
       // but default to base level if something goes wrong.
      yCoordinate = BASE_Y_LEVEL;
      break;
  }

  // Draw the horizontal line for the digital signal level
  for (int x = 0; x < SCREEN_WIDTH; x++) {
    display.dot(x, yCoordinate, 1);
  }

  // Display Status Text for Digital Mode
  display.setCursor(0, 0);
  display.print("Mode: Digital Step");
  display.setCursor(0, 1);
  display.print("Level: "); display.print(levelText);
  display.setCursor(0, 2);
  display.print("L:"); display.print(leftState ? "ON " : "OFF");
  display.print(" R:"); display.print(rightState ? "ON " : "OFF");

}

void loop() {
  // Read the switch states (Inverted due to INPUT_PULLUP)
  bool leftSwitchState = !digitalRead(LEFT_SWITCH_PIN);
  bool rightSwitchState = !digitalRead(RIGHT_SWITCH_PIN);

  // Clear the display buffer for the new frame
  display.clear();

  // --- Conditional Logic: Decide whether to show Analog or Digital ---
  if (!leftSwitchState && !rightSwitchState) {
    // --- ANALOG MODE: No switches pressed ---

    // Read amplitude & frequency from potentiometer
    // Adjust ranges as needed for good visualization around BASE_Y_LEVEL
    // Amplitude range should be small enough not to hit top/bottom easily
    float amplitude = readPotentiometer(1.0, (SCREEN_HEIGHT - BASE_Y_LEVEL) * 0.8); // e.g., 1 to 8 (since base is 10 from bottom)
    // Frequency: cycles across the screen width
    float frequency = readPotentiometer(0.5, 6.0); // e.g., 0.5 to 6 cycles

    // Display Status Text for Analog Mode
    display.setCursor(0, 0);
    display.print("Mode: Analog Wave");
    display.setCursor(0, 1); // Use second line for info
    display.print("Frequency: "); display.print(frequency, 1); display.print(" cycles"); // 1 decimal place

    // Draw the sine wave centered at the base digital level
    drawSineWave(amplitude, frequency, BASE_Y_LEVEL);

  } else {
    drawDigitalStep(leftSwitchState, rightSwitchState);

    // Serial Debugging (Optional)
    Serial.print("Digital - L: "); Serial.print(leftSwitchState);
    Serial.print(", R: "); Serial.println(rightSwitchState);
  }

  // Update the physical display with the buffer contents
  display.update();

  // Short delay for stability and responsiveness
  delay(20);
}