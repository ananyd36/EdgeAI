#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <anany_36-project-1_inferencing.h>
#include <GyverOLED.h>

Adafruit_MPU6050 mpu;
GyverOLED<SSH1106_128x64> display;

#define FREQUENCY_HZ 50                         
#define INTERVAL_MS (1000 / (FREQUENCY_HZ + 1))  


float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
size_t feature_ix = 0;                               


// Variables for label tracking
static unsigned long last_interval_ms = 0;
unsigned long labelStartTime = 0;
String currentLabel = "";
String lastLabel = "";
const unsigned long thresholdTime = 2000; 


void setup() {
    Serial.begin(115200);
    while (!Serial);

    // Initialize the MPU6050 sensor
    Wire.begin();
    mpu.begin();
    display.init();
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);  



    Serial.println("Edge Impulse Inferencing Demo");
}

void loop() {
  sensors_event_t a, g, temp;  // Structs for sensor data

  // Check if it's time for the next sample
  if (millis() > last_interval_ms + INTERVAL_MS) {
    last_interval_ms = millis();  // Update timer

    mpu.getEvent(&a, &g, &temp);  // Get sensor data

    // Add acceleration data to the features buffer
    features[feature_ix++] = a.acceleration.x;
    features[feature_ix++] = a.acceleration.y;
    features[feature_ix++] = a.acceleration.z;

    // If buffer is full, perform inference
    if (feature_ix == EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
      Serial.println("Running the inference...");
      signal_t signal;
      ei_impulse_result_t result;

      // Convert features to a signal for inferencing
      int err = numpy::signal_from_buffer(features, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
      if (err != 0) {
        ei_printf("Failed to create signal from buffer (%d)\n", err);
        return;
      }

      // Run classification
      EI_IMPULSE_ERROR res = run_classifier(&signal, &result, true);
      if (res != 0) return;

    if (res != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", res);
        return;
    }


    ei_printf("run_classifier returned: %d\r\n", res);

    String newLabel = "";
    float highestProbability = 0.0;

    // Analyze the inference results to find the most probable label
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        if (result.classification[i].value > highestProbability) {
            highestProbability = result.classification[i].value;
            newLabel = String(result.classification[i].label);
        }
    }

    // Check if the new label is the same as the last label
    if (newLabel == lastLabel) {
        if (millis() - labelStartTime > thresholdTime) {
            currentLabel = newLabel;
        }
    } else {
        lastLabel = newLabel;
        labelStartTime = millis();
    }

    // Display the current label on the OLED if it has been consistent for more than 2 seconds
    display.clear();
    display.setScale(1);
    display.setCursor(0, 0);
    display.print("Current Motion: ");
    display.println(currentLabel.c_str());
    display.update();

    // Debug: Print the current label and the status
    Serial.print("Current Label: ");
    Serial.print(currentLabel);
    Serial.print(" | Most probable: ");
    Serial.println(newLabel);
    feature_ix = 0;
    }
}
}