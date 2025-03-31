#include <Wire.h>
#include <MPU6050.h>
#include <anany_36-project-1_inferencing.h>
#include <GyverOLED.h>

MPU6050 mpu;
GyverOLED<SSH1106_128x64> display;

// Create a buffer for the features (size should match the number of features your model expects)
float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];

// Variables for label tracking
unsigned long labelStartTime = 0;
String currentLabel = "";
String lastLabel = "";
const unsigned long thresholdTime = 2000; 

void readAccelerometerData() {
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);

    for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += 3) {
        features[ix] = ax;
        features[ix + 1] = ay;
        features[ix + 2] = az;
        delay(10); // Assuming 100 Hz sampling
    }
}

int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
    memcpy(out_ptr, features + offset, length * sizeof(float));
    return 0;
}

void setup() {
    Serial.begin(115200);
    while (!Serial);

    // Initialize the MPU6050 sensor
    Wire.begin();
    mpu.initialize();
    display.init();

    Serial.println("Edge Impulse Inferencing Demo");
}

void loop() {
    readAccelerometerData();

    ei_printf("Edge Impulse standalone inferencing (Arduino)\n");

    if (sizeof(features) / sizeof(float) != EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        ei_printf("The size of your 'features' array is not correct. Expected %lu items, but had %lu\n",
                  EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, sizeof(features) / sizeof(float));
        delay(1000);
        return;
    }

    ei_impulse_result_t result = { 0 };

    signal_t features_signal;
    features_signal.total_length = sizeof(features) / sizeof(features[0]);
    features_signal.get_data = &raw_feature_get_data;

    // Run the classifier with the accelerometer data
    EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, false /* debug */);

    if (res != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", res);
        return;
    }


    ei_printf("run_classifier returned: %d\r\n", res);
    print_inference_result(result);

    delay(1000); // Delay to avoid spamming the serial monitor
}

void print_inference_result(ei_impulse_result_t result) {
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
}