// Acknowledgments

// Creator: Anany Sharma at the University of Florida working under NSF grant. 2405373

// This material is based upon work supported by the National Science Foundation under Grant No. 2405373. 
// Any opinions, findings, and conclusions or recommendations expressed in this material are those of the authors and do not necessarily reflect the views of the National Science Foundation.



#include <Proximity_Motion_Detection_inferencing.h>                // Unified sensor interface used by Adafruit libraries
#include <GyverOLED.h>

const int trigPin = 33 ;
const int echoPin = 32 ;


//define sound speed in cm/uS
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701


long duration;
float distanceCm;

// === Define Sampling Parameters ===
#define FREQUENCY_HZ 9                          // Sampling rate: 60 times per second
#define INTERVAL_MS (1000 / (FREQUENCY_HZ + 1))  // Time between samples in milliseconds

// === Initialize Sensor and Buffer Variables ===
GyverOLED<SSH1106_128x64> display;

float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];  // Buffer to store sampled features
size_t feature_ix = 0;                               // Current index in the features buffer
static unsigned long last_interval_ms = 0;    
unsigned long labelStartTime = 0;
String currentLabel = "";
String lastLabel = "";
const unsigned long thresholdTime = 500; 

void setup() {
  Serial.begin(115200);  // Initialize serial communication at 115200 baud
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
    display.init();
  
  // Show Edge Impulse model input size and label count
  Serial.print("Features: ");
  Serial.println(EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
  Serial.print("Label count: ");
  Serial.println(EI_CLASSIFIER_LABEL_COUNT);
}

// === Main Loop: Runs Continuously ===
void loop() {

  // Check if enough time has passed to take a new sample
  if (millis() > last_interval_ms + INTERVAL_MS) {
    last_interval_ms = millis();  // Update timestamp
    // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  
  // Calculate the distance
  distanceCm = duration * SOUND_SPEED/2;

  // Store acceleration data into features buffer
  features[feature_ix++] = distanceCm;


    // If buffer is full, it's time to classify the motion
    if (feature_ix == EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
      Serial.println("Running the inference...");

      signal_t signal;               // Structure for the input signal to the model
      ei_impulse_result_t result;   // Structure for the classification result

      // Convert features buffer to signal format
      int err = numpy::signal_from_buffer(features, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
      if (err != 0) {
        ei_printf("Failed to create signal from buffer (%d)\n", err);
        return;  // Abort inference if signal couldn't be created
      }

      // Run the classifier with the prepared signal
      EI_IMPULSE_ERROR res = run_classifier(&signal, &result, true);
      if (res != 0) return;  // Abort if inference fails

      // Display timing and prediction results
      ei_printf("Predictions ");
      ei_printf("(DSP: %d ms., Classification: %d ms.)",
                result.timing.dsp, result.timing.classification);
      ei_printf(": \n");

      // Print the score for each possible class label
      for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);

        // (Optional) Store results in a char buffer if needed
        char buffer[50];
        sprintf(buffer, "    %s: %.5f", result.classification[ix].label, result.classification[ix].value);
        // You could send this buffer over WiFi or log it if needed
      }

    String newLabel = "Stationed";
    float highestProbability = 0.0;

    // Analyze the inference results to find the most probable label
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        float conf = result.classification[i].value;
            if (conf > 0.6) {
                newLabel = String(result.classification[i].label);
                highestProbability = conf;
                break;  
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
    String label = newLabel;


    if (distanceCm < 5) {
        label = "TOO CLOSE";
    }
    label.toUpperCase();    // Display the current label on the OLED if it has been consistent for more than 2 seconds
    display.clear();
    display.setScale(2);
    display.setCursor(0, 0);
    display.println(label);
    display.setCursor(0, 3);
    display.setScale(1);
    display.print("Confidence : ");
    display.print((highestProbability)*100);
    display.println("%");
    display.update();

    // Debug: Print the current label and the status
    Serial.print("Current Label: ");
    Serial.print(currentLabel);
    Serial.print(" | Most probable: ");
    Serial.println(newLabel);

      feature_ix = 0;  // Reset the buffer index for the next cycle
    }
  }
}

// === Helper Function to Print Formatted Output ===
void ei_printf(const char* format, ...) {
  static char print_buf[1024] = { 0 };  // Buffer for formatted output

  va_list args;
  va_start(args, format);  // Start capturing variable arguments
  int r = vsnprintf(print_buf, sizeof(print_buf), format, args);  // Format the string
  va_end(args);

  if (r > 0) {
    Serial.write(print_buf);  // Send output to the Serial Monitor
  }
}

