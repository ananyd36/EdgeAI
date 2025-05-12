// Acknowledgments

// Creator: Anany Sharma at the University of Florida working under NSF grant. 2405373

// This material is based upon work supported by the National Science Foundation under Grant No. 2405373. 
// Any opinions, findings, and conclusions or recommendations expressed in this material are those of the authors and do not necessarily reflect the views of the National Science Foundation.


#include <GyverOLED.h>
GyverOLED<SSH1106_128x64> oled;



const int trigPin = 4 ;
const int echoPin = 2 ;

//define sound speed in cm/uS
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701

long duration;
float distanceCm;
float distanceInch;

void setup() {
  oled.init();  
  Serial.begin(115200); // Starts the serial communication
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
}

void loop() {
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
  
  // Convert to inches
  distanceInch = distanceCm * CM_TO_INCH;
  
  // Prints the distance in the Serial Monitor

  Serial.println(distanceCm);

   if (distanceCm < 4.0) {
      Serial.println("GLASS FILLED!");
      oled.clear();
      oled.setScale(1);
      oled.setCursor(0, 0); 
      oled.print("GLASS FILLED!"); // Top-left
      oled.update();
   }
   else {
      Serial.println("START FILLING!");
      oled.clear();
      oled.setScale(1);
      oled.setCursor(0, 0); 
      oled.print("START FILLING!"); // Top-left
      oled.update(); // move to next
   }
  
  delay(100);
}

// const int trigPin = 33;
// const int echoPin = 32;

// // Define sound speed in cm/uS
// #define SOUND_SPEED 0.034
// // #define CM_TO_INCH 0.393701 // Not used if printing smoothed Cm

// // --- Moving Average Definitions ---
// #define MVG_AVG_SIZE 20            // Number of samples to average over
// float readings[MVG_AVG_SIZE];      // Array to store the last N readings
// int readingIndex = 0;              // Index of the current reading
// float total = 0;                   // Running total
// float smoothedDistance = 0;        // Smoothed distance value
// int readingCount = 0;              // Counter for initial readings fill-up
// // --- End Moving Average Definitions ---

// long duration;
// float distanceCm;                  // Raw distance calculation
// // float distanceInch;             // Not used if printing smoothed Cm

// void setup() {
//   Serial.begin(115200); // Starts the serial communication
//   pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
//   pinMode(echoPin, INPUT); // Sets the echoPin as an Input

//   // Initialize the moving average buffer
//   for (int i = 0; i < MVG_AVG_SIZE; i++) {
//     readings[i] = 0; // Initialize buffer with 0s
//   }
//   Serial.println("Setup complete. Starting measurements...");
// }

// void loop() {
//   // --- Trigger the sensor ---
//   digitalWrite(trigPin, LOW);
//   delayMicroseconds(2);
//   digitalWrite(trigPin, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(trigPin, LOW);

//   // --- Read the echo ---
//   // Set a timeout for pulseIn (e.g., 30000 us corresponds roughly to 5m)
//   // helps prevent getting stuck if echo never returns
//   duration = pulseIn(echoPin, HIGH, 30000); // Added timeout

//   // --- Calculate the raw distance ---
//   if (duration > 0) {
//     distanceCm = duration * SOUND_SPEED / 2.0;
//     // Optional: Clamp distance to a realistic maximum range (e.g., 400 cm for HC-SR04)
//     // if (distanceCm > 400.0) {
//     //   distanceCm = 400.0; // Set to max reliable range if exceeded
//     // }
//   } else {
//     // Handle timeout or invalid reading (duration = 0)
//     // Option 1: Use a fixed large value (might skew average initially)
//     // distanceCm = 400.0; // Assign max range
//     // Option 2: Use the *previous* smoothed value (might be more stable)
//     distanceCm = smoothedDistance; // Re-use the last good average
//     // Option 3: Assign 0 (as original code did implicitly) - can pull average down
//     // distanceCm = 0;
//   }


//   // --- Update Moving Average ---
//   // Subtract the oldest reading from the total
//   total = total - readings[readingIndex];
//   // Add the new reading to the buffer
//   readings[readingIndex] = distanceCm;
//   // Add the new reading to the total
//   total = total + readings[readingIndex];
//   // Advance the index
//   readingIndex++;
//   // Wrap the index back to 0 if it reaches the end of the buffer
//   if (readingIndex >= MVG_AVG_SIZE) {
//     readingIndex = 0;
//   }

//   // Keep track of how many readings we have added, up to the buffer size
//   if (readingCount < MVG_AVG_SIZE) {
//      readingCount++;
//   }

//   // Calculate the smoothed distance
//   // Use readingCount for divisor during initial fill-up for better responsiveness
//   if (readingCount > 0) {
//       smoothedDistance = total / readingCount;
//   } else {
//       smoothedDistance = 0; // Should not happen after first loop, but safe fallback
//   }
//   // --- End Moving Average Update ---


//   // --- Print the smoothed distance ---
//   // Serial.print("Raw: "); Serial.print(distanceCm); // Optionally print raw value too
//   // Serial.print(" | Smoothed: ");
//   Serial.println(smoothedDistance); // Print the smoothed value

//   // --- Delay before next measurement ---
//   delay(100); // Maintain ~10Hz overall sample rate
// }