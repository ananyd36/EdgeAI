// Acknowledgments

// Creator: Anany Sharma at the University of Florida working under NSF grant. 2405373

// This material is based upon work supported by the National Science Foundation under Grant No. 2405373. 
// Any opinions, findings, and conclusions or recommendations expressed in this material are those of the authors and do not necessarily reflect the views of the National Science Foundation.


#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h> // Needed for I2C communication
#include <Adafruit_MPU6050.h> // MPU6050 Library
#include <Adafruit_Sensor.h> // Dependency for MPU6050 library

const char* ssid = "Galaxy M14 5G DEA0";
const char* password = "abc12345";


String serverName = "http://api.thingspeak.com/update?api_key=HN04JHLCFJQCDMMF";

unsigned long lastTime = 0;

const unsigned long timerDelay = 500; // 20 seconds delay
Adafruit_MPU6050 mpu; 

WiFiClient client;
HTTPClient http;

void setup() {
  Serial.begin(115200);
  Serial.println("MPU6050 ThingSpeak Uploader");

  Wire.begin(); 

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) { 
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // Optional: Set accelerometer range (MPU6050_RANGE_2_G, MPU6050_RANGE_4_G, MPU6050_RANGE_8_G, MPU6050_RANGE_16_G)
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:  Serial.println("+-2G"); break;
    case MPU6050_RANGE_4_G:  Serial.println("+-4G"); break;
    case MPU6050_RANGE_8_G:  Serial.println("+-8G"); break;
    case MPU6050_RANGE_16_G: Serial.println("+-16G"); break;
  } 

  // Optional: Set gyro range (MPU6050_RANGE_250_DEG, MPU6050_RANGE_500_DEG, MPU6050_RANGE_1000_DEG, MPU6050_RANGE_2000_DEG)
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  // Optional: Set filter bandwidth (MPU6050_BAND_260_HZ ... MPU6050_BAND_5_HZ)
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // --- Connect to WiFi ---
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi...");


  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("Connected to WiFi, IP Address: ");
  Serial.println(WiFi.localIP());

  // Initial message for clarity
  Serial.print("Timer set to ");
  Serial.print(timerDelay / 1000);
  Serial.println(" seconds. Will send data periodically.");
}

void loop() {
  // Check if it's time to send a new HTTP request
  if ((unsigned long)(millis() - lastTime) >= timerDelay) {

    if (WiFi.status() == WL_CONNECTED) {
      // --- Read accelerometer data from MPU6050 ---
      sensors_event_t a, g, temp; // Sensor event objects
      mpu.getEvent(&a, &g, &temp); // Read acceleration, gyro, and temperature data

      // Get accelerometer values
      float accelX = a.acceleration.x;
      float accelY = a.acceleration.y;
      float accelZ = a.acceleration.z;

      // Print values to Serial Monitor for debugging
      Serial.print(" Accel X: "); Serial.print(accelX);
      Serial.print(" m/s^2, Accel Y: "); Serial.print(accelY);
      Serial.print(" m/s^2, Accel Z: "); Serial.print(accelZ);
      Serial.println(" m/s^2");
      // You could also read g.gyro.x, g.gyro.y, g.gyro.z for gyroscope data
      // and temp.temperature for temperature data if needed.

      // --- Construct the server path with accelerometer data ---
      // Use field1 for Accel X, field2 for Accel Y, field3 for Accel Z
      String serverPath = serverName +
                          "&field1=" + String(accelX) +
                          "&field2=" + String(accelY) +
                          "&field3=" + String(accelZ);
      // If you want to send gyro or temp, add more fields (e.g., &field4=..., &field5=...)
      // Ensure your ThingSpeak channel is configured with enough fields!

      // --- Send HTTP GET request ---
      http.begin(client, serverPath.c_str()); // Use client object for ESP32 stability
      Serial.print("Sending data to ThingSpeak: ");
      Serial.println(serverPath); // Print the URL being sent

      int httpResponseCode = http.GET(); // Send the request

      if (httpResponseCode > 0) {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
        String payload = http.getString(); // Get the response payload (usually just an entry number)
        Serial.print("ThingSpeak Response: ");
        Serial.println(payload);
      } else {
        Serial.print("Error sending HTTP GET request, Error code: ");
        Serial.println(httpResponseCode); // Error code if request failed
      }

      // End the HTTP request and free resources
      http.end();

    } else {
      Serial.println("WiFi Disconnected. Trying to reconnect...");
      // Optional: Add WiFi reconnection logic here if needed,
      // otherwise it will just try again on the next timer interval.
      // WiFi.begin(ssid, password); // Simple reconnect attempt
    }

    // Update lastTime to manage the delay for the next reading
    lastTime = millis();
  }
  // No delay() here, loop runs continuously checking the timer
}