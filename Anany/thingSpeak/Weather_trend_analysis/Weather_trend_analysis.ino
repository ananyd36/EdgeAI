// Acknowledgments

// Creator: Anany Sharma at the University of Florida working under NSF grant. 2405373

// This material is based upon work supported by the National Science Foundation under Grant No. 2405373.
// Any opinions, findings, and conclusions or recommendations expressed in this material are those of the authors and do not necessarily reflect the views of the National Science Foundation.

#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h> // Needed for I2C communication
#include <Adafruit_Sensor.h> // Dependency for BME280
#include <Adafruit_BME280.h> // BME280 Library

// --- WiFi Credentials ---
const char* ssid = "Galaxy M14 5G DEA0"; // Your WiFi SSID
const char* password = "abc12345";       // Your WiFi password

String thingspeakApiKey = "9SV3ZA5W6LGWMP1X"; // Your ThingSpeak Write API Key
String serverName = "http://api.thingspeak.com/update"; // ThingSpeak API endpoint

Adafruit_BME280 bme; 


unsigned long lastTime = 0;
const unsigned long timerDelay = 2000; // 20 seconds delay (20000 milliseconds)

WiFiClient client;
HTTPClient http;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BME280 ThingSpeak Uploader...");

  Wire.begin();

  Serial.print("Initializing BME280 sensor...");
  bool bme_status = bme.begin(0x76); 
  if (!bme_status) {
    Serial.println("FAILED!");
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(), 16);
    Serial.println("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085");
    Serial.println("        ID of 0x56-0x58 represents a BMP 280,");
    Serial.println("        ID of 0x60 represents a BME 280.");
    Serial.println("        ID of 0x61 represents a BME 680.");
    while (1) delay(10); // Halt if BME280 fails to initialize
  } else {
    Serial.println("SUCCESS!");
  }

  // --- Connect to WiFi ---
  Serial.print("Connecting to WiFi network: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  Serial.print("Data will be sent to ThingSpeak every ");
  Serial.print(timerDelay / 1000);
  Serial.println(" seconds.");
}

void loop() {
  // Check if it's time to send a new HTTP request
  if ((unsigned long)(millis() - lastTime) >= timerDelay) {
    // Check WiFi connection status
    if (WiFi.status() == WL_CONNECTED) {
      // --- Read BME280 Data ---
      float temperature = bme.readTemperature(); // Celsius
      float pressure = bme.readPressure() / 100.0F; // Pa to hPa (millibars)
      float humidity = bme.readHumidity(); // %

      // Check if sensor returned valid data
      if (isnan(temperature) || isnan(pressure) || isnan(humidity)) {
        Serial.println("Error: Failed to read from BME280 sensor!");
      } else {
        // Print values to Serial Monitor for debugging
        Serial.print("Temperature: "); Serial.print(temperature); Serial.print(" C, ");
        Serial.print("Pressure: "); Serial.print(pressure); Serial.print(" hPa, ");
        Serial.print("Humidity: "); Serial.print(humidity); Serial.println(" %");

        String serverPath = serverName;
        serverPath += "?api_key=";
        serverPath += thingspeakApiKey;
        serverPath += "&field1=";
        serverPath += String(temperature);
        serverPath += "&field2=";
        serverPath += String(pressure);
        serverPath += "&field3=";
        serverPath += String(humidity);

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
      }
    } else {
      Serial.println("WiFi Disconnected. Reconnecting...");
      WiFi.begin(ssid, password); // Attempt to reconnect to WiFi
    }

    // Update lastTime for the next reading
    lastTime = millis();
  }
}