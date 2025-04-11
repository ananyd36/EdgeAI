
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ThingSpeak.h>
#include <WiFi.h>

// Replace with your network credentials
const char* ssid = "Your SSID";
const char* password = "Your password"; 

// Replace with your ThingSpeak settings
const char* thingSpeakApiKey = "Your API writing key";
const unsigned long thingSpeakChannel = 0; //Change the zero for your ID channel NUMBER; 

WiFiClient client;
Adafruit_BME280 bme;

void setup() {
  // Start serial communication
  Serial.begin(115200);

  // Initialize I2C communication with the BME280 sensor
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Initialize a connection to ThingSpeak
  ThingSpeak.begin(client);
}

void loop() {
  // Read sensor data
  float temperatureC = bme.readTemperature();
  float temperatureF = temperatureC * 9 / 5 + 32;
  float humidity = bme.readHumidity();
  float pressure = bme.readPressure() / 100.0F; // Convert Pa to hPa

  // Print data to the serial monitor
  Serial.print("Temperature (C): ");
  Serial.println(temperatureC);
  Serial.print("Temperature (F): ");
  Serial.println(temperatureF);
  Serial.print("Humidity (%): ");
  Serial.println(humidity);
  Serial.print("Pressure (hPa): ");
  Serial.println(pressure);

  // Send data to ThingSpeak
  ThingSpeak.setField(1, temperatureC);
  ThingSpeak.setField(2, humidity);
  ThingSpeak.setField(3, pressure);

  int status = ThingSpeak.writeFields(thingSpeakChannel, thingSpeakApiKey);

  if (status == 200) {
    Serial.println("Data sent to ThingSpeak successfully.");
  } else {
    Serial.println("Failed to send data to ThingSpeak.");
  }

  delay(10000); // Send data every 10 seconds
}

