// Acknowledgments

// Creator: Anany Sharma at the University of Florida working under NSF grant. 2405373

// This material is based upon work supported by the National Science Foundation under Grant No. 2405373.
// Any opinions, findings, and conclusions or recommendations expressed in this material are those of the authors and do not necessarily reflect the views of the National Science Foundation.

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_APDS9960.h> // Include for APDS9960

// --- BME280 Sensor Definitions ---
#define SEALEVELPRESSURE_HPA (1013.25) // Standard sea-level pressure
Adafruit_BME280 bme; // I2C address 0x76 (default)
// For 0x77, use: Adafruit_BME280 bme(0x77);

// --- APDS9960 Sensor Definition ---
Adafruit_APDS9960 apds;

// --- Sampling Rate ---
const unsigned long SENSOR_READ_INTERVAL_MS = 500; // Sample every 500 milliseconds (2 Hz)

void setup() {
    Serial.begin(115200);
    while (!Serial); // Time to get serial running
    Serial.println(F("Starting Combined Sensor Test..."));

    // --- Initialize BME280 ---
    Serial.print("Initializing BME280 sensor...");
    bool bme_status = bme.begin(0x76); // Using the default I2C address 0x76
    if (!bme_status) {
        Serial.println("FAILED!");
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(), 16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        // Don't halt, let APDS9960 try to initialize even if BME fails
    } else {
        Serial.println("SUCCESS!");
    }

    // --- Initialize APDS9960 ---
    Serial.print("Initializing APDS9960 sensor...");
    if (!apds.begin()) {
        Serial.println("FAILED!");
        Serial.println("Failed to initialize APDS9960 device! Please check your wiring.");
        // Don't halt, let BME280 try to initialize even if APDS fails
    } else {
        Serial.println("SUCCESS!");
        // Enable color sensing mode for APDS9960
        apds.enableColor(true);
        Serial.println("APDS9960 Color sensing enabled.");
    }

    if (!bme_status && !apds.begin()) { // If both failed
        Serial.println("ERROR: Both BME280 and APDS9960 failed to initialize. Halting.");
        while(1) delay(10); // Halt if both fail
    }

    Serial.println("\n--- Sensor Readings ---");
    Serial.println("Temperature(C),Pressure(hPa),Humidity(%),Light_Red_Norm,Light_Green_Norm,Light_Blue_Norm,Light_Clear");
    Serial.println("--- Ready to stream data for Edge Impulse ---");
}

void loop() {
    static unsigned long lastSensorReadTime = 0;

    // Check if it's time to read sensors
    if (millis() - lastSensorReadTime >= SENSOR_READ_INTERVAL_MS) {
        lastSensorReadTime = millis();
        printCombinedSensorValues();
    }
}

void printCombinedSensorValues() {
    float temperature = NAN, pressure = NAN, humidity = NAN;
    uint16_t r = 0, g = 0, b = 0, c = 0;
    float r_norm = NAN, g_norm = NAN, b_norm = NAN;

    // --- Read BME280 Data ---
    // Ensure BME280 is valid before reading
    if (bme.sensorID() == 0x60 || bme.sensorID() == 0x56 || bme.sensorID() == 0x57 || bme.sensorID() == 0x58) { // Valid BME280/BMP280 IDs
        temperature = bme.readTemperature();
        pressure = bme.readPressure() / 100.0F; // Convert Pa to hPa
        humidity = bme.readHumidity();
    } else {
        // Sensor not initialized or invalid ID, values remain NAN
        Serial.print("BME280 not found/initialized. Skipping BME readings. ");
    }


    // --- Read APDS9960 Data ---
    if (apds.colorDataReady()) {
        apds.getColorData(&r, &g, &b, &c);

        // Normalize the RGB values only if clear channel is not zero
        if (c > 0) {
            r_norm = (float)r / c;
            g_norm = (float)g / c;
            b_norm = (float)b / c;
        } else {
            // If clear is 0, normalization is not possible. Set to NAN or 0.0
            r_norm = 0.0;
            g_norm = 0.0;
            b_norm = 0.0;
        }
    } else {
        // Sensor not ready or failed, values remain NAN
        Serial.print("APDS9960 color data not ready. Skipping APDS readings. ");
    }

    // --- Print Combined Values in CSV Format ---
    // Use isnan() to handle cases where a sensor failed to initialize or read
    // Edge Impulse can often handle NaN values by ignoring them or you can choose a default (e.g., 0.0 or -999)
    // For simplicity, we'll print what we get. If a sensor failed, it will print 'nan' or '0.0' for light norms if c=0.

    Serial.print(temperature, 2); // Print temperature with 2 decimal places
    Serial.print(",");
    Serial.print(pressure, 2);    // Print pressure with 2 decimal places
    Serial.print(",");
    Serial.print(humidity, 2);    // Print humidity with 2 decimal places
    Serial.print(",");
    Serial.print(r_norm, 4);      // Print normalized red with 4 decimal places
    Serial.print(",");
    Serial.print(g_norm, 4);      // Print normalized green with 4 decimal places
    Serial.print(",");
    Serial.print(b_norm, 4);      // Print normalized blue with 4 decimal places
    Serial.print(",");
    Serial.println(c);            // Print clear channel (raw)

    // Optional: Add a small delay if the loop runs too fast for the serial buffer
    // delay(10);
}