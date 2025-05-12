// Acknowledgments

// Creator: Anany Sharma at the University of Florida working under NSF grant. 2405373

// This material is based upon work supported by the National Science Foundation under Grant No. 2405373. 
// Any opinions, findings, and conclusions or recommendations expressed in this material are those of the authors and do not necessarily reflect the views of the National Science Foundation.



#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C
unsigned long delayTime;

void setup() {
    Serial.begin(115200);
    while (!Serial);    // time to get serial running
    Serial.println(F("BME280 test"));

    bool status;
    
    // default settings with specific I2C address
    status = bme.begin(0x76);  
    // status = bme.begin(0x77); // Uncomment if your sensor uses the address 0x77
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(), 16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1) delay(10);
    }
    
    Serial.println("-- Default Test --");
    delayTime = 500; // Set to a higher frequency (sampling every 500 ms gives 2 Hz)

    Serial.println();
}

void loop() { 
    printValues();
    delay(delayTime);
}

void printValues() {
    float temperature = bme.readTemperature();
    float pressure = bme.readPressure() / 100.0F;
    float humidity = bme.readHumidity();

    // Check if the sensor is returning valid data
    if (isnan(temperature) || isnan(pressure) || isnan(humidity)) {
        Serial.println("Error reading sensor data");
    } else {
        // Print the values in CSV format
        Serial.print(temperature);
        Serial.print(",");
        Serial.print(pressure);
        Serial.print(",");
        Serial.println(humidity);
    }
}