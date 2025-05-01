#define blue 18
#define red 5
#define green 19
#define leftbutton 36
#define rightbutton 39
#include "Wire.h"
#include <GyverOLED.h>
#include <math.h>
GyverOLED<SSH1106_128x64> oled;

const int trigPin = 33;
const int echoPin = 32;
char left_end[] = "Knob is at 100%";
char right_end[] = "Knob is at 0%";

float floatMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//define sound speed in cm/uS
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701

long duration;
float distanceCm;
float distanceInch;

void setup() {
  pinMode(blue, OUTPUT);
  pinMode(red, OUTPUT);
  pinMode(green, OUTPUT);
  pinMode(leftbutton, INPUT);
  pinMode(rightbutton, INPUT);

  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT); 
  
  Serial.begin(115200);
  Wire.begin();

  //I2C scan
  byte error, address;
  int nDevices = 0;
  delay(5000);
  Serial.println("Scanning for I2C devices ...");
  for(address = 0x01; address < 0x7f; address++){
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0){
      Serial.printf("I2C device found at address 0x%02X\n", address);
      nDevices++;
    } else if(error != 2){
      Serial.printf("Error %d at address 0x%02X\n", error, address);
    }
  }
  if (nDevices == 0){
    Serial.println("No I2C devices found");
  }  

 //Distance
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
          Serial.print("Distance (cm): ");
          Serial.println(distanceCm);
          Serial.print("Distance (inch): ");
          Serial.println(distanceInch);
          
          delay(1000);

          oled.init();
          
  }

void loop() {
   //LED test
           if (digitalRead(leftbutton) && digitalRead(rightbutton) == HIGH) {
            digitalWrite(blue, LOW);
              }
          else {
            digitalWrite(blue, HIGH);
           }
          
          if (digitalRead(leftbutton) == HIGH) {
            digitalWrite(red, HIGH);
              }
          else {
            digitalWrite(red, LOW);
           
          }
           if (digitalRead(rightbutton) == HIGH) {
            digitalWrite(green, HIGH);
              }
          else {
            digitalWrite(green, LOW);
          
          }
  //OLED
          
      int analogValue = analogRead(27);
  // Rescale to potentiometer's voltage (from 0V to 3.3V):
  float voltage = floatMap(analogValue, 0, 4095, 0, 3.3);

  // print out the value you read:
  Serial.print("Analog: ");
  Serial.print(analogValue);
  Serial.print(", Voltage: ");
  Serial.println(voltage);
  if (analogValue == 0) {
            showText();
              }
  else if (analogValue == 4095) {
         scaleText();
           }
}

void showText() {
  oled.clear();
  oled.home();
  oled.autoPrintln(true);
  oled.setScale(2);
  oled.print(left_end);
  oled.update();
 }

void scaleText() {
  oled.clear();
  oled.home();
  oled.autoPrintln(true);
  oled.setScale(2);
  oled.print(right_end);
  oled.update();
  }
