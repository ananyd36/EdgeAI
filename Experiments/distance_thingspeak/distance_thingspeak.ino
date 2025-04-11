#include <WiFi.h>
#include <ThingSpeak.h>

const char* ssid = "Add your SSID";
const char* password = "Add your Wifi password";
const char* thingSpeakApiKey = "Get your API Write Key";
const unsigned long thingSpeakChannel = 0; //Change the zero for your ID channel NUMBER; 

const int trigPin = 33;  // Trigger pin of HC-SR04
const int echoPin = 32;  // Echo pin of HC-SR04

#define SOUND_SPEED 0.034

WiFiClient client; // Declare the 'client' variable

void setup() {
  Serial.begin(115200);
  delay(10);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  ThingSpeak.begin(client);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  long duration, distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * SOUND_SPEED/2;

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Send data to ThingSpeak
  ThingSpeak.setField(1, distance);
  int status = ThingSpeak.writeFields(thingSpeakChannel, thingSpeakApiKey);

  if (status == 200) {
    Serial.println("Data sent to ThingSpeak successfully.");
  } else {
    Serial.println("Failed to send data to ThingSpeak.");
  }

  delay(10000); // Send data every 10 seconds
}
