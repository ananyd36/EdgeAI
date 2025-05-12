// --- Constants ---
const int buzzerPin = 26; // GPIO pin the buzzer signal is connected to

void setup() {
  Serial.begin(115200); // Start Serial communication for messages
  Serial.println("Buzzer Test 1: Simple On/Off (for Active Buzzers)");

  // Set the buzzer pin as an output
  pinMode(buzzerPin, OUTPUT);

  Serial.println("Starting test loop...");
}

void loop() {
  // Turn the buzzer ON
  for (int i =0 ; i < 50; i++)
  {
    digitalWrite(buzzerPin, HIGH);
    delay(5);
    digitalWrite(buzzerPin, LOW);
    delay(5);
  } 
  delay(1000);// Keep it off for 1.5 seconds
}