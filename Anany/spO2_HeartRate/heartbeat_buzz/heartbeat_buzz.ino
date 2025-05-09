#include <Wire.h>
#include "MAX30105.h"   // sparkfun MAX3010X library
#include <GyverOLED.h>

GyverOLED<SSH1106_128x64> oled;
MAX30105 particleSensor;

#include "arduinoFFT.h"
arduinoFFT FFT;

// --- Buzzer Configuration ---
const int buzzerPin = 26; // GPIO pin the buzzer signal is connected to
// --------------------------

bool showWait = true;
double avered     = 0;
double aveir      = 0;
double sumirrms   = 0;
double sumredrms  = 0;
int    i          = 0;
int    Num        = 100;  // calculate SpO2 by this sampling interval (Not used in BPM part)
int    Temperature;
int    temp;
float  ESpO2;           // initial value of estimated SpO2 (Not used in BPM part)
double FSpO2      = 0.7;  // filter factor for estimated SpO2 (Not used in BPM part)
double frate      = 0.95; // low pass filter for IR/red LED value to eliminate AC component (Not used in BPM part)
#define TIMETOBOOT    3000  // wait for this time(msec) to output SpO2 (Not used in BPM part)
#define SCALE         88.0  // adjust to display heart beat and SpO2 in the same scale (Not used in BPM part)
#define SAMPLING      100   //25 //5     // if you want to see heart beat more precisely, set SAMPLING to 1 (Not used in BPM part)
#define FINGER_ON     30000 // if red signal is lower than this, it indicates your finger is not on the sensor
#define USEFIFO
#define PULSE_SAMPLES 256
// Acknowledgments

// Creator: Anany Sharma at the University of Florida working under NSF grant. 2405373

// This material is based upon work supported by the National Science Foundation under Grant No. 2405373. 
// Any opinions, findings, and conclusions or recommendations expressed in this material are those of the authors and do not necessarily reflect the views of the National Science Foundation.


#define SAMPLE_FREQ   50

// --- For Heart Rate / Buzzer Timing ---
double beatsPerMinute = 0;
unsigned long lastBuzzTime = 0; // Time the last buzz occurred
unsigned long buzzInterval = 0; // Calculated interval between buzzes (ms)
// -----------------------------------

double redArray[PULSE_SAMPLES]; // array to store samples from the sensor
double vReal[PULSE_SAMPLES];
double vImag[PULSE_SAMPLES];


// --- Function to create a short buzz ---
// (Using a shorter buzz than your original test to avoid disrupting sensor readings)
void doBuzz() {
  for (int i =0 ; i < 10; i++)
  {
    digitalWrite(buzzerPin, HIGH);
    delay(2);
    digitalWrite(buzzerPin, LOW);
    delay(2);
  } 
}
// ---------------------------------------

void setup()
{
    Serial.begin(115200);
    Serial.setDebugOutput(true);
    Serial.println();

    // --- Buzzer Setup ---
    pinMode(buzzerPin, OUTPUT);
    digitalWrite(buzzerPin, LOW); // Ensure buzzer is off initially
    // ------------------

    Serial.println("Running Heart Rate Monitor with Buzzer Feedback...");
    delay(3000);

    // Initialize sensor
    while (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
    {
      Serial.println("MAX30102 was not found. Please check wiring/power/solder jumper at MH-ET LIVE MAX30102 board. ");
      //while (1); // Consider removing or modifying the halt for testing
      delay(1000); // Wait before retrying
    }
    Serial.println("MAX30102 Found!");

    //Setup to sense a nice looking saw tooth on the plotter
    byte ledBrightness = 0x7F;  // Options: 0=Off to 255=50mA
    byte sampleAverage = 4;     // Options: 1, 2, 4, 8, 16, 32
    byte ledMode       = 2;     // Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
    //Options: 1 = IR only, 2 = Red + IR on MH-ET LIVE MAX30102 board
    int sampleRate     = 200;   // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
    int pulseWidth     = 411;   // Options: 69, 118, 215, 411
    int adcRange       = 16384; // Options: 2048, 4096, 8192, 16384

    // Set up the wanted parameters
    particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
    particleSensor.enableDIETEMPRDY(); // Enable die temperature reading (optional)
    oled.init();      // Initialize the OLED

    oled.clear();
    oled.setScale(2);
    oled.setCursor(0, 0);
    oled.print("Place your");
    oled.setCursor(0, 4);
    oled.print("Finger!");
    oled.update();

}

void loop()
{
    uint32_t ir, red, green;
    double fred, fir;
    //double SpO2 = 0; //raw SpO2 before low pass filtered (Not used here)
    //float red_beat = 0; (Not used here)

#ifdef USEFIFO
    particleSensor.check();            // Check the sensor, read up to 3 samples

    while (particleSensor.available())
    {  // Do we have new data
#ifdef MAX30105
      red = particleSensor.getFIFORed(); // Sparkfun's MAX30105
      ir  = particleSensor.getFIFOIR();  // Sparkfun's MAX30105
#else
      red = particleSensor.getFIFOIR();  // Potential swap for some MAX30102 boards
      ir  = particleSensor.getFIFORed(); // Potential swap for some MAX30102 boards
#endif
    // Check if finger is on sensor
    if (ir < FINGER_ON) {
       showWait = true;
       buzzInterval = 0; // Stop buzzing if finger is removed
       // Serial.println("No finger detected."); // Reduced serial noise
       particleSensor.nextSample(); // Consume the sample
       oled.clear();
       oled.setScale(2);
       oled.setCursor(0, 0);
       oled.print("Place your"); // Top-left
       oled.setCursor(0, 4);  // Line 4
       oled.print("Finger!");
       oled.update();
       continue; // Skip processing this sample
    }

      i++;
      i = i % PULSE_SAMPLES; // wrap around every PULSE_SAMPLES

      // Display "Wait" message only during the first sample collection cycle
      if (i < PULSE_SAMPLES - 1 && showWait) {
         buzzInterval = 0; // Ensure no buzzing while waiting
         oled.clear();
         oled.setScale(2);
         oled.setCursor(0, 2); // Center vertically
         oled.print("Pls Wait...");
         oled.update();
      }

      fred = (double)red;
      redArray[i] = fred; // populate the array

      particleSensor.nextSample(); // We're finished with this sample so move to next sample

      // --- Perform FFT and Update BPM calculation every PULSE_SAMPLES ---
      if (i == 0) // execute every PULSE_SAMPLES (e.g., 256 samples)
      {
         showWait = false; // Samples collected, don't show "Wait" anymore
         Serial.print("Calculating FFT... Time: ");
         Serial.println(millis());

         for (int idx=0; idx < PULSE_SAMPLES; idx++)
         {
            vReal[idx] = redArray[idx];
            vImag[idx] = 0.0;
         }

         FFT = arduinoFFT(vReal, vImag, PULSE_SAMPLES, SAMPLE_FREQ); /* Create FFT object */
         FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD); /* Weigh data */
         FFT.Compute(FFT_FORWARD); /* Compute FFT */
         FFT.ComplexToMagnitude(); /* Compute magnitudes */

         double peak = FFT.MajorPeak(); // Find the dominant frequency peak

         beatsPerMinute = peak * 60; // Convert frequency (Hz) to BPM

         // --- Update Buzzer Timing based on new BPM ---
         if (beatsPerMinute > 30 && beatsPerMinute < 200) { // Only buzz if BPM seems valid
             buzzInterval = (unsigned long)(60000.0 / beatsPerMinute); // Calculate ms between beats
             doBuzz(); // Buzz immediately for the first beat of this cycle
             lastBuzzTime = millis(); // Record the time of this buzz
             Serial.print("Valid BPM: "); Serial.print(beatsPerMinute); Serial.print(" | Buzz Interval: "); Serial.println(buzzInterval);
         } else {
             buzzInterval = 0; // Invalid BPM, stop buzzing
             Serial.print("Invalid BPM calculated: "); Serial.println(beatsPerMinute);
         }
         // -------------------------------------------

         // --- Update OLED Display ---
         oled.clear();
         oled.setScale(2);
         oled.setCursor(0, 0);  // Top-left
         oled.print("BPM:");
         oled.println(float(beatsPerMinute));  // cast to float for cleaner output
         oled.update();
         // -------------------------

      } // End of FFT calculation block (i == 0)

    } // End while (particleSensor.available())
#endif // End USEFIFO

    // --- Continuous Buzzing Check ---
    // Check if buzzing is enabled (interval > 0) and if it's time for the next buzz
    if (buzzInterval > 0 && (millis() - lastBuzzTime >= buzzInterval)) {
        doBuzz();             // Make the buzz sound
        lastBuzzTime = millis(); // Update the time of the last buzz
    }
    // ------------------------------

} // End loop()