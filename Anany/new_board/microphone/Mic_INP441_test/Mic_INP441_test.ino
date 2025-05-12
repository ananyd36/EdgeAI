#include <driver/i2s.h>

// Define I2S pins for INMP441 (adjust these if needed for your setup)
const int i2s_bck_pin = 15;    // Serial Clock (BCLK)  - connect to SCK on INMP441
const int i2s_ws_pin = 13;     // Word Select (LRCLK) - connect to WS on INMP441
const int i2s_data_pin = 34;   // Serial Data (DOUT)   - connect to SD on INMP441

void setup() {
  Serial.begin(115200);
  Serial.println("INMP441 Raw Value Test (ESP32)");

  // Configure I2S
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX), // Master receiver
    .sample_rate = 16000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, // Or I2S_CHANNEL_FMT_ONLY_RIGHT
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,     // Interrupt priority
    .dma_buf_count = 4,                          // Number of DMA buffers
    .dma_buf_len = 1024,
    .use_apll = false                            // Samples per DMA buffer
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = i2s_bck_pin,
    .ws_io_num = i2s_ws_pin,
    .data_out_num = -1, // INMP441 is input only
    .data_in_num = i2s_data_pin
  };

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);

  i2s_set_pin(I2S_NUM_0, &pin_config);

  Serial.println("I2S driver installed and configured.");
}

void loop() {
  size_t bytes_read;
  int32_t audio_samples[1024]; // Buffer to read samples

  // Read data from I2S
  esp_err_t err = i2s_read(I2S_NUM_0, (void*)audio_samples, sizeof(audio_samples), &bytes_read, portMAX_DELAY);

  if (err == ESP_OK) {
    for (int i = 0; i < bytes_read / sizeof(int32_t); i++) {
      Serial.println(audio_samples[i]);
      delay(100); // Small delay for serial output
    }
    // You can add further processing of the audio data here
  } else {
    Serial.printf("Error reading from I2S: %d\n", err);
    delay(10);
  }
}