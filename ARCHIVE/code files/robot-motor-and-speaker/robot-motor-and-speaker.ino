#include <driver/i2s.h>
#include <SD.h>
#include <FS.h>
#include <esp_now.h>
#include <WiFi.h>

// Motor pins
#define MOTOR1_PIN1 4    // Motor 1 IN1 (D4)
#define MOTOR1_PIN2 27   // Motor 1 IN2 (D27)
#define MOTOR2_PIN1 14   // Motor 2 IN1 (D14)
#define MOTOR2_PIN2 12   // Motor 2 IN2 (D12)

// I2S configuration pins
#define I2S_NUM         I2S_NUM_0 // I2S port number
#define I2S_BCK_IO      22        // I2S Bit Clock
#define I2S_LRCK_IO     25        // I2S Left-Right Clock
#define I2S_DATA_IO     26        // I2S Serial Data
#define SD_CS           5         // SD Card Chip Select Pin

#define BUFFER_SIZE     1024      // Buffer size for audio data

File audioFile;

// WAV file header structure
struct WavHeader {
  char riff[4];          // "RIFF"
  uint32_t fileSize;     // File size - 8 bytes
  char wave[4];          // "WAVE"
  char fmt[4];           // "fmt "
  uint32_t fmtLength;    // Length of format data
  uint16_t audioFormat;  // Format type (1 = PCM)
  uint16_t numChannels;  // Number of channels
  uint32_t sampleRate;   // Sample rate
  uint32_t byteRate;     // Byte rate
  uint16_t blockAlign;   // Block align
  uint16_t bitsPerSample;// Bits per sample
  char data[4];          // "data"
  uint32_t dataSize;     // Size of audio data
};

// Data structure for receiving joystick direction and button states
typedef struct struct_message {
  String direction;    // Direction value ("forward", "backward", "left", "right", "stop")
  bool button1State;   // Button 1 state (pressed/released)
  bool button2State;   // Button 2 state (pressed/released)
  bool button3State;   // Button 3 state (pressed/released)
  bool button4State;   // Button 4 state (pressed/released)
} struct_message;

struct_message incomingData; // Data received
bool previousButton1State = false; // Stores the previous state of Button 1

// Callback function when data is received
void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *incomingDataRaw, int len) {
  memcpy(&incomingData, incomingDataRaw, sizeof(incomingData));

  // Display received data
  Serial.print("Received Data:\n");
  Serial.print("Direction: ");
  Serial.println(incomingData.direction);
  Serial.print("Button 1: ");
  Serial.println(incomingData.button1State ? "Pressed" : "Released");
  Serial.print("Button 2: ");
  Serial.println(incomingData.button2State ? "Pressed" : "Released");
  Serial.print("Button 3: ");
  Serial.println(incomingData.button3State ? "Pressed" : "Released");
  Serial.print("Button 4: ");
  Serial.println(incomingData.button4State ? "Pressed" : "Released");

  // If Button 1 is pressed and was not pressed before, play the audio
  if (incomingData.button1State && !previousButton1State) {
    playAudio(); // Play the audio file
  }

  // Update the previous Button 1 state
  previousButton1State = incomingData.button1State;

  // Control motors based on direction received
  controlMotors(incomingData.direction);
}

void controlMotors(String direction) {
  // Motor 1 (D4, D27), Motor 2 (D14, D12) control
  if (direction == "forward") {
    // Move forward
    digitalWrite(MOTOR1_PIN1, HIGH);
    digitalWrite(MOTOR1_PIN2, LOW);
    digitalWrite(MOTOR2_PIN1, HIGH);
    digitalWrite(MOTOR2_PIN2, LOW);
  }
  else if (direction == "backward") {
    // Move backward
    digitalWrite(MOTOR1_PIN1, LOW);
    digitalWrite(MOTOR1_PIN2, HIGH);
    digitalWrite(MOTOR2_PIN1, LOW);
    digitalWrite(MOTOR2_PIN2, HIGH);
  }
  else if (direction == "left") {
    // Turn left (Motor 1 backward, Motor 2 forward)
    digitalWrite(MOTOR1_PIN1, LOW);
    digitalWrite(MOTOR1_PIN2, HIGH);
    digitalWrite(MOTOR2_PIN1, HIGH);
    digitalWrite(MOTOR2_PIN2, LOW);
  }
  else if (direction == "right") {
    // Turn right (Motor 1 forward, Motor 2 backward)
    digitalWrite(MOTOR1_PIN1, HIGH);
    digitalWrite(MOTOR1_PIN2, LOW);
    digitalWrite(MOTOR2_PIN1, LOW);
    digitalWrite(MOTOR2_PIN2, HIGH);
  }
  else if (direction == "stop") {
    // Stop both motors
    digitalWrite(MOTOR1_PIN1, LOW);
    digitalWrite(MOTOR1_PIN2, LOW);
    digitalWrite(MOTOR2_PIN1, LOW);
    digitalWrite(MOTOR2_PIN2, LOW);
  }
}

void setupI2S(uint32_t sampleRate) {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX), // Master, TX only
    .sample_rate = sampleRate,                          // WAV sample rate
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,       // 16-bit audio
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,       // Stereo
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,    // I2S MSB format
    .intr_alloc_flags = 0,                              // Interrupt level 1
    .dma_buf_count = 8,                                 // Number of DMA buffers
    .dma_buf_len = 64                                   // Size of each buffer
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCK_IO,
    .ws_io_num = I2S_LRCK_IO,
    .data_out_num = I2S_DATA_IO,
    .data_in_num = I2S_PIN_NO_CHANGE // Not used
  };

  // Install and configure I2S driver
  i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM, &pin_config);
}

void playAudio() {
  // Open the WAV file
  audioFile = SD.open("/startup.wav", FILE_READ);
  if (!audioFile) {
    Serial.println("Failed to open audio file!");
    return;
  }
  Serial.println("Audio file opened successfully.");

  // Read and validate the WAV header
  WavHeader header;
  if (!readWavHeader(audioFile, header)) {
    audioFile.close();
    return;
  }

  // Configure I2S based on the WAV header
  setupI2S(header.sampleRate);

  // Buffer to hold audio data
  uint8_t audioBuffer[BUFFER_SIZE];

  // Play the audio file
  while (audioFile.available()) {
    size_t bytesRead = audioFile.read(audioBuffer, BUFFER_SIZE);

    // If there's data to send, write it to I2S
    if (bytesRead > 0) {
      size_t bytesWritten = 0;
      i2s_write(I2S_NUM, audioBuffer, bytesRead, &bytesWritten, portMAX_DELAY);
      Serial.printf("Bytes written to I2S: %d\n", bytesWritten);
    }
  }

  // Close the file after playback
  audioFile.close();
  Serial.println("Audio playback finished.");
}

bool readWavHeader(File &file, WavHeader &header) {
  if (file.read((uint8_t *)&header, sizeof(WavHeader)) != sizeof(WavHeader)) {
    Serial.println("Failed to read WAV header.");
    return false;
  }

  if (strncmp(header.riff, "RIFF", 4) != 0 || strncmp(header.wave, "WAVE", 4) != 0) {
    Serial.println("Invalid WAV file format.");
    return false;
  }

  Serial.println("WAV Header Info:");
  Serial.printf("Sample Rate: %d Hz\n", header.sampleRate);
  Serial.printf("Channels: %d\n", header.numChannels);
  Serial.printf("Bits Per Sample: %d\n", header.bitsPerSample);
  Serial.printf("Data Size: %d bytes\n", header.dataSize);

  return true;
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Initialize SD card
  if (!SD.begin(SD_CS)) {
    Serial.println("SD Card initialization failed!");
    return;
  }
  Serial.println("SD Card initialized.");

  // Initialize motor pins
  pinMode(MOTOR1_PIN1, OUTPUT);
  pinMode(MOTOR1_PIN2, OUTPUT);
  pinMode(MOTOR2_PIN1, OUTPUT);
  pinMode(MOTOR2_PIN2, OUTPUT);

  // Initialize WiFi and ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW!");
    return;
  }

  // Register callback to receive data
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  // Nothing needed here; everything is handled in the callback
  delay(100);
}
