#include <WiFi.h>
#include <esp_now.h>
#include <driver/i2s.h>
#include <SD.h>
#include <FS.h>
#include <ESP32Servo.h>

// Motor Pins
#define MOTOR1_PIN1 4
#define MOTOR1_PIN2 27
#define MOTOR2_PIN1 14
#define MOTOR2_PIN2 12

// I2S Pins
#define I2S_NUM I2S_NUM_0
#define I2S_BCK_IO 22
#define I2S_LRCK_IO 25
#define I2S_DATA_IO 26
#define SD_CS 5

// Servo Pin
#define SERVO_PIN 15

// Buffer size for audio data
#define BUFFER_SIZE 1024

// Variables for motor control
String direction = "";
bool previousButton1State = false;

// Variables for servo control
bool servoState = false;  // Keeps track of the servo direction
Servo servo;

// File for audio playback
File audioFile;

// WAV header structure
struct WavHeader {
  char riff[4];
  uint32_t fileSize;
  char wave[4];
  char fmt[4];
  uint32_t fmtLength;
  uint16_t audioFormat;
  uint16_t numChannels;
  uint32_t sampleRate;
  uint32_t byteRate;
  uint16_t blockAlign;
  uint16_t bitsPerSample;
  char data[4];
  uint32_t dataSize;
};

// Data structure for receiving data via ESP-NOW
typedef struct struct_message {
  String direction;
  bool button1State;
  bool button2State;
  bool button3State;
  bool button4State;
} struct_message;

struct_message receivedData;

// Function to read WAV header
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

// I2S configuration
void setupI2S(uint32_t sampleRate) {
  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
      .sample_rate = sampleRate,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
      .communication_format = I2S_COMM_FORMAT_I2S_MSB,
      .intr_alloc_flags = 0,
      .dma_buf_count = 8,
      .dma_buf_len = 64};

  i2s_pin_config_t pin_config = {
      .bck_io_num = I2S_BCK_IO,
      .ws_io_num = I2S_LRCK_IO,
      .data_out_num = I2S_DATA_IO,
      .data_in_num = I2S_PIN_NO_CHANGE};

  i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM, &pin_config);
}

// Function to play audio
void playAudio() {
  static bool isAudioPlaying = false;

  if (!isAudioPlaying) {
    isAudioPlaying = true;

    audioFile = SD.open("/startup.wav", FILE_READ);
    if (!audioFile) {
      Serial.println("Failed to open audio file!");
      return;
    }

    WavHeader header;
    if (!readWavHeader(audioFile, header)) {
      audioFile.close();
      return;
    }

    setupI2S(header.sampleRate);

    uint8_t audioBuffer[BUFFER_SIZE];
    size_t bytesRead;

    while ((bytesRead = audioFile.read(audioBuffer, BUFFER_SIZE)) > 0) {
      size_t bytesWritten;
      i2s_write(I2S_NUM, audioBuffer, bytesRead, &bytesWritten, portMAX_DELAY);
    }

    audioFile.close();
    isAudioPlaying = false;
    Serial.println("Audio playback finished.");
  }
}

// Function to toggle servo position
void handleServoToggle() {
  static bool previousButton2State = false;

  if (receivedData.button2State && !previousButton2State) {
    servoState = !servoState;  // Toggle servo state
    int position = servoState ? 180 : 0;
    servo.write(position);     // Move servo to the new position
    Serial.printf("Servo moved to %d degrees.\n", position);
  }

  previousButton2State = receivedData.button2State;
}

// Callback function for ESP-NOW
void OnDataRecv(const esp_now_recv_info_t *recvInfo, const uint8_t *incomingData, int len) {
  memcpy(&receivedData, incomingData, sizeof(receivedData));

  // Motor control logic
  if (receivedData.direction == "forward") {
    digitalWrite(MOTOR1_PIN1, HIGH);
    digitalWrite(MOTOR1_PIN2, LOW);
    digitalWrite(MOTOR2_PIN1, HIGH);
    digitalWrite(MOTOR2_PIN2, LOW);
  } else if (receivedData.direction == "backward") {
    digitalWrite(MOTOR1_PIN1, LOW);
    digitalWrite(MOTOR1_PIN2, HIGH);
    digitalWrite(MOTOR2_PIN1, LOW);
    digitalWrite(MOTOR2_PIN2, HIGH);
  } else if (receivedData.direction == "right") {
    digitalWrite(MOTOR1_PIN1, HIGH);
    digitalWrite(MOTOR1_PIN2, LOW);
    digitalWrite(MOTOR2_PIN1, LOW);
    digitalWrite(MOTOR2_PIN2, HIGH);
  } else if (receivedData.direction == "left") {
    digitalWrite(MOTOR1_PIN1, LOW);
    digitalWrite(MOTOR1_PIN2, HIGH);
    digitalWrite(MOTOR2_PIN1, HIGH);
    digitalWrite(MOTOR2_PIN2, LOW);
  } else {
    digitalWrite(MOTOR1_PIN1, LOW);
    digitalWrite(MOTOR1_PIN2, LOW);
    digitalWrite(MOTOR2_PIN1, LOW);
    digitalWrite(MOTOR2_PIN2, LOW);
  }

  // Button 1: Play audio
  if (receivedData.button1State && !previousButton1State) {
    playAudio();
  }
  previousButton1State = receivedData.button1State;

  // Button 2: Servo control
  handleServoToggle();

  // Log other button states
  Serial.printf("Button 3: %s\n", receivedData.button3State ? "Pressed" : "Released");
  Serial.printf("Button 4: %s\n", receivedData.button4State ? "Pressed" : "Released");
}

void setup() {
  Serial.begin(115200);

  // Initialize motor pins
  pinMode(MOTOR1_PIN1, OUTPUT);
  pinMode(MOTOR1_PIN2, OUTPUT);
  pinMode(MOTOR2_PIN1, OUTPUT);
  pinMode(MOTOR2_PIN2, OUTPUT);

  // Initialize servo
  servo.attach(SERVO_PIN);

  // Initialize SD card
  if (!SD.begin(SD_CS)) {
    Serial.println("SD Card initialization failed!");
  }

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
