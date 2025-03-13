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

// Servo Pins
#define SERVO_PIN 15
#define NECK_SIDEWAYS_PIN 33 // Pin for neck sideways movement
#define NECK_UPDOWN_PIN 32  // Pin for neck up-down movement

// LED Pins
#define LED_1 35
#define LED_2 13

// Buffer size for audio data
#define BUFFER_SIZE 1024

// Variables for motor control
String direction = "";
bool previousButton1State = false;

// Variables for servo control
bool compartmentState = false;  // Keeps track of the servo direction
Servo compartmentServo;
Servo neckSidewaysServo;
Servo neckUpDownServo;

// Variables to track servo positions
int neckSidewaysPosition = 90; // Initial position for neck sideways servo (neutral)
int neckUpDownPosition = 90;   // Initial position for neck up-down servo (neutral)

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

// Function to toggle compartment (compartment servo control)
void toggleCompartment() {
  static bool previousButton2State = false;

  if (receivedData.button2State && !previousButton2State) {
    compartmentState = !compartmentState;  // Toggle servo state
    int position = compartmentState ? 180 : 0;
    compartmentServo.write(position);     // Move servo to the new position
    Serial.printf("Compartment servo moved to %d degrees.\n", position);
  }

  previousButton2State = receivedData.button2State;
}

// Function to move neck servos in a set pattern
void handleNeckMovement() {
  static bool previousButton3State = false;

  if (receivedData.button3State && !previousButton3State) {
    // Return servos to their original position first
    neckSidewaysServo.write(90);  // Move sideways servo to neutral position
    neckUpDownServo.write(90);    // Move up-down servo to neutral position
    delay(1000);  // Wait for servos to reach their neutral position

    // Move neck sideways from left to right
    for (int pos = 90; pos <= 180; pos++) {
      neckSidewaysServo.write(pos);
      // digitalWrite(LED_1, HIGH);
      delay(30);  // Delay between movements
    }
    // digitalWrite(LED_1, LOW);

    for (int pos = 180; pos >= 0; pos--) {
      neckSidewaysServo.write(pos);
      // digitalWrite(LED_2, HIGH);
      delay(30);  // Delay between movements
    }
    // digitalWrite(LED_2, LOW);

    for (int pos = 0; pos <= 90; pos++) {
      neckSidewaysServo.write(pos);
      // digitalWrite(LED_1, HIGH);
      delay(30);  // Delay between movements
    }
    // digitalWrite(LED_1, LOW);

    // Move neck up-down from down to up
    for (int pos = 90; pos <= 180; pos++) {
      neckUpDownServo.write(pos);
      delay(30);  // Delay between movements
    }
    for (int pos = 180; pos >= 90; pos--) {
      neckUpDownServo.write(pos);
      delay(30);  // Delay between movements
    }
  }

  previousButton3State = receivedData.button3State;
}

void performDance() {
  static bool previousButton4State = false;

  if (receivedData.button4State && !previousButton4State) {
    // Return servos to their original position first
    neckSidewaysServo.write(90);  // Move sideways servo to neutral position
    neckUpDownServo.write(90);    // Move up-down servo to neutral position
    delay(1000);  // Wait for servos to reach their neutral position

    // audio play start
    static bool isAudioPlaying = false;

    if (!isAudioPlaying) {
      isAudioPlaying = true;

      audioFile = SD.open("/tada.wav", FILE_READ);
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


      // // move head start
      // // Rotate neck sideways servo in a short range repeatedly
      // for (int i = 0; i < 3; i++) {  // Perform 3 cycles of neck movement
      //   // Move neck sideways from left to right
      //   for (int pos = 90; pos <= 140; pos++) {
      //     neckSidewaysServo.write(pos);
      //     delay(30);  // Delay between movements
      //   }

      //   for (int pos = 140; pos >= 55; pos--) {
      //     neckSidewaysServo.write(pos);
      //     delay(30);  // Delay between movements
      //   }

      //   for (int pos = 55; pos <= 90; pos++) {
      //     neckSidewaysServo.write(pos);
      //     delay(30);  // Delay between movements
      //   }
      // }
      // // move head end

      // LED glow start
      // for (int i = 0; i < 10; i++) {
      //   // Randomize the on/off states of the LEDs
      //   int led1State = random(2);  // Random value between 0 and 1 for LED1 (0=OFF, 1=ON)
      //   int led2State = random(2);  // Random value between 0 and 1 for LED2 (0=OFF, 1=ON)

      //   // Set LED states based on random values
      //   digitalWrite(LED_1, led1State == 1 ? HIGH : LOW);
      //   digitalWrite(LED_2, led2State == 1 ? HIGH : LOW);

      //   // Randomize the delay between state changes
      //   int randomDelay = random(200, 1000);  // Random delay between 200ms and 1000ms
      //   delay(randomDelay);  // Wait for the random delay
      // }
      //

      // wheel move start
      // Set up the duration for each motion (milliseconds)
      const int motionDuration = 1000; // 1 second for each movement
      const int spinDuration = 1500;   // 1.5 seconds for spinning

      // Movement 1: Move Forward
      digitalWrite(MOTOR1_PIN1, HIGH);
      digitalWrite(MOTOR1_PIN2, LOW);
      digitalWrite(MOTOR2_PIN1, HIGH);
      digitalWrite(MOTOR2_PIN2, LOW);
      delay(motionDuration);

      // Movement 2: Spin in place
      digitalWrite(MOTOR1_PIN1, HIGH);
      digitalWrite(MOTOR1_PIN2, LOW);
      digitalWrite(MOTOR2_PIN1, LOW);
      digitalWrite(MOTOR2_PIN2, HIGH);
      delay(spinDuration);

      // Movement 3: Move Backward
      digitalWrite(MOTOR1_PIN1, LOW);
      digitalWrite(MOTOR1_PIN2, HIGH);
      digitalWrite(MOTOR2_PIN1, LOW);
      digitalWrite(MOTOR2_PIN2, HIGH);
      delay(motionDuration);

      // Movement 4: Move Right
      digitalWrite(MOTOR1_PIN1, HIGH);
      digitalWrite(MOTOR1_PIN2, LOW);
      digitalWrite(MOTOR2_PIN1, LOW);
      digitalWrite(MOTOR2_PIN2, LOW);
      delay(motionDuration);

      // Movement 5: Move Left
      digitalWrite(MOTOR1_PIN1, LOW);
      digitalWrite(MOTOR1_PIN2, LOW);
      digitalWrite(MOTOR2_PIN1, HIGH);
      digitalWrite(MOTOR2_PIN2, LOW);
      delay(motionDuration);

      // Stop all motors after the sequence
      digitalWrite(MOTOR1_PIN1, LOW);
      digitalWrite(MOTOR1_PIN2, LOW);
      digitalWrite(MOTOR2_PIN1, LOW);
      digitalWrite(MOTOR2_PIN2, LOW);
      // wheel move end

      audioFile.close();
      isAudioPlaying = false;
      Serial.println("Audio playback finished.");
    }
    // audio play end


    // // Return servos to their original position first
    // neckSidewaysServo.write(90);  // Move sideways servo to neutral position
    // neckUpDownServo.write(90);    // Move up-down servo to neutral position
    // delay(1000);  // Wait for servos to reach their neutral position

    Serial.println("Dance routine started!");
    // // Rotate neck sideways servo in a short range repeatedly
    // for (int i = 0; i < 3; i++) {  // Perform 3 cycles of neck movement
    //   // Move neck sideways from left to right
    //   for (int pos = 90; pos <= 140; pos++) {
    //     neckSidewaysServo.write(pos);
    //     // digitalWrite(LED_1, HIGH);
    //     delay(30);  // Delay between movements
    //   }
    //   // digitalWrite(LED_1, LOW);

    //   for (int pos = 140; pos >= 55; pos--) {
    //     neckSidewaysServo.write(pos);
    //     // digitalWrite(LED_2, HIGH);
    //     delay(30);  // Delay between movements
    //   }
    //   // digitalWrite(LED_2, LOW);

    //   for (int pos = 55; pos <= 90; pos++) {
    //     neckSidewaysServo.write(pos);
    //     // digitalWrite(LED_1, HIGH);
    //     delay(30);  // Delay between movements
    //   }
    //   // digitalWrite(LED_1, LOW);
    // }

    // Return the neck to its neutral position
    neckSidewaysServo.write(90);
    Serial.println("Dance routine completed!");
  }

  previousButton4State = receivedData.button4State;
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

  // Button 2: Toggle compartment
  toggleCompartment();

  // Button 3: Neck movement routine
  handleNeckMovement();

  // Log other button states
  // Serial.printf("Button 3: %s\n", receivedData.button3State ? "Pressed" : "Released");
  if (receivedData.button4State) {
    performDance();
  }
  Serial.printf("Button 4: %s\n", receivedData.button4State ? "Pressed" : "Released");
}

void setup() {
  Serial.begin(115200);

  // Initialize motor pins
  pinMode(MOTOR1_PIN1, OUTPUT);
  pinMode(MOTOR1_PIN2, OUTPUT);
  pinMode(MOTOR2_PIN1, OUTPUT);
  pinMode(MOTOR2_PIN2, OUTPUT);

  // Initialize servo pins
  compartmentServo.attach(SERVO_PIN);
  neckSidewaysServo.attach(NECK_SIDEWAYS_PIN);
  neckUpDownServo.attach(NECK_UPDOWN_PIN);

  // Initialize LED pins
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);

  // Initialize ESP-NOW
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_recv_cb(OnDataRecv);

  // Initialize SD card
  if (!SD.begin(SD_CS)) {
    Serial.println("SD card initialization failed!");
    return;
  }
}

void loop() {
  // Main loop: ESP-NOW will call OnDataRecv whenever data is received
  // The loop function doesn't need to do anything else
}
