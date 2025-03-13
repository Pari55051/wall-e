#include <esp_now.h>
#include <WiFi.h>

// Define GPIO pins for joystick and buttons
#define JOYSTICK_X_PIN 34   // Analog pin for joystick X-axis
#define JOYSTICK_Y_PIN 35   // Analog pin for joystick Y-axis
#define BUTTON1_PIN 13
#define BUTTON2_PIN 14
#define BUTTON3_PIN 27
#define BUTTON4_PIN 26

// Data structure for the message
typedef struct struct_message {
  String direction;   // Direction value ("forward", "backward", "left", "right", "stop")
  bool button1State;  // State of button 1 (pressed/released)
  bool button2State;  // State of button 2 (pressed/released)
  bool button3State;  // State of button 3 (pressed/released)
  bool button4State;  // State of button 4 (pressed/released)
} struct_message;

struct_message outgoingData; // Data to send

// MAC address of the receiver ESP32 (adjust accordingly)
uint8_t deviceBAddress[] = {0x1C, 0x69, 0x20, 0x31, 0xB3, 0x88};

// Callback function when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void setup() {
  Serial.begin(115200);

  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Configure joystick and button pins
  pinMode(JOYSTICK_X_PIN, INPUT);
  pinMode(JOYSTICK_Y_PIN, INPUT);
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  pinMode(BUTTON3_PIN, INPUT_PULLUP);
  pinMode(BUTTON4_PIN, INPUT_PULLUP);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register send callback
  esp_now_register_send_cb(OnDataSent);

  // Register peer (Receiver ESP32)
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, deviceBAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  // Read joystick values
  int joystickX = analogRead(JOYSTICK_X_PIN);  // X-axis value (0-4095)
  int joystickY = analogRead(JOYSTICK_Y_PIN);  // Y-axis value (0-4095)

  // Map joystick readings from 0-4095 to -2048 to 2047
  joystickX = map(joystickX, 0, 4095, -2048, 2047);
  joystickY = map(joystickY, 0, 4095, -2048, 2047);

  // Threshold value for movement detection (increased threshold to 500)
  int threshold = 500;

  // Process joystick values and determine direction
  if (joystickY < -threshold) {
    outgoingData.direction = "left";
  }
  else if (joystickY > threshold) {
    outgoingData.direction = "right";
  }
  else if (joystickX < -threshold) {
    outgoingData.direction = "backward";
  }
  else if (joystickX > threshold) {
    outgoingData.direction = "forward";
  }
  else {
    outgoingData.direction = "stop"; // If joystick is near center, stop movement
  }

  // Read button states
  outgoingData.button1State = digitalRead(BUTTON1_PIN) == LOW; // true if pressed
  outgoingData.button2State = digitalRead(BUTTON2_PIN) == LOW; // true if pressed
  outgoingData.button3State = digitalRead(BUTTON3_PIN) == LOW; // true if pressed
  outgoingData.button4State = digitalRead(BUTTON4_PIN) == LOW; // true if pressed

  // Send direction and button states to the receiver
  esp_now_send(deviceBAddress, (uint8_t*)&outgoingData, sizeof(outgoingData));

  // Debugging: print direction and button states
  Serial.print("Sending Direction: ");
  Serial.print(outgoingData.direction);
  Serial.print(" | Button 1: ");
  Serial.print(outgoingData.button1State ? "Pressed" : "Released");
  Serial.print(" | Button 2: ");
  Serial.print(outgoingData.button2State ? "Pressed" : "Released");
  Serial.print(" | Button 3: ");
  Serial.print(outgoingData.button3State ? "Pressed" : "Released");
  Serial.print(" | Button 4: ");
  Serial.println(outgoingData.button4State ? "Pressed" : "Released");

  delay(100); // Small delay to make the system responsive
}
