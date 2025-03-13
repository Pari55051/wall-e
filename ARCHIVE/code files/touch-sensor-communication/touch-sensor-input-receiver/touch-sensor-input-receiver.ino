#include <esp_now.h>
#include <WiFi.h>

#define LED_PIN_1  14  // Change to your GPIO pin
#define LED_PIN_2  27  // Change to your GPIO pin
#define LED_PIN_3  26  // Change to your GPIO pin
#define LED_PIN_4  25  // Change to your GPIO pin

typedef struct struct_message {
    int input;
} struct_message;

struct_message myData;
bool isExecuting = false; // Flag to indicate if an LED is currently being activated

void activateLED(int ledPin, int delayTime) {
    digitalWrite(ledPin, HIGH);
    delay(delayTime); // Keep LED on for the specified delay time
    digitalWrite(ledPin, LOW);
    isExecuting = false; // Reset the flag after execution
}

void onDataRecv(const esp_now_recv_info* info, const uint8_t* incomingData, int len) {
    if (isExecuting) {
        Serial.println("Ignoring input, already executing.");
        return; // Ignore new input if already executing
    }

    memcpy(&myData, incomingData, sizeof(myData));
    Serial.printf("Received input: %d\n", myData.input);
    
    isExecuting = true; // Set the flag to indicate execution is in progress

    // Control LEDs based on input with different delay times
    switch (myData.input) {
        case 1:
            activateLED(LED_PIN_1, 1000); // 1 second for LED 1
            break;
        case 2:
            activateLED(LED_PIN_2, 2000); // 2 seconds for LED 2
            break;
        case 3:
            activateLED(LED_PIN_3, 3000); // 3 seconds for LED 3
            break;
        case 4:
            activateLED(LED_PIN_4, 4000); // 4 seconds for LED 4
            break;
        default:
            isExecuting = false; // Reset the flag if input is invalid
            break;
    }
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    Serial.println("ESP-NOW Receiver");

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Register callback for receiving data
    esp_now_register_recv_cb(onDataRecv);

    // Initialize LED pins
    pinMode(LED_PIN_1, OUTPUT);
    pinMode(LED_PIN_2, OUTPUT);
    pinMode(LED_PIN_3, OUTPUT);
    pinMode(LED_PIN_4, OUTPUT);
}

void loop() {
    // Nothing to do here, everything is handled in the callback
}