#include <esp_now.h>
#include <WiFi.h>

#define TTP224_PIN_1  5  // Change to your GPIO pin
#define TTP224_PIN_2  18  // Change to your GPIO pin
#define TTP224_PIN_3  19  // Change to your GPIO pin
#define TTP224_PIN_4  21  // Change to your GPIO pin

typedef struct struct_message {
    int input;
} struct_message;

struct_message myData;
esp_now_peer_info_t peerInfo; // Declare peerInfo here

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    Serial.println("ESP-NOW Sender");

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Define the peer
    uint8_t peerAddress[] = {0x1C, 0x69, 0x20, 0x31, 0x39, 0x40}; // Replace with the MAC address of the receiver
    memcpy(peerInfo.peer_addr, peerAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;

    // Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }

    // Set the TTP224 pins as input
    pinMode(TTP224_PIN_1, INPUT);
    pinMode(TTP224_PIN_2, INPUT);
    pinMode(TTP224_PIN_3, INPUT);
    pinMode(TTP224_PIN_4, INPUT);
}

void loop() {
    if (digitalRead(TTP224_PIN_1) == HIGH) {
        myData.input = 1;
        esp_now_send(peerInfo.peer_addr, (uint8_t *)&myData, sizeof(myData));
        delay(500); // Debounce delay
    }
    if (digitalRead(TTP224_PIN_2) == HIGH) {
        myData.input = 2;
        esp_now_send(peerInfo.peer_addr, (uint8_t *)&myData, sizeof(myData));
        delay(500);
    }
    if (digitalRead(TTP224_PIN_3) == HIGH) {
        myData.input = 3;
        esp_now_send(peerInfo.peer_addr, (uint8_t *)&myData, sizeof(myData));
        delay(500);
    }
    if (digitalRead(TTP224_PIN_4) == HIGH) {
        myData.input = 4;
        esp_now_send(peerInfo.peer_addr, (uint8_t *)&myData, sizeof(myData));
        delay(500);
    }
}