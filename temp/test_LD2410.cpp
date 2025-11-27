#include <Arduino.h>
#include "MyLD2410.h"

// ESP32-C3 pins for LD2410
#define LD2410_RX_PIN 20  // Connect to LD2410 TX
#define LD2410_TX_PIN 21  // Connect to LD2410 RX
#define LED_PIN 8         // ESP32-C3 Super Mini onboard LED (usually GPIO8)

// Use Serial1 for LD2410, Serial (USB) for debugging
MyLD2410 sensor(Serial1);

void setup() {
    pinMode(LED_PIN, OUTPUT);

    // USB Serial for debugging
    Serial.begin(115200);

    // Wait for USB serial to be ready (ESP32-C3 USB CDC)
    while (!Serial && millis() < 3000) {
        delay(10);
    }
    delay(500);

    Serial.println("\n\n=== LD2410 Test Starting ===");
    Serial.println("Serial output working!");

    // Serial1 for LD2410 sensor    
    Serial1.begin(LD2410_BAUD_RATE, SERIAL_8N1, LD2410_RX_PIN, LD2410_TX_PIN);
    Serial.println("Serial1 initialized (256000 baud)");
    Serial.printf("  RX Pin: GPIO%d (connect to LD2410 TX)\n", LD2410_RX_PIN);
    Serial.printf("  TX Pin: GPIO%d (connect to LD2410 RX)\n", LD2410_TX_PIN);
    delay(500);

    Serial.println("\nInitializing LD2410 sensor...");
    if (sensor.begin()) {
        Serial.println("✓ LD2410 initialized successfully!");

        // Blink fast = SUCCESS
        for(int i=0; i<10; i++) {
            digitalWrite(LED_PIN, HIGH);
            delay(100);
            digitalWrite(LED_PIN, LOW);
            delay(100);
        }
    } else {
        Serial.println("✗ LD2410 initialization FAILED!");
        Serial.println("Check wiring:");
        Serial.println("  - LD2410 VCC -> 5V");
        Serial.println("  - LD2410 GND -> GND");
        Serial.printf("  - LD2410 TX -> GPIO%d\n", LD2410_RX_PIN);
        Serial.printf("  - LD2410 RX -> GPIO%d\n", LD2410_TX_PIN);

        // Blink slow = FAILED
        while(1) {
            digitalWrite(LED_PIN, HIGH);
            delay(1000);
            digitalWrite(LED_PIN, LOW);
            delay(1000);
        }
    }
}

void loop() {
    static unsigned long lastPrint = 0;

    if (sensor.check() == MyLD2410::DATA) {
        int status = sensor.getStatus();
        digitalWrite(LED_PIN, status > 0 ? HIGH : LOW);

        // Print detection status
        static int lastStatus = -1;
        if (status != lastStatus) {
            lastStatus = status;
            unsigned long now = millis();
            if (status > 0) {
                Serial.printf("[%lu] Motion detected!\n", now);
            } else {
                Serial.printf("[%lu] No motion\n", now);
            }
        }
    }

    // Heartbeat every 5 seconds to confirm serial is working
    if (millis() - lastPrint > 5000) {
        lastPrint = millis();
        Serial.printf("[%lu] Running... (sensor active)\n", lastPrint);
    }
}