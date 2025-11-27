/*
 * LINEAR WAVE - Dual Stepper Linear Actuators
 * Two CD-ROM linear actuators creating wave motion with suspended LED filament
 * Base: black acrylic of 310 x 55mm
 */

#include <Arduino.h>
#include <Stepper.h>
#include "MyLD2410.h"

#define RX_PIN 2  // Connect to LD2410 TX
#define TX_PIN 3  // Connect to LD2410 RX (through voltage divider!)
// Arduino Nano          LD2410 Sensor
// Pin D3 (RX) --------- TX
// Pin D2 (TX) --[1kΩ]-- RX
//                   └─[2kΩ]─ GND
// GND ----------------- GND
// 5V ------------------ VCC (5-12V, 200mA minimum)

// ============================================================================
// CONFIGURATION
// ============================================================================
const int maxPosition = 250; // Maximum position (adjust based on actuator travel)
const int stepsPerRevolution = 100;
const int minMotorSpeed = 200; // rpm
const int maxMotorSpeed = 600; // rpm 
const int minUpdateInterval = 100; // ms
const int maxUpdateInterval = 600; // ms
const int minSteps = 100; // steps
const int maxSteps = 400; // steps

Stepper motor1(stepsPerRevolution, 8, 9, 10, 11);
Stepper motor2(stepsPerRevolution, 4, 5, 6, 7);
MyLD2410 sensor(Serial);

bool sensorInitialized = false;

void setup() {
    Serial.begin(LD2410_BAUD_RATE);
    pinMode(13, OUTPUT);

    // Home both motors (retract to position 0)
    motor1.setSpeed(minMotorSpeed);
    motor2.setSpeed(minMotorSpeed);
    motor1.step(-maxPosition);  // Move negative to ensure we hit the limit
    motor2.step(-maxPosition);
        
    delay(1000);  // Wait for sensor to stabilize

    int blink_delay = 1000;

    if (sensor.begin()) {
        sensorInitialized = true;
        blink_delay = 100;

    } 
    // Blink fast = SUCCESS
    for(int i=0; i<10; i++) {
        digitalWrite(13, HIGH);
        delay(blink_delay);
        digitalWrite(13, LOW);
        delay(blink_delay);
    }    
}

void loop() {
    // Position tracking
    static int pos1 = 0;    // Current position of motor 1
    static int pos2 = 0;    // Current position of motor 2

    // Motion parameters
    static int updateInterval1 = 0, updateInterval2 = 0; // ms between updates
    static unsigned long last_motion1 = 0, last_motion2 = 0;
    static float speedMultiplier = 1.0;  // Default normal speed

    // Timing parameters
    static unsigned long lastFlicker = 0;
    static int flickerInterval = 100;  // ms between flicker checks
    unsigned long current_time = millis();

    if (sensorInitialized && sensor.check() == MyLD2410::DATA) {
        if (sensor.getStatus() > 0) {
            unsigned long distance = sensor.detectedDistance();

            flickerInterval = map(distance, 0, 600, 20, 500);  // 20ms to 500ms
            if (int(current_time - lastFlicker) > flickerInterval) {
                digitalWrite(13, !digitalRead(13));  // Toggle LED
                lastFlicker = current_time;
            }
            // Map distance (0-600cm) to multiplier (1-5x)
            speedMultiplier = map(distance, 0, 600, 10, 50) / 10.0;  // Gives 1.0 to 5.0
            // Or constrain it: speedMultiplier = constrain(map(...), 1, 5);
        }        
    }

    // Motor 1 - randomized timing and steps with position tracking
    if (int(current_time - last_motion1) >= updateInterval1) {
        last_motion1 = current_time;
        int steps1 = random(minSteps, maxSteps + 1);

        // Determine direction based on position bounds
        int dir1;
        if (pos1 <= 0) {
            dir1 = 1;  // Must go forward if at or below 0
        } else if (pos1 + steps1 >= maxPosition) {
            dir1 = -1;  // Must go backward if would exceed max
        } else {
            dir1 = random(0, 2) == 0 ? -1 : 1;  // Random direction otherwise
        }

        // Clamp steps to stay within bounds
        if (dir1 > 0 && pos1 + steps1 > maxPosition) {
            steps1 = maxPosition - pos1;
        } else if (dir1 < 0 && pos1 - steps1 < 0) {
            steps1 = pos1;
        }

        motor1.step(dir1 * steps1);
        pos1 += dir1 * steps1;

        // Release current to prevent overheating
        digitalWrite(8, LOW);
        digitalWrite(9, LOW);
        digitalWrite(10, LOW);
        digitalWrite(11, LOW);

        // Randomize speed and timing for next movement
        motor1.setSpeed(random(minMotorSpeed, maxMotorSpeed  + 1));
        updateInterval1 = random(minUpdateInterval, maxUpdateInterval) * speedMultiplier;
    }

    // Motor 2 - randomized timing and steps with position tracking
    if (int(current_time - last_motion2) >= updateInterval2) {
        last_motion2 = current_time;
        int steps2 = random(minSteps, maxSteps + 1);

        // Determine direction based on position bounds
        int dir2;
        if (pos2 <= 0) {
            dir2 = 1;  // Must go forward if at or below 0
        } else if (pos2 + steps2 >= maxPosition) {
            dir2 = -1;  // Must go backward if would exceed max
        } else {
            dir2 = random(0, 2) == 0 ? -1 : 1;  // Random direction otherwise
        }

        // Clamp steps to stay within bounds
        if (dir2 > 0 && pos2 + steps2 > maxPosition) {
            steps2 = maxPosition - pos2;
        } else if (dir2 < 0 && pos2 - steps2 < 0) {
            steps2 = pos2;
        }

        motor2.step(dir2 * steps2);
        pos2 += dir2 * steps2;
        // Release current to prevent overheating
        digitalWrite(2, LOW);
        digitalWrite(3, LOW);
        digitalWrite(4, LOW);
        digitalWrite(5, LOW);

        // Randomize speed and timing for next movement
        motor2.setSpeed(random(minMotorSpeed, maxMotorSpeed  + 1));
        updateInterval2 = random(minUpdateInterval, maxUpdateInterval) * speedMultiplier;
    }
}
