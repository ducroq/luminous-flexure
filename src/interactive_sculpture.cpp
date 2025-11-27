/*
 * INTERACTIVE LIGHT SCULPTURE
 * ESP32-C3 with LD2410 Sensor and Dual Linear Actuators
 *
 * Architecture:
 *   - Task 1 (Core 0): SensorTask - Reads LD2410, publishes to queue
 *   - Task 2 (Core 1): SculptureTask - Controls motors based on state
 *   - Task 3 (Core 0): ControlTask - Custom control logic
 *
 * Modules:
 *   - config.h: Hardware configuration and constants
 *   - state_machine: State definitions and transition logic
 *   - motion_engine: Motion patterns for linear actuators
 *   - sculpture_tasks: FreeRTOS task implementations
 */

#include <Arduino.h>
#include <AccelStepper.h>
#include "MyLD2410.h"
#include "config.h"
#include "state_machine.h"
#include "motion_engine.h"
#include "sculpture_tasks.h"

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// FreeRTOS resources
QueueHandle_t sensorQueue;
SemaphoreHandle_t stateMutex;

// State machine
SculptureState currentState = SLEEPING;
StateMachine stateMachine;

// Hardware instances
AccelStepper motor1(AccelStepper::FULL4WIRE, MOTOR1_PIN1, MOTOR1_PIN2, MOTOR1_PIN3, MOTOR1_PIN4);
AccelStepper motor2(AccelStepper::FULL4WIRE, MOTOR2_PIN1, MOTOR2_PIN2, MOTOR2_PIN3, MOTOR2_PIN4);
MyLD2410 sensor(Serial1);

void setup() {
    pinMode(LED_PIN, OUTPUT);
    pinMode(LED_FILAMENT_EN_PIN, OUTPUT);
    digitalWrite(LED_FILAMENT_EN_PIN, LOW);  // Start with LED off

    // USB Serial for debugging
    Serial.begin(115200);
    while (!Serial && millis() < 1000) delay(10);
    delay(500);

    Serial.println("\n\n=== Interactive Light Sculpture ===");
    Serial.println("Dual linear actuators with LD2410 sensor");
    Serial.println("Modular architecture with FreeRTOS tasks\n");

    // Initialize LD2410 sensor
    Serial1.begin(LD2410_BAUD_RATE, SERIAL_8N1, LD2410_RX_PIN, LD2410_TX_PIN);
    Serial.println("Initializing LD2410 sensor...");
    while (!Serial1 && millis() < 1000) delay(10);
    delay(500);

    if (sensor.begin()) {
        Serial.println("✓ LD2410 initialized");

        // Enable enhanced mode to see per-gate signals
        sensor.enhancedMode(true);

        // Read current parameters
        if (sensor.requestParameters()) {
            Serial.println("\nCurrent thresholds:");
            Serial.print("  Moving:     ");
            sensor.getMovingThresholds().forEach([](byte v) { Serial.printf("%3d ", v); });
            Serial.print("\n  Stationary: ");
            sensor.getStationaryThresholds().forEach([](byte v) { Serial.printf("%3d ", v); });
            Serial.printf("\n  Range: %d gates, NoOne window: %ds\n",
                         sensor.getRange(), sensor.getNoOneWindow());
        }

        // Option 1: Run auto-calibration (requires firmware >= 2.44)
        // Uncomment to calibrate - make sure area is CLEAR of people!
        /*
        Serial.println("\n⚠ AUTO-CALIBRATION - Clear the area!");
        delay(3000);
        if (sensor.autoThresholds(10)) {
            Serial.println("Auto-calibration started (10s timeout)...");
            // Wait for completion
            while (sensor.getAutoStatus() == AutoStatus::IN_PROGRESS) {
                sensor.check();
                delay(100);
            }
            if (sensor.getAutoStatus() == AutoStatus::COMPLETED) {
                Serial.println("✓ Calibration complete!");
            } else {
                Serial.println("✗ Calibration failed");
            }
        }
        */

        // Option 2: Manually raise gate 0 thresholds (the close-range gate)
        // This reduces sensitivity at 0-75cm range
        sensor.configMode(true);
        sensor.setGateParameters(0, 80, 80);  // gate 0: motion=60, stationary=60
        sensor.setGateParameters(1, 50, 50);  // gate 1: slightly lower
        sensor.configMode(false);
        Serial.println("✓ Adjusted gate 0-1 thresholds (reduced close-range sensitivity)");

        // Fast blink = success
        for(int i=0; i<5; i++) {
            digitalWrite(LED_PIN, HIGH);
            delay(100);
            digitalWrite(LED_PIN, LOW);
            delay(100);
        }
    } else {
        Serial.println("✗ LD2410 initialization FAILED");
        Serial.println("Check wiring and restart");
        // Slow blink = failure
        while(1) {
            digitalWrite(LED_PIN, HIGH);
            delay(1000);
            digitalWrite(LED_PIN, LOW);
            delay(1000);
        }
    }

    // Create FreeRTOS queue and mutex
    sensorQueue = xQueueCreate(10, sizeof(SensorData));
    stateMutex = xSemaphoreCreateMutex();

    if (sensorQueue == NULL || stateMutex == NULL) {
        Serial.println("✗ Failed to create queue/mutex");
        while(1) delay(1000);
    }

    Serial.println("✓ Queue and mutex created");

    // Create FreeRTOS tasks
    Serial.println("\nCreating tasks...");

    xTaskCreatePinnedToCore(
        sensorTask,
        "SensorTask",
        4096,       // Stack size
        NULL,
        1,          // Priority
        NULL,
        0           // Core 0
    );

    xTaskCreatePinnedToCore(
        sculptureTask,
        "SculptureTask",
        8192,       // Larger stack for motor control
        NULL,
        1,          // Priority
        NULL,
        1           // Core 1
    );

    xTaskCreatePinnedToCore(
        controlTask,
        "ControlTask",
        4096,       // Stack size
        NULL,
        1,          // Priority
        NULL,
        0           // Core 0 (with sensor task)
    );

    Serial.println("✓ Tasks created (3 tasks running)");
    Serial.println("  - SensorTask (Core 0): Reads LD2410");
    Serial.println("  - SculptureTask (Core 1): Controls motors");
    Serial.println("  - ControlTask (Core 0): Custom control logic");
    Serial.println("\n=== System running ===\n");
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
