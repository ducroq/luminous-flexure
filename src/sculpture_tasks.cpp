#include "sculpture_tasks.h"

void sensorTask(void* parameter) {
    Serial.println("Sensor task started");
    TickType_t lastDebugTime = 0;

    while(true) {
        if (sensor.check() == MyLD2410::DATA) {
            SensorData data;
            data.presence = sensor.presenceDetected();
            data.distance = sensor.detectedDistance();
            data.motionEnergy = sensor.movingTargetSignal();
            data.stationaryEnergy = sensor.stationaryTargetSignal();
            data.timestamp = millis();  // OK to use millis() for inter-task data

            // Send to queue (non-blocking)
            xQueueSend(sensorQueue, &data, 0);

            // Debug output (every 2 seconds) - use FreeRTOS ticks
            TickType_t now = xTaskGetTickCount();
            if (now - lastDebugTime > pdMS_TO_TICKS(2000)) {
                lastDebugTime = now;
                Serial.printf("Sensor: presence=%d dist=%dcm motion=%d stationary=%d\n",
                             data.presence, data.distance, data.motionEnergy, data.stationaryEnergy);

                // Show per-gate signals in enhanced mode (helps diagnose false triggers)
                if (sensor.inEnhancedMode()) {
                    Serial.print("  Gates [0-8]: ");
                    sensor.getMovingSignals().forEach([](byte v) { Serial.printf("%3d ", v); });
                    Serial.println(" (motion)");
                    Serial.print("             : ");
                    sensor.getStationarySignals().forEach([](byte v) { Serial.printf("%3d ", v); });
                    Serial.println(" (stationary)");
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));  // Check sensor every 50ms
    }
}

void sculptureTask(void* parameter) {
    Serial.println("Sculpture task started");

    MotionEngine* motionEngine = new MotionEngine(motor1, motor2);

    Serial.println("Running homing sequence...");
    motionEngine->init();

    Serial.println("\n=== SENSOR-DRIVEN INTERACTIVE MODE ===");
    Serial.println("Sculpture responding to presence and motion\n");

    // State names for debugging
    const char* stateNames[] = {
        "SLEEPING", "AWARE", "PLAYFUL", "EXCITED", "CONTEMPLATIVE", "RETREATING"
    };

    TickType_t lastSensorCheck = 0;

    while(true) {
        // Check for sensor data (non-blocking)
        SensorData sensorData;
        if (xQueueReceive(sensorQueue, &sensorData, 0) == pdTRUE) {
            lastSensorCheck = xTaskGetTickCount();

            // Update state machine with sensor data
            stateMachine.update(sensorData);
        }

        // Get current state from global (thread-safe)
        SculptureState state;
        xSemaphoreTake(stateMutex, portMAX_DELAY);
        state = currentState;
        xSemaphoreGive(stateMutex);

        Serial.printf("\n━━━ State: %s ━━━\n", stateNames[state]);

        // Select pattern based on state
        MotionPattern pattern = selectPatternForState(state);

        // Get pause duration for this state
        long pauseDuration = getPauseDurationForState(state);

        // Perform the pattern
        motionEngine->perform(pattern);

        // Pause between movements (short when active, longer when sleeping)
        if (pauseDuration > 0) {
            Serial.printf("Pausing for %ld ms...\n", pauseDuration);
            vTaskDelay(pdMS_TO_TICKS(pauseDuration));
        }
    }
}

void controlTask(void* parameter) {
    Serial.println("Control task started");
    TickType_t lastCheck = 0;

    while(true) {
        TickType_t now = xTaskGetTickCount();

        // Run control logic every 5 seconds
        if (now - lastCheck > pdMS_TO_TICKS(5000)) {
            lastCheck = now;

            // Read current state (thread-safe)
            SculptureState state;
            xSemaphoreTake(stateMutex, portMAX_DELAY);
            state = currentState;
            xSemaphoreGive(stateMutex);

            // ========================================================
            // PLACEHOLDER: Add your custom control logic here
            // ========================================================
            // Examples:
            // - Read buttons/switches to force specific states
            // - Check web API/MQTT for remote commands
            // - Implement time-based behaviors (e.g., sleep at night)
            // - Override state based on custom sensors
            // - Implement sequences or choreography

            // Example: Force state change (currently disabled)
            /*
            if (someCondition) {
                xSemaphoreTake(stateMutex, portMAX_DELAY);
                currentState = EXCITED;  // Force to EXCITED state
                xSemaphoreGive(stateMutex);
                Serial.println("Control: Forced state to EXCITED");
            }
            */

            // Debug: Print current state periodically
            Serial.printf("Control: Current state = %d\n", state);
        }

        vTaskDelay(pdMS_TO_TICKS(100));  // Check every 100ms
    }
}
