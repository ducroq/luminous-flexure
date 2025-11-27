#include "state_machine.h"

StateMachine::StateMachine() : state(SLEEPING), stateEntryTime(0), lastValidDistance(300), lastUpdateTime(0) {}

void StateMachine::transitionTo(SculptureState newState) {
    if (newState != state) {
        Serial.printf("State transition: %s -> %s\n",
                     getStateName(state), getStateName(newState));
        state = newState;
        stateEntryTime = xTaskGetTickCount();

        // Update global state (thread-safe)
        xSemaphoreTake(stateMutex, portMAX_DELAY);
        currentState = newState;
        xSemaphoreGive(stateMutex);
    }
}

const char* StateMachine::getStateName(SculptureState s) {
    switch(s) {
        case SLEEPING: return "SLEEPING";
        case AWARE: return "AWARE";
        case PLAYFUL: return "PLAYFUL";
        case EXCITED: return "EXCITED";
        case CONTEMPLATIVE: return "CONTEMPLATIVE";
        case RETREATING: return "RETREATING";
        default: return "UNKNOWN";
    }
}

void StateMachine::update(const SensorData& data) {
    TickType_t now = xTaskGetTickCount();

    // Debounce: Only update state machine every 500ms
    if (now - lastUpdateTime < pdMS_TO_TICKS(500)) {
        return;  // Skip this update
    }
    lastUpdateTime = now;

    // Filter distance: Ignore 0cm readings (sensor uncertainty)
    uint16_t distance = data.distance;
    if (distance == 0) {
        distance = lastValidDistance;  // Use last known good distance
    } else {
        lastValidDistance = distance;  // Update last valid distance
    }

    // No presence detected
    if (!data.presence) {
        // Turn LED OFF when no one is around
        digitalWrite(LED_FILAMENT_EN_PIN, LOW);

        if (state != SLEEPING && state != RETREATING) {
            transitionTo(RETREATING);
        } else if (state == RETREATING &&
                  (now - stateEntryTime > pdMS_TO_TICKS(5000))) {
            transitionTo(SLEEPING);
        }
        return;
    }

    // Presence detected - turn LED ON to lure them in!
    digitalWrite(LED_FILAMENT_EN_PIN, HIGH);

    // Minimum time in state before allowing transitions (stability)
    const TickType_t MIN_STATE_TIME = pdMS_TO_TICKS(3000);  // 3 seconds
    bool canTransition = (now - stateEntryTime > MIN_STATE_TIME);

    // Determine state based on distance and motion with hysteresis
    if (distance < 50 && canTransition) {
        // Very close - excited/aggressive response
        if (state != EXCITED) {
            transitionTo(EXCITED);
        }
    } else if (distance < 150 && data.motionEnergy > 50 && canTransition) {
        // Medium distance with significant movement - playful
        if (state != PLAYFUL) {
            transitionTo(PLAYFUL);
        }
    } else if (distance < 150 && data.stationaryEnergy > 40 && data.motionEnergy < 30 && canTransition) {
        // Medium distance, mostly stationary - contemplative watching
        if (state != CONTEMPLATIVE) {
            transitionTo(CONTEMPLATIVE);
        }
    } else if (distance >= 150 && canTransition) {
        // Far away - gentle AWARE state to lure them closer
        if (state != AWARE) {
            transitionTo(AWARE);
        }
    } else if ((state == SLEEPING || state == RETREATING) && canTransition) {
        // Just detected - start with aware
        transitionTo(AWARE);
    }
}

SculptureState StateMachine::getState() const {
    return state;
}
