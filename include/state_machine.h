#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <Arduino.h>
#include "config.h"

// ============================================================================
// STATE DEFINITIONS
// ============================================================================

enum SculptureState {
    SLEEPING,       // No presence - dormant or slow breathing
    AWARE,          // Presence detected - gentle observation
    PLAYFUL,        // Moderate engagement - varied gestures
    EXCITED,        // Close/active - aggressive movements
    CONTEMPLATIVE,  // Stationary presence - slow sweeps
    RETREATING      // Person leaving - winding down
};

// ============================================================================
// STATE MACHINE CLASS
// ============================================================================

class StateMachine {
private:
    SculptureState state;
    TickType_t stateEntryTime;
    uint16_t lastValidDistance;  // Track last non-zero distance
    TickType_t lastUpdateTime;    // Debounce updates

    void transitionTo(SculptureState newState);
    const char* getStateName(SculptureState s);

public:
    StateMachine();
    void update(const SensorData& data);
    SculptureState getState() const;
};

// Global state variable (protected by mutex in main)
extern SculptureState currentState;
extern SemaphoreHandle_t stateMutex;

#endif // STATE_MACHINE_H
