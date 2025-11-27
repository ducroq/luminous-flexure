#ifndef MOTION_ENGINE_H
#define MOTION_ENGINE_H

#include <Arduino.h>
#include <AccelStepper.h>
#include "config.h"
#include "state_machine.h"

// ============================================================================
// MOTION PATTERNS
// ============================================================================

enum MotionPattern {
    BREATHING,          // Slow synchronized in/out
    SYNCHRONIZED_WAVE,  // Both move together
    ALTERNATING,        // One extends while other retracts
    FIGURE_EIGHT,       // Phase-shifted for 3D curves
    RANDOM_WALK,        // Independent random movements
    TREMOR,             // Small rapid vibrations
    CHASE,              // One follows the other with delay
    STILLNESS,          // Hold position
    ORGANIC_DRIFT       // Asynchronous independent random walk
};

// ============================================================================
// MOTION ENGINE CLASS
// ============================================================================

class MotionEngine {
private:
    AccelStepper& motor1;
    AccelStepper& motor2;

    void moveBothTo(int pos1, int pos2, int speed, int accel);
    void releaseMotors();

public:
    MotionEngine(AccelStepper& m1, AccelStepper& m2);
    void init();
    void perform(MotionPattern pattern);
};

// ============================================================================
// PATTERN SELECTION
// ============================================================================

MotionPattern selectPatternForState(SculptureState state);
long getPauseDurationForState(SculptureState state);

#endif // MOTION_ENGINE_H
