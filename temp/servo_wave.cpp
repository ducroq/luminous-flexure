/*
 * LUMINOUS FLEXURE - Dual servos articulated arms with suspended LED filament
 *
 * Clean separation of concerns:
 * - Motion patterns as data
 * - Phase accumulation engine
 * - Easing system
 * - Geometric constraints
 * - Pattern sequencer
 */

#include <Arduino.h>
#include <Servo.h>

// ============================================================================
// CONFIGURATION
// ============================================================================

namespace Config {
    // Hardware
    const int SERVO_PIN_1 = 9;
    const int SERVO_PIN_2 = 10;

    // Physical geometry
    const float SERVO_SEPARATION = 25.0;  // cm
    const float ARM_LENGTH = 16.0;        // cm (increased for more spatial drama)
    const float LED_MAX_LENGTH = 25.0;    // cm (reduced to prevent breaking!)
    const float LED_MIN_LENGTH = 5.0;     // cm

    // Motion
    const int CENTER_ANGLE = 90;
    const int MIN_ANGLE = 0;
    const int MAX_ANGLE = 180;
    const long TRANSITION_DURATION_MS = 5000;

    // Randomness
    const float PERIOD_VARIATION = 0.2;      // ±20%
    const float AMPLITUDE_VARIATION = 0.15;  // ±15%
    const int PHASE_VARIATION = 15;          // ±15°
    const float DURATION_VARIATION = 0.25;   // ±25%

    // Noise/drift
    const float DRIFT_SLOW_FREQ = 0.05;
    const float DRIFT_FAST_FREQ = 0.2;
    const float DRIFT_SLOW_AMOUNT = 0.1;
    const float DRIFT_FAST_AMOUNT = 0.05;
}

// ============================================================================
// MOTION PATTERN
// ============================================================================

struct MotionPattern {
    const char* name;
    long period_ms;
    int phase_difference;
    float amplitude;
    long duration_seconds;
};

namespace Patterns {
    const MotionPattern LIBRARY[] = {
        {"Gentle Breathing",   8000,   0, 0.30,  30},
        {"Slow Wave",          8000,  60, 0.40,  40},
        {"Quick Ripple",       3000,  45, 0.25,  25},
        {"Figure-8",          10000, 180, 0.45,  45},
        {"Deep Meditation",   15000,  90, 0.35,  50},
        {"Synchronized Pump",  5000,   0, 0.50,  30},
        {"Asymmetric Wave",    7000, 120, 0.38,  35},
    };
    const int COUNT = sizeof(LIBRARY) / sizeof(LIBRARY[0]);
}

// ============================================================================
// EASING SYSTEM
// ============================================================================

class EasingEngine {
public:
    static float smoothstep(float x) {
        x = constrain(x, 0.0, 1.0);
        return x * x * (3.0 - 2.0 * x);
    }

    static float easedSine(float sin_value) {
        float normalized = (sin_value + 1.0) / 2.0;
        float eased = smoothstep(normalized);
        return (eased * 2.0) - 1.0;
    }

    static float lerp(float a, float b, float t) {
        return a + (b - a) * t;
    }
};

// ============================================================================
// NOISE GENERATOR (for organic drift)
// ============================================================================

class NoiseGenerator {
private:
    float offset;

public:
    NoiseGenerator(float seed_offset = 0.0) : offset(seed_offset) {}

    float smoothNoise(float t, float frequency) {
        return sin(t * frequency) * 0.5 + 0.5;
    }

    float getDrift(float time_sec) {
        float t = time_sec + offset;
        float drift1 = smoothNoise(t, Config::DRIFT_SLOW_FREQ) - 0.5;
        float drift2 = smoothNoise(t + 100, Config::DRIFT_FAST_FREQ) - 0.5;
        return (drift1 * Config::DRIFT_SLOW_AMOUNT + drift2 * Config::DRIFT_FAST_AMOUNT);
    }
};

// ============================================================================
// PHASE ACCUMULATOR (solves morphing discontinuity)
// ============================================================================

class PhaseAccumulator {
private:
    float phase;              // Radians
    unsigned long last_time;  // Milliseconds

public:
    PhaseAccumulator() : phase(0.0), last_time(0) {}

    void reset() {
        phase = 0.0;
        last_time = 0;
    }

    float accumulate(unsigned long current_time_ms, float period_ms) {
        // Calculate time delta
        unsigned long dt_ms = (last_time > 0) ? (current_time_ms - last_time) : 0;
        last_time = current_time_ms;

        // Increment phase based on current period
        float phase_increment = (dt_ms / period_ms) * (2.0 * PI);
        phase += phase_increment;

        // Keep phase in reasonable range
        while (phase > 2.0 * PI) {
            phase -= 2.0 * PI;
        }

        return phase;
    }

    float getPhase() const { return phase; }
};

// ============================================================================
// GEOMETRY CONSTRAINTS
// ============================================================================

class GeometryConstraints {
private:
    static float calculateLEDLength(int angle1, int angle2) {
        float theta1 = radians(angle1 - 90);
        float theta2 = radians(angle2 - 90);

        float x1 = Config::ARM_LENGTH * cos(theta1);
        float y1 = Config::ARM_LENGTH * sin(theta1);

        float x2 = Config::SERVO_SEPARATION + Config::ARM_LENGTH * cos(theta2);
        float y2 = Config::ARM_LENGTH * sin(theta2);

        float dx = x2 - x1;
        float dy = y2 - y1;

        return sqrt(dx * dx + dy * dy);
    }

    static bool isValid(int angle1, int angle2) {
        float length = calculateLEDLength(angle1, angle2);
        return (length >= Config::LED_MIN_LENGTH && length <= Config::LED_MAX_LENGTH);
    }

public:
    static int constrainAngle(int angle1, int desired_angle2) {
        if (isValid(angle1, desired_angle2)) {
            return desired_angle2;
        }

        // Search for nearest valid angle
        int center = Config::CENTER_ANGLE;
        int offset = abs(desired_angle2 - center);

        for (int test_offset = offset; test_offset >= 0; test_offset -= 2) {
            int test_angle = (desired_angle2 >= center) ?
                            center + test_offset : center - test_offset;

            // Constrain manually (avoid macro)
            if (test_angle < Config::MIN_ANGLE) test_angle = Config::MIN_ANGLE;
            if (test_angle > Config::MAX_ANGLE) test_angle = Config::MAX_ANGLE;

            if (isValid(angle1, test_angle)) {
                return test_angle;
            }
        }

        return center;  // Fallback
    }

    static float getLength(int angle1, int angle2) {
        return calculateLEDLength(angle1, angle2);
    }
};

// ============================================================================
// PATTERN VARIATION GENERATOR
// ============================================================================

class VariationGenerator {
private:
    float period_factor;
    float amplitude_factor;
    int phase_offset;
    float duration_factor;

    static float randomFloat(float min_val, float max_val) {
        return min_val + (random(10000) / 10000.0) * (max_val - min_val);
    }

public:
    void generate() {
        period_factor = randomFloat(1.0 - Config::PERIOD_VARIATION,
                                   1.0 + Config::PERIOD_VARIATION);
        amplitude_factor = randomFloat(1.0 - Config::AMPLITUDE_VARIATION,
                                      1.0 + Config::AMPLITUDE_VARIATION);
        phase_offset = random(-Config::PHASE_VARIATION, Config::PHASE_VARIATION + 1);
        duration_factor = randomFloat(1.0 - Config::DURATION_VARIATION,
                                     1.0 + Config::DURATION_VARIATION);
    }

    long applyToPeriod(long base_period) const {
        return base_period * period_factor;
    }

    float applyToAmplitude(float base_amplitude) const {
        return base_amplitude * amplitude_factor;
    }

    int applyToPhase(int base_phase) const {
        return base_phase + phase_offset;
    }

    unsigned long applyToDuration(long base_duration) const {
        return (unsigned long)(base_duration * duration_factor);
    }
};

// ============================================================================
// MOTION GENERATOR
// ============================================================================

class MotionGenerator {
private:
    NoiseGenerator noise;

    int calculateAngleFromPhase(float phase, int phase_offset, float amplitude,
                                float time_sec, bool apply_drift, bool apply_easing) {
        // Add phase offset
        float total_phase = phase + radians(phase_offset);

        // Calculate sine
        float sin_value = sin(total_phase);

        // Apply easing
        if (apply_easing) {
            sin_value = EasingEngine::easedSine(sin_value);
        }

        // Add drift
        if (apply_drift) {
            float drift = noise.getDrift(time_sec);
            sin_value += drift;
            sin_value = constrain(sin_value, -1.0, 1.0);
        }

        // Scale to servo range
        float full_range = Config::MAX_ANGLE - Config::MIN_ANGLE;
        float swing = (full_range / 2.0) * amplitude;
        int angle = Config::CENTER_ANGLE + (sin_value * swing);

        // Constrain manually (avoid macro)
        if (angle < Config::MIN_ANGLE) angle = Config::MIN_ANGLE;
        if (angle > Config::MAX_ANGLE) angle = Config::MAX_ANGLE;
        return angle;
    }

public:
    MotionGenerator(float noise_seed) : noise(noise_seed) {}

    void generateAngles(float phase, float amplitude, int phase_difference,
                       float time_sec, bool apply_drift, int& angle1, int& angle2) {
        angle1 = calculateAngleFromPhase(phase, 0, amplitude, time_sec,
                                        apply_drift, true);
        angle2 = calculateAngleFromPhase(phase, phase_difference, amplitude,
                                        time_sec, apply_drift, true);
    }
};

// ============================================================================
// PATTERN SEQUENCER
// ============================================================================

class PatternSequencer {
private:
    int current_index;
    int previous_index;
    unsigned long pattern_start_time;
    bool in_transition;
    unsigned long transition_start_time;

    VariationGenerator variation;
    PhaseAccumulator phase_accumulator;
    MotionGenerator motion_generator;

    int selectRandomPattern() {
        if (Patterns::COUNT <= 1) return 0;

        int next;
        do {
            next = random(Patterns::COUNT);
        } while (next == current_index);

        return next;
    }

public:
    PatternSequencer(float noise_seed)
        : current_index(0), previous_index(0), pattern_start_time(0),
          in_transition(false), transition_start_time(0),
          motion_generator(noise_seed) {}

    void begin(unsigned long current_time) {
        current_index = random(Patterns::COUNT);
        variation.generate();
        pattern_start_time = current_time;
        phase_accumulator.reset();
    }

    void update(unsigned long current_time, int& angle1, int& angle2) {
        const MotionPattern& current = Patterns::LIBRARY[current_index];

        // Check for pattern change
        unsigned long time_in_pattern_sec = (current_time - pattern_start_time) / 1000;
        unsigned long varied_duration = variation.applyToDuration(current.duration_seconds);

        if (!in_transition && time_in_pattern_sec >= varied_duration) {
            // Start transition
            previous_index = current_index;
            current_index = selectRandomPattern();
            variation.generate();

            in_transition = true;
            transition_start_time = current_time;
            pattern_start_time = current_time;
        }

        // Calculate active parameters (with morphing during transition)
        long active_period;
        float active_amplitude;
        int active_phase;

        if (in_transition) {
            const MotionPattern& prev = Patterns::LIBRARY[previous_index];

            float progress = (float)(current_time - transition_start_time) /
                           Config::TRANSITION_DURATION_MS;
            progress = constrain(progress, 0.0, 1.0);
            float t = EasingEngine::smoothstep(progress);

            // Morph parameters
            active_period = EasingEngine::lerp(prev.period_ms,
                                              variation.applyToPeriod(current.period_ms), t);
            active_amplitude = EasingEngine::lerp(prev.amplitude,
                                                 variation.applyToAmplitude(current.amplitude), t);
            active_phase = EasingEngine::lerp(prev.phase_difference,
                                             variation.applyToPhase(current.phase_difference), t);

            if (progress >= 1.0) {
                in_transition = false;
            }
        } else {
            active_period = variation.applyToPeriod(current.period_ms);
            active_amplitude = variation.applyToAmplitude(current.amplitude);
            active_phase = variation.applyToPhase(current.phase_difference);
        }

        // Accumulate phase
        float phase = phase_accumulator.accumulate(current_time, active_period);

        // Generate motion
        float time_sec = current_time / 1000.0;
        bool apply_drift = !in_transition;
        motion_generator.generateAngles(phase, active_amplitude, active_phase,
                                       time_sec, apply_drift, angle1, angle2);

        // Apply geometric constraints
        angle2 = GeometryConstraints::constrainAngle(angle1, angle2);
    }

    const char* getCurrentPatternName() const {
        return Patterns::LIBRARY[current_index].name;
    }

    bool isInTransition() const {
        return in_transition;
    }

    float getTransitionProgress(unsigned long current_time) const {
        if (!in_transition) return 0.0;
        return (float)(current_time - transition_start_time) / Config::TRANSITION_DURATION_MS;
    }
};

// ============================================================================
// MAIN APPLICATION
// ============================================================================

Servo servo1;
Servo servo2;
PatternSequencer* sequencer = nullptr;

void setup() {
    Serial.begin(115200);
    delay(1000);

    // Seed random
    randomSeed(analogRead(0) + analogRead(1) * 1000 + millis());
    float noise_seed = random(1000) / 10.0;

    Serial.println("\n=== LUMINOUS FLEXURE - Modular Architecture ===\n");

    // Initialize servos
    servo1.attach(Config::SERVO_PIN_1);
    servo2.attach(Config::SERVO_PIN_2);
    servo1.write(Config::CENTER_ANGLE);
    servo2.write(Config::CENTER_ANGLE);

    // Initialize sequencer
    sequencer = new PatternSequencer(noise_seed);
    sequencer->begin(millis());

    Serial.print("Loaded ");
    Serial.print(Patterns::COUNT);
    Serial.println(" patterns\n");

    Serial.print("Starting: ");
    Serial.println(sequencer->getCurrentPatternName());

    delay(2000);
}

void loop() {
    static unsigned long last_debug = 0;
    unsigned long current_time = millis();

    // Update motion
    int angle1, angle2;
    sequencer->update(current_time, angle1, angle2);

    // Drive servos
    servo1.write(angle1);
    servo2.write(angle2);

    // Debug output
    if (current_time - last_debug > 3000) {
        Serial.print(sequencer->getCurrentPatternName());

        if (sequencer->isInTransition()) {
            float progress = sequencer->getTransitionProgress(current_time) * 100;
            Serial.print(" (morphing ");
            Serial.print(progress, 0);
            Serial.print("%)");
        }

        Serial.print("  S1:");
        Serial.print(angle1);
        Serial.print("° S2:");
        Serial.print(angle2);
        Serial.print("° LED:");
        Serial.print(GeometryConstraints::getLength(angle1, angle2), 1);
        Serial.println("cm");

        last_debug = current_time;
    }

    delay(15);
}
