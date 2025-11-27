#include <Arduino.h>
#include <Servo.h>

/*
 * LUMINOUS FLEXURE - Kinetic Light Sculpture with Organic Motion
 *
 * RECOMMENDED PHYSICAL SETUP:
 *   - Mount servos on horizontal baseline (side by side)
 *   - Servos pointing UPWARD (90° = arms straight up)
 *   - LED filament hangs between arms with gravity
 *   - Creates graceful catenary curves
 *
 * MOTION CHARACTERISTICS:
 *   - S-curve easing: Smooth acceleration/deceleration at direction reversals (no snapping)
 *   - Parameter morphing: Patterns evolve into each other (speed, amplitude, phase shift gradually)
 *   - Organic drift: Continuous subtle variation using layered smooth noise
 *   - Random pattern selection: Never repeats same pattern twice consecutively
 *   - Random variations: Each pattern instance is unique (±20% period, ±15% amplitude/phase, ±25% duration)
 *
 * PATTERN LIBRARY:
 *   7 motion presets that are randomly selected and varied
 *   Transitions create smooth metamorphosis between different motion qualities
 */

// --- HARDWARE CONFIGURATION ---
const int SERVO_PIN_1 = 9;
const int SERVO_PIN_2 = 10;

// --- PHYSICAL GEOMETRY ---
const float SERVO_SEPARATION = 25.0;  // cm between servos
const float ARM_LENGTH = 11.0;        // cm (average of 10-12cm)
const float LED_MAX_LENGTH = 30.0;    // cm - maximum stretch
const float LED_MIN_LENGTH = 8.0;     // cm - minimum to prevent slack bunching

// Servo motion range (will be dynamically constrained based on geometry)
const int MIN_ANGLE = 0;
const int MAX_ANGLE = 180;

// --- MOTION PATTERN SYSTEM ---
// Center position for motion (90° = straight up)
const int CENTER_ANGLE = 90;

// Define motion patterns
struct MotionPattern {
    const char* name;
    long period_ms;
    int phase_difference;
    float amplitude;
    long duration_seconds;  // How long to stay in this pattern
};

// Pattern library - sculpture cycles through these
const MotionPattern PATTERNS[] = {
    {"Gentle Breathing",   8000,   0, 0.30,  30},  // Synchronized, meditative
    {"Slow Wave",          8000,  60, 0.40,  40},  // Rolling wave
    {"Quick Ripple",       3000,  45, 0.25,  25},  // Fast, delicate
    {"Figure-8",          10000, 180, 0.45,  45},  // Opposite motion, graceful
    {"Deep Meditation",   15000,  90, 0.35,  50},  // Very slow quarter-wave
    {"Synchronized Pump",  5000,   0, 0.50,  30},  // Breathing with more amplitude
    {"Asymmetric Wave",    7000, 120, 0.38,  35},  // Unusual phase relationship
};

const int NUM_PATTERNS = sizeof(PATTERNS) / sizeof(PATTERNS[0]);

// Current pattern tracking
int current_pattern_index = 0;
unsigned long pattern_start_time = 0;

// Smooth transition parameters
bool in_transition = false;
unsigned long transition_start = 0;
const long TRANSITION_DURATION = 5000;  // 5 seconds to blend between patterns
int prev_pattern_index = 0;

// Randomness system
float current_period_variation = 1.0;
float current_amplitude_variation = 1.0;
int current_phase_variation = 0;
float current_duration_variation = 1.0;

// Perlin-like noise for organic drift
unsigned long noise_seed = 0;
float noise_offset = 0.0;

// Phase accumulation for smooth morphing
float accumulated_phase = 0.0;  // Radians
unsigned long last_update_time = 0;

Servo servo1;
Servo servo2;

// Simple random float between min and max
float randomFloat(float min_val, float max_val) {
    return min_val + (random(10000) / 10000.0) * (max_val - min_val);
}

// Smooth organic noise using sine waves (simpler than Perlin but effective)
float smoothNoise(float t, float frequency) {
    return sin(t * frequency) * 0.5 + 0.5;  // Returns 0.0 to 1.0
}

// Select next random pattern (avoid repeating the same pattern twice)
int selectRandomPattern(int current_index) {
    if (NUM_PATTERNS <= 1) return 0;

    int next_index;
    do {
        next_index = random(NUM_PATTERNS);
    } while (next_index == current_index);

    return next_index;
}

// Generate random variations for the selected pattern
void generatePatternVariations() {
    // Period variation: ±20%
    current_period_variation = randomFloat(0.8, 1.2);

    // Amplitude variation: ±15%
    current_amplitude_variation = randomFloat(0.85, 1.15);

    // Phase variation: ±15 degrees
    current_phase_variation = random(-15, 16);

    // Duration variation: ±25%
    current_duration_variation = randomFloat(0.75, 1.25);
}

// Calculate distance between LED endpoints given two servo angles
float calculateLEDLength(int angle1, int angle2) {
    // Convert servo angles to radians (90° = perpendicular/straight forward)
    // Subtract 90 so that 90° servo = 0° geometric = perpendicular to baseline
    float theta1 = radians(angle1 - 90);
    float theta2 = radians(angle2 - 90);

    // Endpoint positions (servo 1 at origin, servo 2 at distance D)
    float x1 = ARM_LENGTH * cos(theta1);
    float y1 = ARM_LENGTH * sin(theta1);

    float x2 = SERVO_SEPARATION + ARM_LENGTH * cos(theta2);
    float y2 = ARM_LENGTH * sin(theta2);

    // Distance between endpoints
    float dx = x2 - x1;
    float dy = y2 - y1;

    return sqrt(dx * dx + dy * dy);
}

// Check if angle combination is geometrically valid
bool isValidConfiguration(int angle1, int angle2) {
    float length = calculateLEDLength(angle1, angle2);
    return (length >= LED_MIN_LENGTH && length <= LED_MAX_LENGTH);
}

// Constrain angles to keep LED within safe length
// Returns adjusted angle2 based on angle1
int constrainAngle2(int angle1, int desired_angle2) {
    // Start with desired angle
    int safe_angle2 = desired_angle2;

    // Check if valid
    if (isValidConfiguration(angle1, safe_angle2)) {
        return safe_angle2;
    }

    // If not valid, search for nearest valid angle
    // Try reducing range symmetrically around 90° (center position)
    int center = 90;
    int offset = abs(safe_angle2 - center);

    // Reduce offset until valid
    for (int test_offset = offset; test_offset >= 0; test_offset -= 2) {
        int test_angle = (safe_angle2 >= center) ? center + test_offset : center - test_offset;
        test_angle = constrain(test_angle, MIN_ANGLE, MAX_ANGLE);

        if (isValidConfiguration(angle1, test_angle)) {
            return test_angle;
        }
    }

    // Fallback to center position
    return center;
}

// Linear interpolation for smooth transitions
float lerp(float a, float b, float t) {
    return a + (b - a) * t;
}

// Smoothstep ease-in-out function for S-curve motion
float smoothstep(float x) {
    // Clamp to 0-1 range
    x = constrain(x, 0.0, 1.0);
    // Smoothstep formula: 3x² - 2x³
    return x * x * (3.0 - 2.0 * x);
}

// Apply S-curve easing to sine wave output
// Converts linear sine motion into smooth acceleration/deceleration
float easedSine(float sin_value) {
    // Convert sine value from [-1, 1] to [0, 1] range
    float normalized = (sin_value + 1.0) / 2.0;

    // Apply smoothstep easing
    float eased = smoothstep(normalized);

    // Convert back to [-1, 1] range
    return (eased * 2.0) - 1.0;
}

// Calculate servo angle using accumulated phase (prevents morphing discontinuities)
int calculateAngleFromPhase(float base_phase, int phase_offset, float amplitude, long time_ms, bool add_drift = true, bool apply_easing = true) {
    // Add phase offset
    float phase_rad = ((float)phase_offset / 360.0) * (2.0 * PI);
    float total_phase = base_phase + phase_rad;

    // Calculate sine value from phase
    float sin_value = sin(total_phase);

    // Apply S-curve easing for smooth acceleration/deceleration
    if (apply_easing) {
        sin_value = easedSine(sin_value);
    }

    // Add subtle organic drift using layered noise
    if (add_drift) {
        float t = time_ms / 1000.0;  // Time in seconds
        float drift1 = smoothNoise(t + noise_offset, 0.05) - 0.5;  // Very slow drift
        float drift2 = smoothNoise(t + noise_offset + 100, 0.2) - 0.5;  // Medium drift
        float drift = (drift1 * 0.1 + drift2 * 0.05);  // Combine and scale
        sin_value += drift;
        sin_value = constrain(sin_value, -1.0, 1.0);
    }

    // Calculate swing range based on amplitude
    float full_range = MAX_ANGLE - MIN_ANGLE;
    float swing = (full_range / 2.0) * amplitude;

    // Center the motion around CENTER_ANGLE and apply sine wave
    int angle = CENTER_ANGLE + (sin_value * swing);

    return constrain(angle, MIN_ANGLE, MAX_ANGLE);
}

// Calculate angles with parameter morphing using phase accumulation
void calculateBlendedAngles(long current_time, int& angle1, int& angle2) {
    const MotionPattern& current = PATTERNS[current_pattern_index];

    // Apply random variations to current pattern parameters
    long varied_period = current.period_ms * current_period_variation;
    float varied_amplitude = current.amplitude * current_amplitude_variation;
    int varied_phase = current.phase_difference + current_phase_variation;

    // Calculate time delta for phase accumulation
    if (last_update_time == 0) {
        last_update_time = current_time;
    }
    float dt_ms = current_time - last_update_time;
    last_update_time = current_time;

    // Determine current period based on transition state
    long active_period;
    float active_amplitude;
    int active_phase;

    if (in_transition) {
        // TRUE PARAMETER MORPHING - interpolate the parameters themselves
        const MotionPattern& prev = PATTERNS[prev_pattern_index];

        float transition_progress = (float)(current_time - transition_start) / TRANSITION_DURATION;
        transition_progress = constrain(transition_progress, 0.0, 1.0);

        // Ease-in-out interpolation (smoother than linear)
        float t = transition_progress * transition_progress * (3.0 - 2.0 * transition_progress);

        // Morph parameters gradually
        active_period = lerp((float)prev.period_ms, (float)varied_period, t);
        active_amplitude = lerp(prev.amplitude, varied_amplitude, t);
        active_phase = lerp((float)prev.phase_difference, (float)varied_phase, t);

        // End transition
        if (transition_progress >= 1.0) {
            in_transition = false;
        }
    } else {
        // Normal operation - use current pattern with variations
        active_period = varied_period;
        active_amplitude = varied_amplitude;
        active_phase = varied_phase;
    }

    // Accumulate phase based on current period
    // phase_increment = (time_delta / period) * 2π
    float phase_increment = (dt_ms / (float)active_period) * (2.0 * PI);
    accumulated_phase += phase_increment;

    // Keep phase in reasonable range (prevent overflow)
    while (accumulated_phase > 2.0 * PI) {
        accumulated_phase -= 2.0 * PI;
    }

    // Calculate angles from accumulated phase
    bool add_drift = !in_transition;  // Disable drift during transition
    angle1 = calculateAngleFromPhase(accumulated_phase, 0, active_amplitude, current_time, add_drift, true);
    angle2 = calculateAngleFromPhase(accumulated_phase, active_phase, active_amplitude, current_time, add_drift, true);
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    // Seed random number generator with analog noise
    randomSeed(analogRead(0) + analogRead(1) * 1000 + millis());
    noise_seed = random(10000);
    noise_offset = random(1000) / 10.0;

    Serial.println("\n=== LUMINOUS FLEXURE - Organic Motion System ===");
    Serial.print("Geometry: ");
    Serial.print(SERVO_SEPARATION);
    Serial.print("cm separation, ");
    Serial.print(ARM_LENGTH);
    Serial.println("cm arms");

    Serial.print("LED: ");
    Serial.print(LED_MIN_LENGTH);
    Serial.print("-");
    Serial.print(LED_MAX_LENGTH);
    Serial.println("cm range\n");

    Serial.print("Loaded ");
    Serial.print(NUM_PATTERNS);
    Serial.println(" motion patterns:");
    for (int i = 0; i < NUM_PATTERNS; i++) {
        Serial.print("  ");
        Serial.print(i + 1);
        Serial.print(". ");
        Serial.print(PATTERNS[i].name);
        Serial.print(" (~");
        Serial.print(PATTERNS[i].duration_seconds);
        Serial.println("s)");
    }
    Serial.println("\nMotion features:");
    Serial.println("  - S-curve easing for smooth reversals");
    Serial.println("  - Parameter morphing between patterns");
    Serial.println("  - Organic drift and random variations\n");

    servo1.attach(SERVO_PIN_1);
    servo2.attach(SERVO_PIN_2);

    // Start at center position
    servo1.write(CENTER_ANGLE);
    servo2.write(CENTER_ANGLE);

    float initial_length = calculateLEDLength(CENTER_ANGLE, CENTER_ANGLE);
    Serial.print("Starting LED length: ");
    Serial.print(initial_length);
    Serial.println("cm\n");

    // Initialize with first random pattern
    current_pattern_index = random(NUM_PATTERNS);
    generatePatternVariations();
    pattern_start_time = millis();
    last_update_time = millis();  // Initialize phase accumulation timer
    accumulated_phase = 0.0;

    Serial.print(">>> Starting with: ");
    Serial.println(PATTERNS[current_pattern_index].name);

    delay(2000);
}

void loop() {
    static unsigned long last_debug = 0;
    long current_time = millis();

    // Check if it's time to transition to next pattern
    unsigned long time_in_pattern = (current_time - pattern_start_time) / 1000;  // seconds
    const MotionPattern& current = PATTERNS[current_pattern_index];

    // Apply random duration variation
    long varied_duration = current.duration_seconds * current_duration_variation;

    if (!in_transition && time_in_pattern >= (unsigned long)varied_duration) {
        // Start transition to next RANDOM pattern
        prev_pattern_index = current_pattern_index;
        current_pattern_index = selectRandomPattern(current_pattern_index);

        // Generate new random variations for the new pattern
        generatePatternVariations();

        in_transition = true;
        transition_start = current_time;
        pattern_start_time = current_time;

        Serial.print("\n>>> Morphing to: ");
        Serial.print(PATTERNS[current_pattern_index].name);
        Serial.print(" (");
        Serial.print(PATTERNS[current_pattern_index].duration_seconds * current_duration_variation, 0);
        Serial.println("s)");
    }

    // Calculate angles with pattern blending
    int desired_angle1, desired_angle2;
    calculateBlendedAngles(current_time, desired_angle1, desired_angle2);

    // Apply geometric constraints
    int safe_angle1 = desired_angle1;
    int safe_angle2 = constrainAngle2(safe_angle1, desired_angle2);

    // Update servo positions
    servo1.write(safe_angle1);
    servo2.write(safe_angle2);

    // Debug output every 3 seconds
    if (current_time - last_debug > 3000) {
        float led_length = calculateLEDLength(safe_angle1, safe_angle2);

        Serial.print(PATTERNS[current_pattern_index].name);
        if (in_transition) {
            float progress = (float)(current_time - transition_start) / TRANSITION_DURATION * 100.0;
            Serial.print(" (morphing ");
            Serial.print(progress, 0);
            Serial.print("%)");
        } else {
            Serial.print(" [");
            Serial.print(time_in_pattern);
            Serial.print("/");
            Serial.print(varied_duration);
            Serial.print("s]");
        }

        Serial.print("  S1:");
        Serial.print(safe_angle1);
        Serial.print("° S2:");
        Serial.print(safe_angle2);
        Serial.print("° LED:");
        Serial.print(led_length, 1);
        Serial.print("cm");

        // Show active variations
        if (!in_transition && (current_period_variation != 1.0 || current_amplitude_variation != 1.0)) {
            Serial.print(" [varied]");
        }

        Serial.println();

        last_debug = current_time;
    }

    delay(15);
}