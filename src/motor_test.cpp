/*
 * PWM MICROSTEPPING MOTOR DRIVER
 * Smooth motion using sine/cosine current profiles
 * Works with L9110S H-bridges on ESP32
 */

#include <Arduino.h>

// Motor 1 pins (original wiring)
#define M1_A1 1   // Coil A pin 1
#define M1_A2 2   // Coil A pin 2
#define M1_B1 3   // Coil B pin 1
#define M1_B2 4   // Coil B pin 2

// ESP32 LEDC PWM channels (one per pin)
#define CH_A1 0
#define CH_A2 1
#define CH_B1 2
#define CH_B2 3



// PWM settings
const int PWM_FREQ = 20000;      // 20kHz - above audible range
const int PWM_RESOLUTION = 8;    // 8-bit (0-255)

// Motor parameters
const float STEPS_PER_REV = 200.0;        // Full steps per revolution (1.8° motor)
const int MICROSTEPS = 16;                 // Virtual microsteps per full step
const float DEGREES_PER_MICROSTEP = 360.0 / (STEPS_PER_REV * MICROSTEPS);

// Motion parameters
const uint8_t RUN_POWER = 255;            // PWM power when moving (0-255)
const uint8_t HOLD_POWER = 0;            // PWM power when holding position
const float MAX_SPEED = 10000.0;            // Microsteps per second
const float ACCELERATION = 500.0;        // Microsteps per second^2

// State
float currentPosition = 0;      // Current position in microsteps
float targetPosition = 0;       // Target position in microsteps
float currentSpeed = 0;         // Current speed in microsteps/sec
unsigned long lastUpdateMicros = 0;

// Sine table for efficiency (quarter wave, 0-90 degrees)
const uint8_t SINE_TABLE[65] = {
    0, 3, 6, 9, 12, 16, 19, 22, 25, 28, 31, 34, 37, 40, 43, 46,
    49, 51, 54, 57, 60, 63, 65, 68, 71, 73, 76, 78, 81, 83, 85, 88,
    90, 92, 94, 96, 98, 100, 102, 104, 106, 107, 109, 111, 112, 113, 115, 116,
    117, 118, 120, 121, 122, 122, 123, 124, 125, 125, 126, 126, 126, 127, 127, 127, 127
};

// Fast sine lookup (0-255 input = 0-360 degrees, returns -127 to 127)
int8_t fastSin(uint8_t angle) {
    uint8_t quadrant = angle >> 6;  // 0-3
    uint8_t idx = angle & 0x3F;     // 0-63

    if (quadrant & 1) idx = 64 - idx;  // Mirror for quadrants 1,3

    int8_t value = SINE_TABLE[idx];
    if (quadrant >= 2) value = -value;  // Negate for quadrants 2,3

    return value;
}

int8_t fastCos(uint8_t angle) {
    return fastSin(angle + 64);  // cos = sin + 90°
}

// Set coil currents based on electrical angle
void setCoils(float electricalAngle, uint8_t power) {
    // Convert to 0-255 range for lookup
    uint8_t angle = (uint8_t)((electricalAngle / 360.0) * 256.0) & 0xFF;

    // Get sine/cosine values (-127 to 127)
    // Swapped to match coil wiring from testCoils()
    int16_t sinVal = fastSin(angle);
    int16_t cosVal = fastCos(angle);

    // Scale by power (coilA=cos, coilB=sin to match tested sequence)
    int16_t coilA = (cosVal * power) / 127;
    int16_t coilB = (sinVal * power) / 127;

    // Drive coil A
    if (coilA >= 0) {
        ledcWrite(CH_A1, coilA);
        ledcWrite(CH_A2, 0);
    } else {
        ledcWrite(CH_A1, 0);
        ledcWrite(CH_A2, -coilA);
    }

    // Drive coil B
    if (coilB >= 0) {
        ledcWrite(CH_B1, coilB);
        ledcWrite(CH_B2, 0);
    } else {
        ledcWrite(CH_B1, 0);
        ledcWrite(CH_B2, -coilB);
    }
}

// Convert microstep position to electrical angle
float positionToElectricalAngle(float pos) {
    // 4 electrical cycles per full step (for typical 2-phase stepper)
    // Each full step = MICROSTEPS microsteps
    float fullSteps = pos / MICROSTEPS;
    return fmod(fullSteps * 90.0, 360.0);  // 90° electrical per full step
}

// Release motor (no holding torque)
void releaseMotor() {
    ledcWrite(CH_A1, 0);
    ledcWrite(CH_A2, 0);
    ledcWrite(CH_B1, 0);
    ledcWrite(CH_B2, 0);
}

// Hold position with reduced current
void holdPosition() {
    float angle = positionToElectricalAngle(currentPosition);
    setCoils(angle, HOLD_POWER);
}

// Move to target position (call frequently in loop)
bool run() {
    unsigned long now = micros();
    float dt = (now - lastUpdateMicros) / 1000000.0;  // Delta time in seconds
    lastUpdateMicros = now;

    // Clamp dt to avoid issues after long pauses
    if (dt > 0.1) dt = 0.1;

    float distance = targetPosition - currentPosition;

    if (abs(distance) < 0.5) {
        // At target - release coils
        currentSpeed = 0;
        currentPosition = targetPosition;
        releaseMotor();
        return false;
    }

    // Calculate stopping distance at current speed
    float stoppingDistance = (currentSpeed * currentSpeed) / (2.0 * ACCELERATION);

    // Determine target speed
    float targetSpeed;
    if (abs(distance) <= stoppingDistance) {
        // Need to decelerate
        targetSpeed = sqrt(2.0 * ACCELERATION * abs(distance));
    } else {
        // Can accelerate or maintain speed
        targetSpeed = MAX_SPEED;
    }

    // Apply direction
    if (distance < 0) targetSpeed = -targetSpeed;

    // Accelerate/decelerate toward target speed
    float speedDiff = targetSpeed - currentSpeed;
    float maxSpeedChange = ACCELERATION * dt;

    if (abs(speedDiff) <= maxSpeedChange) {
        currentSpeed = targetSpeed;
    } else if (speedDiff > 0) {
        currentSpeed += maxSpeedChange;
    } else {
        currentSpeed -= maxSpeedChange;
    }

    // Update position
    currentPosition += currentSpeed * dt;

    // Set coil currents
    float angle = positionToElectricalAngle(currentPosition);
    setCoils(angle, RUN_POWER);

    return true;
}

void moveTo(float position) {
    targetPosition = position;
}

void moveRelative(float delta) {
    targetPosition = currentPosition + delta;
}

void setCurrentPosition(float position) {
    currentPosition = position;
    targetPosition = position;
}

float getCurrentPosition() {
    return currentPosition;
}

bool isRunning() {
    return abs(targetPosition - currentPosition) >= 0.5;
}

// Homing sequence
void homeMotor() {
    Serial.println("\n=== HOMING SEQUENCE ===");
    Serial.println("Retracting to find physical minimum...");

    // Move slowly backward
    float homeSpeed = MAX_SPEED * 0.2;
    float originalMaxSpeed = MAX_SPEED;

    // Manually drive backward at slow speed
    for (int i = 0; i < 500 * MICROSTEPS; i++) {
        currentPosition -= 1;
        float angle = positionToElectricalAngle(currentPosition);
        setCoils(angle, RUN_POWER * 0.6);
        delayMicroseconds((unsigned long)(1000000.0 / homeSpeed));
    }

    Serial.println("Physical minimum reached");
    releaseMotor();
    delay(500);

    setCurrentPosition(0);
    Serial.println("Zero position set!");

    // Move to center
    Serial.println("Moving to center position...");
    moveTo(125 * MICROSTEPS);
    while (run()) {
        // Wait for motion to complete
    }

    Serial.println("=== HOMING COMPLETE ===");
    Serial.printf("Current position: %.1f microsteps\n\n", currentPosition);
    holdPosition();
    delay(1000);
}

// Simple full-step test to verify wiring
void testCoils() {
    Serial.println("\n=== COIL TEST (4 full steps) ===");
    uint8_t pwr = 200;

    // Full step sequence for bipolar stepper
    // Step 1: A+, B+
    Serial.println("Step 1: A+, B+");
    ledcWrite(CH_A1, pwr); ledcWrite(CH_A2, 0);
    ledcWrite(CH_B1, pwr); ledcWrite(CH_B2, 0);
    delay(500);

    // Step 2: A-, B+
    Serial.println("Step 2: A-, B+");
    ledcWrite(CH_A1, 0); ledcWrite(CH_A2, pwr);
    ledcWrite(CH_B1, pwr); ledcWrite(CH_B2, 0);
    delay(500);

    // Step 3: A-, B-
    Serial.println("Step 3: A-, B-");
    ledcWrite(CH_A1, 0); ledcWrite(CH_A2, pwr);
    ledcWrite(CH_B1, 0); ledcWrite(CH_B2, pwr);
    delay(500);

    // Step 4: A+, B-
    Serial.println("Step 4: A+, B-");
    ledcWrite(CH_A1, pwr); ledcWrite(CH_A2, 0);
    ledcWrite(CH_B1, 0); ledcWrite(CH_B2, pwr);
    delay(500);

    releaseMotor();
    Serial.println("=== Did motor make 4 steps? ===\n");
    delay(1000);
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) delay(10);
    delay(1000);

    Serial.println("\n=== PWM MICROSTEPPING TEST (ESP32) ===");
    Serial.printf("Pins: A(%d,%d) B(%d,%d)\n", M1_A1, M1_A2, M1_B1, M1_B2);
    Serial.printf("Microsteps per full step: %d\n", MICROSTEPS);
    Serial.printf("Max speed: %.0f microsteps/sec\n", MAX_SPEED);
    Serial.printf("Acceleration: %.0f microsteps/sec^2\n", ACCELERATION);

    // Setup ESP32 LEDC PWM channels
    ledcSetup(CH_A1, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(CH_A2, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(CH_B1, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(CH_B2, PWM_FREQ, PWM_RESOLUTION);

    // Attach pins to channels
    ledcAttachPin(M1_A1, CH_A1);
    ledcAttachPin(M1_A2, CH_A2);
    ledcAttachPin(M1_B1, CH_B1);
    ledcAttachPin(M1_B2, CH_B2);

    // TEST: Verify coils work before doing anything else
    // testCoils();

    lastUpdateMicros = micros();

    // Home the motor
    // homeMotor();

    Serial.println("Starting oscillation test...");
    moveTo(200 * MICROSTEPS);
}

void loop() {
    static bool forward = true;
    static bool wasRunning = true;

    bool running = run();

    if (wasRunning && !running) {
        // Just stopped
        Serial.printf("Reached position %.0f\n", currentPosition / MICROSTEPS);
        holdPosition();
        delay(1000);

        // Change direction
        if (forward) {
            Serial.println("→ Moving backward to 0...");
            moveTo(0);
            forward = false;
        } else {
            Serial.println("→ Moving forward to 200...");
            moveTo(200 * MICROSTEPS);
            forward = true;
        }
    }

    wasRunning = running;
}
