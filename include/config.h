#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// HARDWARE PIN DEFINITIONS
// ============================================================================

// LD2410 Sensor
#define LD2410_RX_PIN 20
#define LD2410_TX_PIN 21

// LED (ESP32-C3 Super Mini built-in LED)
#define LED_PIN 8  // Built-in LED on most ESP32-C3 boards

// LED Filament (3.3V regulator enable pin)
#define LED_FILAMENT_EN_PIN 10  // Enable pin for LED filament regulator

// Stepper Motor 1 (4-wire)
#define MOTOR1_PIN1 1
#define MOTOR1_PIN2 2
#define MOTOR1_PIN3 3
#define MOTOR1_PIN4 4

// Stepper Motor 2 (4-wire)
#define MOTOR2_PIN1 5
#define MOTOR2_PIN2 6
#define MOTOR2_PIN3 7
#define MOTOR2_PIN4 9

// Note: Avoiding GPIO 0 (boot button), GPIO 20/21 (LD2410)
// GPIO 10 is free for future expansion

// ============================================================================
// MOTION CONFIGURATION
// ============================================================================

// Linear actuator limits
#define MAX_POSITION 250
#define MIN_POSITION 0

// Motion configuration
namespace MotionConfig {
    const int STEPS_PER_REV = 100;  // CD-ROM stepper motor

    // Speeds in steps/second
    const int SLOW_SPEED = 3000;    // Slow movements
    const int MEDIUM_SPEED = 4000;  // Medium movements
    const int FAST_SPEED = 5000;    // Fast, aggressive movements

    // Acceleration in steps/sec² (high for responsive motion)
    const int GENTLE_ACCEL = 150;
    const int NORMAL_ACCEL = 200;
    const int AGGRESSIVE_ACCEL = 250;
}

// ============================================================================
// SENSOR DATA STRUCTURE
// ============================================================================

struct SensorData {
    bool presence;
    uint16_t distance;        // cm
    uint8_t motionEnergy;     // 0-100 (moving target signal)
    uint8_t stationaryEnergy; // 0-100 (stationary target signal)
    unsigned long timestamp;
};

#endif // CONFIG_H
