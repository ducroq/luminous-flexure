#include "motion_engine.h"

MotionEngine::MotionEngine(AccelStepper& m1, AccelStepper& m2) : motor1(m1), motor2(m2) {}

void MotionEngine::moveBothTo(int pos1, int pos2, int speed, int accel) {
    motor1.setMaxSpeed(speed);
    motor2.setMaxSpeed(speed);
    motor1.setAcceleration(accel);
    motor2.setAcceleration(accel);

    motor1.moveTo(pos1);
    motor2.moveTo(pos2);

    // Run both motors until they reach target
    while (motor1.distanceToGo() != 0 || motor2.distanceToGo() != 0) {
        motor1.run();
        motor2.run();
        vTaskDelay(pdMS_TO_TICKS(1));  // Yield to other tasks
    }
}

void MotionEngine::releaseMotors() {
    // Release coils to prevent overheating
    digitalWrite(MOTOR1_PIN1, LOW);
    digitalWrite(MOTOR1_PIN2, LOW);
    digitalWrite(MOTOR1_PIN3, LOW);
    digitalWrite(MOTOR1_PIN4, LOW);
    digitalWrite(MOTOR2_PIN1, LOW);
    digitalWrite(MOTOR2_PIN2, LOW);
    digitalWrite(MOTOR2_PIN3, LOW);
    digitalWrite(MOTOR2_PIN4, LOW);
}

void MotionEngine::init() {
    Serial.println("\n=== MOTOR HOMING SEQUENCE ===");
    Serial.println("Step 1: Retracting to find physical minimum...");

    motor1.setMaxSpeed(1000);
    motor2.setMaxSpeed(1000);
    motor1.setAcceleration(50);
    motor2.setAcceleration(50);

    motor1.move(-500);
    motor2.move(-500);

    // Run until complete with timeout
    unsigned long startTime = millis();
    while ((motor1.distanceToGo() != 0 || motor2.distanceToGo() != 0) &&
           (millis() - startTime < 5000)) {
        motor1.run();
        motor2.run();
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    Serial.printf("Homing took %lu ms\n", millis() - startTime);
    Serial.println("Step 2: Physical minimum reached");

    releaseMotors();
    vTaskDelay(pdMS_TO_TICKS(500));

    // Set zero position
    motor1.setCurrentPosition(0);
    motor2.setCurrentPosition(0);
    Serial.println("Step 3: Zero position set!");

    // Move to center
    Serial.println("Step 4: Moving to center position (125)...");
    motor1.moveTo(125);
    motor2.moveTo(125);

    while (motor1.distanceToGo() != 0 || motor2.distanceToGo() != 0) {
        motor1.run();
        motor2.run();
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    Serial.println("=== HOMING COMPLETE ===");
    Serial.printf("Current position: Motor1=%ld, Motor2=%ld\n",
                  motor1.currentPosition(), motor2.currentPosition());

    releaseMotors();
    vTaskDelay(pdMS_TO_TICKS(500));
}

void MotionEngine::perform(MotionPattern pattern) {
    switch(pattern) {
        case BREATHING: {
            // Slow synchronized in/out - 3 cycles (HIGH SPEED for smooth motion!)
            for(int i = 0; i < 3; i++) {
                moveBothTo(MAX_POSITION * 0.75, MAX_POSITION * 0.75,
                          MotionConfig::FAST_SPEED, MotionConfig::AGGRESSIVE_ACCEL);
                delay(100);
                moveBothTo(MAX_POSITION * 0.25, MAX_POSITION * 0.25,
                          MotionConfig::FAST_SPEED, MotionConfig::AGGRESSIVE_ACCEL);
                delay(100);
            }
            moveBothTo(MAX_POSITION / 2, MAX_POSITION / 2,
                      MotionConfig::FAST_SPEED, MotionConfig::AGGRESSIVE_ACCEL);
            break;
        }

        case SYNCHRONIZED_WAVE: {
            // Both move together - fast and fluid
            moveBothTo(MAX_POSITION * 0.8, MAX_POSITION * 0.8,
                      MotionConfig::FAST_SPEED, MotionConfig::AGGRESSIVE_ACCEL);
            moveBothTo(MAX_POSITION * 0.2, MAX_POSITION * 0.2,
                      MotionConfig::FAST_SPEED, MotionConfig::AGGRESSIVE_ACCEL);
            moveBothTo(MAX_POSITION / 2, MAX_POSITION / 2,
                      MotionConfig::FAST_SPEED, MotionConfig::AGGRESSIVE_ACCEL);
            break;
        }

        case ALTERNATING: {
            // One extends, other retracts - continuous
            for(int i = 0; i < 2; i++) {
                moveBothTo(MAX_POSITION * 0.8, MAX_POSITION * 0.2,
                          MotionConfig::FAST_SPEED, MotionConfig::AGGRESSIVE_ACCEL);
                moveBothTo(MAX_POSITION * 0.2, MAX_POSITION * 0.8,
                          MotionConfig::FAST_SPEED, MotionConfig::AGGRESSIVE_ACCEL);
            }
            moveBothTo(MAX_POSITION / 2, MAX_POSITION / 2,
                      MotionConfig::FAST_SPEED, MotionConfig::AGGRESSIVE_ACCEL);
            break;
        }

        case FIGURE_EIGHT: {
            // Phase-shifted sine-like motion - fast and flowing
            moveBothTo(MAX_POSITION * 0.8, MAX_POSITION * 0.5,
                      MotionConfig::FAST_SPEED, MotionConfig::AGGRESSIVE_ACCEL);
            moveBothTo(MAX_POSITION * 0.5, MAX_POSITION * 0.8,
                      MotionConfig::FAST_SPEED, MotionConfig::AGGRESSIVE_ACCEL);
            moveBothTo(MAX_POSITION * 0.2, MAX_POSITION * 0.5,
                      MotionConfig::FAST_SPEED, MotionConfig::AGGRESSIVE_ACCEL);
            moveBothTo(MAX_POSITION * 0.5, MAX_POSITION * 0.2,
                      MotionConfig::FAST_SPEED, MotionConfig::AGGRESSIVE_ACCEL);
            moveBothTo(MAX_POSITION / 2, MAX_POSITION / 2,
                      MotionConfig::FAST_SPEED, MotionConfig::AGGRESSIVE_ACCEL);
            break;
        }

        case RANDOM_WALK: {
            // Independent random movements - fast and chaotic
            for(int i = 0; i < 5; i++) {
                int pos1 = random(MIN_POSITION, MAX_POSITION);
                int pos2 = random(MIN_POSITION, MAX_POSITION);
                moveBothTo(pos1, pos2, MotionConfig::FAST_SPEED, MotionConfig::AGGRESSIVE_ACCEL);
                delay(random(0, 100));  // Minimal or no delay
            }
            moveBothTo(MAX_POSITION / 2, MAX_POSITION / 2,
                      MotionConfig::FAST_SPEED, MotionConfig::AGGRESSIVE_ACCEL);
            break;
        }

        case TREMOR: {
            // Small rapid vibrations
            int centerPos1 = motor1.currentPosition();
            int centerPos2 = motor2.currentPosition();
            for(int i = 0; i < 15; i++) {
                moveBothTo(centerPos1 + 10, centerPos2 + 10,
                          MotionConfig::FAST_SPEED, MotionConfig::AGGRESSIVE_ACCEL);
                moveBothTo(centerPos1 - 10, centerPos2 - 10,
                          MotionConfig::FAST_SPEED, MotionConfig::AGGRESSIVE_ACCEL);
            }
            moveBothTo(centerPos1, centerPos2,
                      MotionConfig::MEDIUM_SPEED, MotionConfig::NORMAL_ACCEL);
            break;
        }

        case CHASE: {
            // Motor 2 follows motor 1 with brief delay
            moveBothTo(MAX_POSITION * 0.8, motor2.currentPosition(),
                      MotionConfig::FAST_SPEED, MotionConfig::AGGRESSIVE_ACCEL);
            delay(50);
            moveBothTo(motor1.currentPosition(), MAX_POSITION * 0.8,
                      MotionConfig::FAST_SPEED, MotionConfig::AGGRESSIVE_ACCEL);
            delay(50);
            moveBothTo(MAX_POSITION * 0.2, motor2.currentPosition(),
                      MotionConfig::FAST_SPEED, MotionConfig::AGGRESSIVE_ACCEL);
            delay(50);
            moveBothTo(motor1.currentPosition(), MAX_POSITION * 0.2,
                      MotionConfig::FAST_SPEED, MotionConfig::AGGRESSIVE_ACCEL);
            moveBothTo(MAX_POSITION / 2, MAX_POSITION / 2,
                      MotionConfig::FAST_SPEED, MotionConfig::AGGRESSIVE_ACCEL);
            break;
        }

        case STILLNESS: {
            // Brief pause only
            releaseMotors();
            delay(1000);  // Short rest
            break;
        }

        case ORGANIC_DRIFT: {
            // Asynchronous independent random walk with organic timing
            const int minSteps = 5;
            const int maxSteps = 30;
            const int baseUpdateInterval = 150;  // Base timing in ms
            const int durationMs = 8000;  // Run for 8 seconds

            // Track positions (initialize from current)
            int pos1 = motor1.currentPosition();
            int pos2 = motor2.currentPosition();

            // Independent timing for each motor
            unsigned long lastMotion1 = millis();
            unsigned long lastMotion2 = millis();
            unsigned long startTime = millis();

            // Randomize initial intervals (stagger the motors)
            int updateInterval1 = random(baseUpdateInterval * 0.8, baseUpdateInterval * 1.2);
            int updateInterval2 = random(baseUpdateInterval * 0.8, baseUpdateInterval * 1.2);

            while (millis() - startTime < durationMs) {
                unsigned long currentTime = millis();

                // Motor 1 update
                if (currentTime - lastMotion1 >= updateInterval1) {
                    lastMotion1 = currentTime;
                    int steps1 = random(minSteps, maxSteps + 1);

                    // Determine direction based on position bounds
                    int dir1;
                    if (pos1 <= MIN_POSITION) {
                        dir1 = 1;  // Must go forward if at min
                    } else if (pos1 + steps1 >= MAX_POSITION) {
                        dir1 = -1;  // Must go backward if would exceed max
                    } else {
                        dir1 = random(0, 2) == 0 ? -1 : 1;  // Random direction
                    }

                    // Clamp steps to stay within bounds
                    if (dir1 > 0 && pos1 + steps1 > MAX_POSITION) {
                        steps1 = MAX_POSITION - pos1;
                    } else if (dir1 < 0 && pos1 - steps1 < MIN_POSITION) {
                        steps1 = pos1 - MIN_POSITION;
                    }

                    // Move motor 1
                    motor1.move(dir1 * steps1);
                    while (motor1.distanceToGo() != 0) {
                        motor1.run();
                    }
                    pos1 += dir1 * steps1;

                    // Release to prevent overheating
                    digitalWrite(MOTOR1_PIN1, LOW);
                    digitalWrite(MOTOR1_PIN2, LOW);
                    digitalWrite(MOTOR1_PIN3, LOW);
                    digitalWrite(MOTOR1_PIN4, LOW);

                    // Randomize next interval
                    updateInterval1 = random(baseUpdateInterval * 0.5, baseUpdateInterval * 1.5);
                }

                // Motor 2 update (independent timing)
                if (currentTime - lastMotion2 >= updateInterval2) {
                    lastMotion2 = currentTime;
                    int steps2 = random(minSteps, maxSteps + 1);

                    // Determine direction based on position bounds
                    int dir2;
                    if (pos2 <= MIN_POSITION) {
                        dir2 = 1;
                    } else if (pos2 + steps2 >= MAX_POSITION) {
                        dir2 = -1;
                    } else {
                        dir2 = random(0, 2) == 0 ? -1 : 1;
                    }

                    // Clamp steps to stay within bounds
                    if (dir2 > 0 && pos2 + steps2 > MAX_POSITION) {
                        steps2 = MAX_POSITION - pos2;
                    } else if (dir2 < 0 && pos2 - steps2 < MIN_POSITION) {
                        steps2 = pos2 - MIN_POSITION;
                    }

                    // Move motor 2
                    motor2.move(dir2 * steps2);
                    while (motor2.distanceToGo() != 0) {
                        motor2.run();
                    }
                    pos2 += dir2 * steps2;

                    // Release to prevent overheating
                    digitalWrite(MOTOR2_PIN1, LOW);
                    digitalWrite(MOTOR2_PIN2, LOW);
                    digitalWrite(MOTOR2_PIN3, LOW);
                    digitalWrite(MOTOR2_PIN4, LOW);

                    // Randomize next interval
                    updateInterval2 = random(baseUpdateInterval * 0.5, baseUpdateInterval * 1.5);
                }

                // Small delay to prevent busy-waiting
                delay(10);
            }

            // Return to center smoothly
            moveBothTo(MAX_POSITION / 2, MAX_POSITION / 2,
                      MotionConfig::FAST_SPEED, MotionConfig::AGGRESSIVE_ACCEL);
            break;
        }
    }

    releaseMotors();  // Always release after movement
}

MotionPattern selectPatternForState(SculptureState state) {
    switch(state) {
        case SLEEPING:
            return random(0, 100) < 80 ? STILLNESS : BREATHING;

        case AWARE: {
            // Mix of gentle patterns including organic drift
            int choice = random(0, 100);
            if (choice < 40) return ORGANIC_DRIFT;
            else if (choice < 70) return SYNCHRONIZED_WAVE;
            else return BREATHING;
        }

        case PLAYFUL: {
            MotionPattern playful[] = {SYNCHRONIZED_WAVE, ALTERNATING, FIGURE_EIGHT, CHASE, ORGANIC_DRIFT};
            return playful[random(0, 5)];
        }

        case EXCITED: {
            MotionPattern excited[] = {RANDOM_WALK, TREMOR, CHASE, ALTERNATING};
            return excited[random(0, 4)];
        }

        case CONTEMPLATIVE: {
            // Organic drift is perfect for contemplative state
            int choice = random(0, 100);
            if (choice < 50) return ORGANIC_DRIFT;
            else if (choice < 75) return SYNCHRONIZED_WAVE;
            else return BREATHING;
        }

        case RETREATING: {
            // Slow down with organic drift
            int choice = random(0, 100);
            if (choice < 30) return ORGANIC_DRIFT;
            else if (choice < 65) return BREATHING;
            else return SYNCHRONIZED_WAVE;
        }

        default:
            return STILLNESS;
    }
}

long getPauseDurationForState(SculptureState state) {
    switch(state) {
        case SLEEPING:
            return random(3000, 6000);   // Pauses when alone
        case AWARE:
            return random(200, 500);     // Brief pause
        case PLAYFUL:
            return random(100, 300);     // Nearly continuous
        case EXCITED:
            return random(0, 200);       // No pause - constant motion!
        case CONTEMPLATIVE:
            return random(500, 1000);    // Short thoughtful pause
        case RETREATING:
            return random(800, 1500);    // Slowing down slightly
        default:
            return 500;
    }
}
