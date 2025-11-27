#ifndef SCULPTURE_TASKS_H
#define SCULPTURE_TASKS_H

#include <Arduino.h>
#include "MyLD2410.h"
#include "config.h"
#include "state_machine.h"
#include "motion_engine.h"

// ============================================================================
// FREERTOS TASK DECLARATIONS
// ============================================================================

void sensorTask(void* parameter);
void sculptureTask(void* parameter);
void controlTask(void* parameter);

// ============================================================================
// GLOBAL TASK RESOURCES
// ============================================================================

extern QueueHandle_t sensorQueue;
extern MyLD2410 sensor;
extern AccelStepper motor1;
extern AccelStepper motor2;
extern StateMachine stateMachine;

#endif // SCULPTURE_TASKS_H
