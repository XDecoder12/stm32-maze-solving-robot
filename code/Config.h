#ifndef CONFIG_H
#define CONFIG_H

// --- PIN DEFINITIONS (FINAL VERIFIED LAYOUT) ---
#define USER_BUTTON_PIN PA0
#define LED_PIN PB2
#define MOTOR_A_IN1 PB12 // Right motor
#define MOTOR_A_IN2 PB13
#define MOTOR_A_PWM PB6
#define MOTOR_B_IN1 PB14 // Left motor
#define MOTOR_B_IN2 PB15
#define MOTOR_B_PWM PB7
#define LEFT_ENCODER_A  PB10
#define RIGHT_ENCODER_A PB5
#define SENSOR_1_PIN PA1
#define SENSOR_2_PIN PA2
#define SENSOR_3_PIN PA3
#define SENSOR_4_PIN PA4
#define SENSOR_5_PIN PA5
#define SENSOR_6_PIN PA6
#define SENSOR_7_PIN PA7
#define SENSOR_8_PIN PB0

#define NUM_SENSORS 8
#define MAX_PATH_LENGTH 100

// --- TUNING PARAMETERS ---

// PID Constants for Analog Error Calculation
// KP: Proportional gain. This is the main tuning parameter. 
//     Start with this value (e.g., 20-40). Decrease if the robot oscillates wildly.
//     Increase if the robot is sluggish and doesn't correct enough.
#define KP 30.0 

// KI: Integral gain. Helps correct for steady-state error. 
//     Keep at 0.0 for now. Only add a very small value (e.g., 0.01) if the bot consistently stays slightly off-center on straight lines.
#define KI 0.0

// KD: Derivative gain. Dampens oscillations and smooths corrections.
//     Start with a value around KP * 2. Increase if the robot overshoots turns, decrease if it's too sluggish to react.
#define KD 60.0

// SPEEDS (0-255)
#define BASE_SPEED 100
#define FAST_SPEED 255
#define TURN_SPEED 100

// MOTOR TRIM
// Adjust this value to make the robot drive straight.
// Positive values slow down the right motor, negative values slow down the left.
// Range: -50 to 50
#define MOTOR_TRIM 0

// ENCODER TICKS (Calibrated Values)
#define TICKS_FOR_90_DEG_TURN 240
#define TICKS_FOR_180_DEG_TURN 480
#define TICKS_SENSORS_TO_AXLE 300

#endif

