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
#define KP 45.0 
#define KI 0.0
#define KD 100.0

// Anti-windup clamp for Integral (used if KI > 0)
#define INTEGRAL_MAX 1000 

// Proportional gain for straight-line encoder PID
// Tune this to make the robot drive straight in forwardEncoderPID
#define K_STRAIGHT 0.5

// SPEEDS (0-255)
#define BASE_SPEED 100
#define FAST_SPEED 255
#define TURN_SPEED 100

// MOTOR TRIM
#define MOTOR_TRIM 0

// ENCODER TICKS (Your Calibrated Values)
#define TICKS_FOR_90_DEG_TURN 230
#define TICKS_FOR_180_DEG_TURN 460  // (2 * 230)
#define TICKS_SENSORS_TO_AXLE 235

#endif