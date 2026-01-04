#ifndef CONFIG_H
#define CONFIG_H

// --- PIN DEFINITIONS (FINAL VERIFIED LAYOUT) ---
#define USER_BUTTON_PIN PA0
#define LED_PIN PB2         // Onboard LED for status (like calibration)
#define GOAL_LED_PIN PB11   // New External LED for goal *only*
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
#define KP_TRIAL 45.0 //40.0 
#define KI 0.0
#define KD_TRIAL 120.0 //165.89 //120.0 //67.5

#define KP_FAST 40 //50 //80.0
#define KD_FAST 70 //80 //410.0

// Anti-windup clamp for Integral (used if KI > 0)
#define INTEGRAL_MAX 1000 

// Proportional gain for straight-line encoder PID
#define K_STRAIGHT 0.5

// SPEEDS (0-255)
#define BASE_SPEED 130
#define FAST_SPEED 230
#define TURN_SPEED 100

#define FAST_TURN_SPEED 90

// MOTOR TRIM
#define MOTOR_TRIM 0

// ENCODER TICKS (Your Calibrated Values)
#define TICKS_FOR_90_DEG_TURN 255 //240
#define TICKS_FOR_180_DEG_TURN 510 //480
#define TICKS_SENSORS_TO_AXLE 280 //250         //235 //155.56

#define FAST_TICKS_SENSORS_TO_AXLE 255 //180 //130     //200 //210

#endif