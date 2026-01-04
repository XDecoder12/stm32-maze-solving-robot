#ifndef CONFIG_H
#define CONFIG_H

#define USER_BUTTON_PIN PA0
#define LED_PIN PB2
#define GOAL_LED_PIN PB11
#define MOTOR_A_IN1 PB12
#define MOTOR_A_IN2 PB13
#define MOTOR_A_PWM PB6
#define MOTOR_B_IN1 PB14
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

#define KP_TRIAL 45.0
#define KI 0.0
#define KD_TRIAL 150.0

#define KP_FAST 50
#define KD_FAST 80

#define INTEGRAL_MAX 1000 

#define K_STRAIGHT 0.5

#define BASE_SPEED 130
#define FAST_SPEED 220
#define TURN_SPEED 100

#define FAST_TURN_SPEED 80

#define MOTOR_TRIM 0

#define TICKS_FOR_90_DEG_TURN 255
#define TICKS_FOR_180_DEG_TURN 510
#define TICKS_SENSORS_TO_AXLE 280

#define FAST_TICKS_SENSORS_TO_AXLE 240

#endif