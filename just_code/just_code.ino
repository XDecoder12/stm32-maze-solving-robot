#include "config.h"

#define DEBUG 0

uint16_t sensor_pins[NUM_SENSORS] = {SENSOR_8_PIN, SENSOR_7_PIN, SENSOR_6_PIN, SENSOR_5_PIN, SENSOR_4_PIN, SENSOR_3_PIN, SENSOR_2_PIN, SENSOR_1_PIN};
uint16_t sensor_values[NUM_SENSORS];
uint16_t sensor_min[NUM_SENSORS];
uint16_t sensor_max[NUM_SENSORS];
uint16_t sensor_thresholds[NUM_SENSORS];
bool sensor_digital[NUM_SENSORS];

volatile long left_encoder_ticks = 0;
volatile long right_encoder_ticks = 0;

float error = 0, last_error = 0;
float integral = 0, derivative = 0;
int motor_speed_correction;

float current_kp;
float current_kd;

char path[MAX_PATH_LENGTH];
int path_length = 0;
bool goal_reached = false;
char simplified_path[MAX_PATH_LENGTH];
int simplified_path_length = 0;

enum RobotState { IDLE, CALIBRATING_SENSORS, SOLVING_MAZE, FAST_RUN, STOPPED, FINISHED };
RobotState currentState = IDLE;

bool sensor_calibration_done = false;

void readSensorsAnalog();
void calibrateSensors();
void waitForButtonPress();
void readSensorsDigital();
float calculateError();
void pidControl(int base_speed);
void motorControl(int left_speed, int right_speed);
void stopMotors();
void forwardEncoderPID(int speed, long ticks);
void turnLeft();
void turnRight();
void turnAround();
void followSegment(int speed);
void solveMaze();
void simplifyPath();
void runFast();
void printPath(char* p, int len);
void leftEncoderISR();
void rightEncoderISR();

void setup() {
    Serial.begin(115200);
    while(!Serial && millis() < 2000);

    pinMode(LED_PIN, OUTPUT);
    pinMode(GOAL_LED_PIN, OUTPUT);
    pinMode(USER_BUTTON_PIN, INPUT_PULLDOWN);

    pinMode(MOTOR_A_IN1, OUTPUT);
    pinMode(MOTOR_A_IN2, OUTPUT);
    pinMode(MOTOR_A_PWM, PWM);
    pinMode(MOTOR_B_IN1, OUTPUT);
    pinMode(MOTOR_B_IN2, OUTPUT);
    pinMode(MOTOR_B_PWM, PWM);
    
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensor_min[i] = 0;
        sensor_max[i] = 4095;
    }

    pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
    pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), leftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightEncoderISR, RISING);

    digitalWrite(LED_PIN, LOW);
    digitalWrite(GOAL_LED_PIN, LOW);
}

void loop() {
    switch (currentState) {
        case IDLE:
            if (digitalRead(USER_BUTTON_PIN) == HIGH) {
                waitForButtonPress();
                currentState = CALIBRATING_SENSORS;
            }
            break;

        case CALIBRATING_SENSORS:
            digitalWrite(LED_PIN, HIGH);
            calibrateSensors();
            digitalWrite(LED_PIN, LOW);
            sensor_calibration_done = true;
            currentState = STOPPED;
            break;

        case SOLVING_MAZE:
            solveMaze();
            simplifyPath();
            currentState = STOPPED;
            break;

        case FAST_RUN:
            runFast();
            currentState = STOPPED;
            break;

        case STOPPED:
            stopMotors();
            
            if (goal_reached) {
                digitalWrite(LED_PIN, HIGH);
                digitalWrite(GOAL_LED_PIN, HIGH);
            }

            if (digitalRead(USER_BUTTON_PIN) == HIGH) {
                waitForButtonPress();
                
                digitalWrite(LED_PIN, LOW);
                digitalWrite(GOAL_LED_PIN, LOW);
                
                if (!sensor_calibration_done) {
                    currentState = CALIBRATING_SENSORS;
                } else if (!goal_reached) {
                    currentState = SOLVING_MAZE;
                } else {
                    currentState = FAST_RUN;
                }
            }
            break;
        
        case FINISHED:
            stopMotors();
            digitalWrite(LED_PIN, HIGH);
            digitalWrite(GOAL_LED_PIN, HIGH);
            break;
    }
}

void readSensorsAnalog() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensor_values[i] = analogRead(sensor_pins[i]);
    }
}

void calibrateSensors() {
    left_encoder_ticks = 0;
    right_encoder_ticks = 0;
    motorControl(TURN_SPEED, -TURN_SPEED);
    while( (left_encoder_ticks + right_encoder_ticks) / 2 < (TICKS_FOR_180_DEG_TURN / 3) ) {
            readSensorsAnalog();
        for (int k = 0; k < NUM_SENSORS; k++) {
            if (sensor_values[k] < sensor_min[k]) sensor_min[k] = sensor_values[k];
            if (sensor_values[k] > sensor_max[k]) sensor_max[k] = sensor_values[k];
        }
    }

    left_encoder_ticks = 0;
    right_encoder_ticks = 0;
    motorControl(-TURN_SPEED, TURN_SPEED);
    while( (left_encoder_ticks + right_encoder_ticks) / 2 < ((TICKS_FOR_180_DEG_TURN / 3))*2 ) {
        readSensorsAnalog();
        for (int k = 0; k < NUM_SENSORS; k++) {
            if (sensor_values[k] < sensor_min[k]) sensor_min[k] = sensor_values[k];
            if (sensor_values[k] > sensor_max[k]) sensor_max[k] = sensor_values[k];
        }
    }

    left_encoder_ticks = 0;
    right_encoder_ticks = 0;
    motorControl(TURN_SPEED, -TURN_SPEED);
    while( (left_encoder_ticks + right_encoder_ticks) / 2 < (TICKS_FOR_180_DEG_TURN / 3) ) {
        readSensorsAnalog();
        for (int k = 0; k < NUM_SENSORS; k++) {
            if (sensor_values[k] < sensor_min[k]) sensor_min[k] = sensor_values[k];
            if (sensor_values[k] > sensor_max[k]) sensor_max[k] = sensor_values[k];
        }
    }
    stopMotors();

    for (int i = 0; i < NUM_SENSORS; i++) {
        if(sensor_max[i] > sensor_min[i]) {
            sensor_thresholds[i] = sensor_min[i] + (uint16_t)((sensor_max[i] - sensor_min[i]) * 0.45);
        } else {
            sensor_thresholds[i] = 2048;
        }
    }
}

void waitForButtonPress() {
    delay(200);
    while(digitalRead(USER_BUTTON_PIN) == LOW);
    delay(50);
    while(digitalRead(USER_BUTTON_PIN) == HIGH);
    delay(50);
}


void readSensorsDigital() {
    readSensorsAnalog();
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensor_digital[i] = (sensor_values[i] < sensor_thresholds[i]);
    }
}

float calculateError() {
    readSensorsAnalog();
    int on_line_count = 0;
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensor_digital[i] = (sensor_values[i] < sensor_thresholds[i]);
        if(sensor_digital[i]) {
            on_line_count++;
        }
    }

    if (on_line_count == 0) {
        integral = 0;
        return ( last_error > 0 ? 3.5 : -3.5 );
    }

    float weighted_sum = 0;
    float activation_sum = 0;
    
    for (int i = 0; i < NUM_SENSORS; i++) {
        float activation = 0;
        if (sensor_digital[i]) {
            if (sensor_thresholds[i] > sensor_min[i]) {
                 activation = (float)(sensor_thresholds[i] - sensor_values[i]) / (sensor_thresholds[i] - sensor_min[i]);
            } else {
                 activation = 1.0;
            }
        }
        
        weighted_sum += activation * i;
        activation_sum += activation;
    }
    
    if (activation_sum == 0) {
        return last_error;
    }

    float position = weighted_sum / activation_sum;
    return 3.5 - position;
}

void pidControl(int base_speed) {
    error = calculateError();
    
    if (KI != 0.0) {
      integral += error;
      if (integral > INTEGRAL_MAX) integral = INTEGRAL_MAX;
      if (integral < -INTEGRAL_MAX) integral = -INTEGRAL_MAX;
    }
    
    if ((error > 0 && last_error < 0) || (error < 0 && last_error > 0)) {
        integral = 0;
    }
    
    derivative = error - last_error;
    last_error = error;

    motor_speed_correction = (current_kp * error) + (KI * integral) + (current_kd * derivative);

    int left_speed = base_speed + motor_speed_correction;
    int right_speed = base_speed - motor_speed_correction;

    motorControl(left_speed, right_speed);
}

void motorControl(int left_speed, int right_speed) {
    left_speed += MOTOR_TRIM;
    right_speed -= MOTOR_TRIM;

    left_speed = constrain(left_speed, -255, 255);
    right_speed = constrain(right_speed, -255, 255);
    
    if (left_speed >= 0) {
        digitalWrite(MOTOR_B_IN1, LOW); digitalWrite(MOTOR_B_IN2, HIGH);
    } else {
        digitalWrite(MOTOR_B_IN1, HIGH); digitalWrite(MOTOR_B_IN2, LOW);
    }
    analogWrite(MOTOR_B_PWM, abs(left_speed));

    if (right_speed >= 0) {
        digitalWrite(MOTOR_A_IN1, LOW); digitalWrite(MOTOR_A_IN2, HIGH);
    } else {
        digitalWrite(MOTOR_A_IN1, HIGH); digitalWrite(MOTOR_A_IN2, LOW);
    }
    analogWrite(MOTOR_A_PWM, abs(right_speed));
}

void stopMotors() {
    motorControl(0, 0);
}

void forwardEncoderPID(int speed, long ticks) {
    left_encoder_ticks = 0;
    right_encoder_ticks = 0;

    long encoder_error = 0;
    int correction = 0;

    while( (left_encoder_ticks + right_encoder_ticks) / 2 < ticks) {
        encoder_error = left_encoder_ticks - right_encoder_ticks;
        correction = (int)(K_STRAIGHT * encoder_error);
        motorControl(speed - correction, speed + correction);
    }
}

void turnLeft() {
    left_encoder_ticks = 0;
    right_encoder_ticks = 0;

    motorControl(TURN_SPEED, -TURN_SPEED); 

    delay(300);
    while(true) {
        readSensorsDigital();
        
        if (sensor_digital[3] || sensor_digital[4]) {
            break;
        }
    }
    stopMotors();
    
    delay(100);
}

void fastturnLeft() {
    left_encoder_ticks = 0;
    right_encoder_ticks = 0;

    motorControl(FAST_TURN_SPEED, -FAST_TURN_SPEED); 

    delay(150); 

    unsigned long safety_timer = millis();
    while(true) {
        readSensorsDigital();
        if (!sensor_digital[3] && !sensor_digital[4]) break;
        
        if(millis() - safety_timer > 500) break; 
    }

    while(true) {
        readSensorsDigital();
        if (sensor_digital[3] || sensor_digital[4]) {
            stopMotors(); 
            break; 
        }
    }
    delay(50);
}

void turnRight() {
    left_encoder_ticks = 0;
    right_encoder_ticks = 0;

    motorControl(-TURN_SPEED, TURN_SPEED); 

    delay(300);
    while(true) {
        readSensorsDigital();
        
        if (sensor_digital[3] || sensor_digital[4]) {
            break;
        }
    }
    stopMotors();
    
    delay(100);
}

void fastturnRight() {
    left_encoder_ticks = 0;
    right_encoder_ticks = 0;
    motorControl(-FAST_TURN_SPEED, FAST_TURN_SPEED); 

    delay(150); 

    unsigned long safety_timer = millis();
    while(true) {
        readSensorsDigital();
        if (!sensor_digital[3] && !sensor_digital[4]) break;
        if(millis() - safety_timer > 500) break; 
    }

    while(true) {
        readSensorsDigital();
        if (sensor_digital[3] || sensor_digital[4]) {
            stopMotors(); 
            break; 
        }
    }
    delay(50);
}

void turnAround() {
    left_encoder_ticks = 0;
    right_encoder_ticks = 0;

    motorControl(-TURN_SPEED, TURN_SPEED); 

    delay(450); 
    while(true) {
        readSensorsDigital();
        
        if (sensor_digital[3] || sensor_digital[4]) {
            break;
        }
    }

    stopMotors();
    delay(100);
}

void followSegment(int speed) {
    left_encoder_ticks = 0;
    right_encoder_ticks = 0;

    while (true) {
        pidControl(speed);
        
        int sensor_count = 0;
        for (int i = 0; i < NUM_SENSORS; i++) {
            if (sensor_digital[i]) sensor_count++;
        }
        
        if (sensor_count >= 5 || sensor_count == 0) { 
            return;
        }
    }
}

void solveMaze() {

    current_kp = KP_TRIAL;
    current_kd = KD_TRIAL;

    while (!goal_reached) {
        followSegment(BASE_SPEED);

        stopMotors();
        delay(25);


        readSensorsDigital();
        bool can_go_left = sensor_digital[0] || sensor_digital[1];
        bool can_go_right = sensor_digital[6] || sensor_digital[7];

        long peek_ticks = 60; 
        forwardEncoderPID(BASE_SPEED, peek_ticks);
        stopMotors();
        delay(50);

        readSensorsDigital();
        int new_sensor_count = 0;
        for(int i = 0; i < NUM_SENSORS; i++) {
            if(sensor_digital[i]) new_sensor_count++;
        }

        if (new_sensor_count >= 6) {
            goal_reached = true;
            continue; 
        }
        
        bool can_go_straight = sensor_digital[3] || sensor_digital[4];
        
        long remaining_ticks = TICKS_SENSORS_TO_AXLE - peek_ticks;
        if (remaining_ticks < 0) remaining_ticks = 0;
        
        if (can_go_left) {
            path[path_length++] = 'L';
            
            if (remaining_ticks > 0) {
                forwardEncoderPID(BASE_SPEED, remaining_ticks);
            }
            stopMotors();
            turnLeft();

        } else if (can_go_straight) {
            path[path_length++] = 'S';
            if (remaining_ticks > 0) {
                forwardEncoderPID(BASE_SPEED, remaining_ticks);
            }

        } else if (can_go_right) {
            path[path_length++] = 'R';

            if (remaining_ticks > 0) {
                forwardEncoderPID(BASE_SPEED, remaining_ticks);
            }
            stopMotors();
            turnRight();

        } else {
            path[path_length++] = 'B';
            stopMotors();
            if (remaining_ticks > 0) forwardEncoderPID(BASE_SPEED, remaining_ticks);
            turnAround();

        }
    }
    stopMotors();
}



void simplifyPath() {
    for(int i=0; i<path_length; i++) {
        simplified_path[i] = path[i];
    }
    simplified_path_length = path_length;

    for (int i = 0; i <= simplified_path_length - 3; i++) {
        if (simplified_path[i+1] == 'B') {
            char turn1 = simplified_path[i];
            char turn2 = simplified_path[i+2];
            char new_turn = 'X';

            if (turn1 == 'L' && turn2 == 'R') new_turn = 'B';
            else if (turn1 == 'R' && turn2 == 'L') new_turn = 'B';
            else if (turn1 == 'S' && turn2 == 'S') new_turn = 'B';
            else if (turn1 == 'L' && turn2 == 'S') new_turn = 'R';
            else if (turn1 == 'R' && turn2 == 'S') new_turn = 'L';
            else if (turn1 == 'S' && turn2 == 'L') new_turn = 'R';
            else if (turn1 == 'S' && turn2 == 'R') new_turn = 'L';
            else if (turn1 == 'L' && turn2 == 'L') new_turn = 'S';
            else if (turn1 == 'R' && turn2 == 'R') new_turn = 'S';

            if (new_turn != 'X') {
                simplified_path[i] = new_turn;
                for (int j = i + 1; j < simplified_path_length - 2; j++) {
                    simplified_path[j] = simplified_path[j+2];
                }
                simplified_path_length -= 2;
                i = -1;
            }
        }
    }
}

void runFast() {
    current_kp = KP_FAST;
    current_kd = KD_FAST;

    for (int i = 0; i < simplified_path_length; i++) {
        
        followSegment(FAST_SPEED); 
        forwardEncoderPID(BASE_SPEED, FAST_TICKS_SENSORS_TO_AXLE);
        
        stopMotors(); 
        delay(50);
        
        char direction = simplified_path[i];

        if (direction == 'L') {
            fastturnLeft();
        } else if (direction == 'R') {
            fastturnRight();
        } else if (direction == 'S') {
        } else if (direction == 'B') {
            turnAround();
        }
    }
    followSegment(FAST_SPEED);
    stopMotors();
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(GOAL_LED_PIN, HIGH);
}

void printPath(char* p, int len) {
    for(int i=0; i<len; i++) Serial.print(p[i]);
    Serial.println();
}

volatile unsigned long last_left_time = 0;
volatile unsigned long last_right_time = 0;

#define DEBOUNCE_TIME 150 

void leftEncoderISR() {
    if (micros() - last_left_time > DEBOUNCE_TIME) {
        left_encoder_ticks++;
        last_left_time = micros();
    }
}

void rightEncoderISR() {
    if (micros() - last_right_time > DEBOUNCE_TIME) {
        right_encoder_ticks++;
        last_right_time = micros();
    }
}