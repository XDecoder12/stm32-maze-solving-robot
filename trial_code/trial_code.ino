#include "config.h"

// --- DEBUGGING ---
// Set to 1 to print debug info, 0 for final run
#define DEBUG 0

// --- Global Variables ---
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

// --- Function Prototypes ---
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

// --- Setup Function ---
void setup() {
    Serial.begin(115200);
    while(!Serial && millis() < 2000);

    pinMode(LED_PIN, OUTPUT);
    pinMode(GOAL_LED_PIN, OUTPUT); // This line reads PB11 from config.h
    pinMode(USER_BUTTON_PIN, INPUT_PULLDOWN);

    pinMode(MOTOR_A_IN1, OUTPUT);
    pinMode(MOTOR_A_IN2, OUTPUT);
    pinMode(MOTOR_A_PWM, PWM);
    pinMode(MOTOR_B_IN1, OUTPUT);
    pinMode(MOTOR_B_IN2, OUTPUT);
    pinMode(MOTOR_B_PWM, PWM);
    
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensor_min[i] = 0; // Assuming 12-bit ADC
        sensor_max[i] = 4095;
    }

    pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
    pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), leftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightEncoderISR, RISING);

    digitalWrite(LED_PIN, LOW);
    digitalWrite(GOAL_LED_PIN, LOW);
    Serial.println("\n--- Maze Solver Initialized ---");
    Serial.println("Press button on PA0 to start sensor calibration.");
}

// --- Main Loop ---
void loop() {
    switch (currentState) {
        case IDLE:
            if (digitalRead(USER_BUTTON_PIN) == HIGH) {
                waitForButtonPress();
                currentState = CALIBRATING_SENSORS;
            }
            break;

        case CALIBRATING_SENSORS:
            Serial.println("State: CALIBRATING SENSORS");
            digitalWrite(LED_PIN, HIGH);
            calibrateSensors();
            digitalWrite(LED_PIN, LOW);
            Serial.println("Sensor calibration finished.");
            Serial.println("Press button to start SOLVING the maze.");
            sensor_calibration_done = true;
            currentState = STOPPED;
            break;

        case SOLVING_MAZE:
            Serial.println("\nState: SOLVING_MAZE");
            solveMaze();
            simplifyPath();
            Serial.println("--- Maze Solved! ---");
            Serial.print("Original Path: ");
            printPath(path, path_length);
            Serial.print("Simplified Path: ");
            printPath(simplified_path, simplified_path_length);
            Serial.println("Press button to start the fast run.");
            currentState = STOPPED;
            break;

        case FAST_RUN:
            Serial.println("\nState: FAST_RUN");
            runFast();
            Serial.println("--- Fast run finished! ---");
            currentState = STOPPED; //FINISHED
            break;

        case STOPPED:
            stopMotors();
            
            if (goal_reached) {
                digitalWrite(LED_PIN, HIGH);
                digitalWrite(GOAL_LED_PIN, HIGH); // Turn on the GOAL LED (PB11)
            }

            if (digitalRead(USER_BUTTON_PIN) == HIGH) {
                waitForButtonPress();
                
                // Turn off LED before starting next run
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
            // --- LED ON (solid) after fast run ---
            digitalWrite(LED_PIN, HIGH);
            digitalWrite(GOAL_LED_PIN, HIGH);
            break;
    }
}


// --- Core Functions ---

void readSensorsAnalog() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensor_values[i] = analogRead(sensor_pins[i]);
    }
}

void calibrateSensors() {
    left_encoder_ticks = 0;
    right_encoder_ticks = 0;
    motorControl(TURN_SPEED, -TURN_SPEED); 
    //long start_ticks = (left_encoder_ticks + right_encoder_ticks) / 2;
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

    Serial.println("--- Calibration Values ---");
    for (int i = 0; i < NUM_SENSORS; i++) {
        if(sensor_max[i] > sensor_min[i]) {
            sensor_thresholds[i] = sensor_min[i] + (uint16_t)((sensor_max[i] - sensor_min[i]) * 0.45);
        } else {
            sensor_thresholds[i] = 2048; // Default if calibration failed
        }
        Serial.print("Sensor "); Serial.print(i);
        Serial.print(" | Min: "); Serial.print(sensor_min[i]);
        Serial.print(" | Max: "); Serial.print(sensor_max[i]);
        Serial.print(" | Threshold: "); Serial.println(sensor_thresholds[i]);
    }
}

void waitForButtonPress() {
    delay(200); // Prevent immediate re-trigger
    while(digitalRead(USER_BUTTON_PIN) == LOW); // Wait for press
    delay(50); // Debounce
    while(digitalRead(USER_BUTTON_PIN) == HIGH);  // Wait for release
    delay(50); // Debounce
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
        // return last_error; 
        return ( last_error > 0 ? 3.5 : -3.5 );
    }

    float weighted_sum = 0;
    float activation_sum = 0;
    
    for (int i = 0; i < NUM_SENSORS; i++) {
        float activation = 0;
        if (sensor_digital[i]) {
            if (sensor_thresholds[i] > sensor_min[i]) {
                 activation = (float)(sensor_thresholds[i] - sensor_values[i]) / (sensor_thresholds[i] - sensor_min[i]);
                 //if (activation > 1.0) activation = 1.0;
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

    // --- LOGIC FIX ---
    // This is the corrected turning direction.(FOR MY BOT SPECIFICALLY)
    // Positive error (line left) -> positive correction -> speed up left motor
    int left_speed = base_speed + motor_speed_correction;
    int right_speed = base_speed - motor_speed_correction;

    #if DEBUG
    static unsigned long lastPidPrint = 0;
    if (millis() - lastPidPrint > 250) {
      Serial.print("Sensors: [");
      for (int i = 0; i < NUM_SENSORS; i++) {
        Serial.print(sensor_digital[i]);
        if (i < NUM_SENSORS - 1) Serial.print(",");
      }
      Serial.print("] | Error: "); Serial.print(error);
      Serial.print(" | Correction: "); Serial.print(motor_speed_correction);
      Serial.print(" | Speeds L/R: "); Serial.print(left_speed); Serial.print("/"); Serial.println(right_speed);
      lastPidPrint = millis();
    }
    #endif

    motorControl(left_speed, right_speed);
}

void motorControl(int left_speed, int right_speed) {
    left_speed += MOTOR_TRIM;
    right_speed -= MOTOR_TRIM;

    left_speed = constrain(left_speed, -255, 255);
    right_speed = constrain(right_speed, -255, 255);
    
    // Left motor (MOTOR_B)
    if (left_speed >= 0) { // Forward
        digitalWrite(MOTOR_B_IN1, LOW); digitalWrite(MOTOR_B_IN2, HIGH);
    } else { // Reverse
        digitalWrite(MOTOR_B_IN1, HIGH); digitalWrite(MOTOR_B_IN2, LOW);
    }
    analogWrite(MOTOR_B_PWM, abs(left_speed));

    // Right motor (MOTOR_A)
    if (right_speed >= 0) { // Forward
        digitalWrite(MOTOR_A_IN1, LOW); digitalWrite(MOTOR_A_IN2, HIGH);
    } else { // Reverse
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


// void turnLeft() {
//     left_encoder_ticks = 0;
//     right_encoder_ticks = 0;
//     motorControl(TURN_SPEED, -TURN_SPEED);

//     long min_turn_ticks = TICKS_FOR_90_DEG_TURN * 0.95; //0.9 //0.65
//     while( (left_encoder_ticks + right_encoder_ticks) / 2 < min_turn_ticks);

//     while( (left_encoder_ticks + right_encoder_ticks) / 2 < TICKS_FOR_90_DEG_TURN) {
//         readSensorsDigital();
//         if (sensor_digital[3] || sensor_digital[4]) {
//             #if DEBUG
//             Serial.print("Line detected early in left turn at ");
//             Serial.print((left_encoder_ticks + right_encoder_ticks) / 2);
//             Serial.println(" ticks.");
//             #endif
//             break;
//         }
//     }
//     stopMotors();
// }
void turnLeft() {
    // 1. Reset ticks just for debugging (optional)
    left_encoder_ticks = 0;
    right_encoder_ticks = 0;

    // 2. Start Motors blindly
    // INCREASE this speed to ensure it overcomes friction
    motorControl(TURN_SPEED, -TURN_SPEED); 

    // 3. THE FIX: Forget encoders. Just wait.
    // Adjust this number (300) until you get a 90 degree turn.
    delay(300);
    while(true) {
        readSensorsDigital();
        
        // If the CENTER sensors (3 or 4) see the black line...
        if (sensor_digital[3] || sensor_digital[4]) {
            break; // ... STOP immediately. We found the path.
        }
    }
    // 4. Stop
    stopMotors();
    
    // 5. Short pause to let momentum settle
    delay(100);
}




// void fastturnLeft() {
//     left_encoder_ticks = 0;
//     right_encoder_ticks = 0;
//     motorControl(FAST_TURN_SPEED, -FAST_TURN_SPEED);

//     long min_turn_ticks = TICKS_FOR_90_DEG_TURN * 0.9;
//     while( (left_encoder_ticks + right_encoder_ticks) / 2 < min_turn_ticks);

//     while( (left_encoder_ticks + right_encoder_ticks) / 2 < TICKS_FOR_90_DEG_TURN) {
//         readSensorsDigital();
//         if (sensor_digital[3] || sensor_digital[4]) {
//             #if DEBUG
//             Serial.print("Line detected early in left turn at ");
//             Serial.print((left_encoder_ticks + right_encoder_ticks) / 2);
//             Serial.println(" ticks.");
//             #endif
//             break;
//         }
//     }
//     stopMotors();
// }
// void fastturnLeft() {
//     // 1. Reset ticks just for debugging (optional)
//     left_encoder_ticks = 0;
//     right_encoder_ticks = 0;

//     // 2. Start Motors blindly
//     // INCREASE this speed to ensure it overcomes friction
//     motorControl(FAST_TURN_SPEED, -FAST_TURN_SPEED); 

//     // 3. THE FIX: Forget encoders. Just wait.
//     // Adjust this number (300) until you get a 90 degree turn.
//     delay(300);
//     while(true) {
//         readSensorsDigital();
        
//         // If the CENTER sensors (3 or 4) see the black line...
//         if (sensor_digital[3] || sensor_digital[4]) {
//             break; // ... STOP immediately. We found the path.
//         }
//     }
//     // 4. Stop
//     stopMotors();
    
//     // 5. Short pause to let momentum settle
//     delay(100);
// }
void fastturnLeft() {
    left_encoder_ticks = 0;
    right_encoder_ticks = 0;

    // Start Turn
    motorControl(FAST_TURN_SPEED, -FAST_TURN_SPEED); 

    // 1. BLIND CLEARANCE: Spin blindly to get OFF the current line
    delay(150); 

    // 2. WAIT FOR WHITE (Safety Check)
    // Ensure we are in the middle of the turn (on the white floor)
    // before we start looking for the new line.
    unsigned long safety_timer = millis();
    while(true) {
        readSensorsDigital();
        // If center is WHITE, we are safe to proceed to step 3
        if (!sensor_digital[3] && !sensor_digital[4]) break;
        
        // Timeout to prevent infinite loop
        if(millis() - safety_timer > 500) break; 
    }

    // 3. WAIT FOR BLACK (Target Lock)
    while(true) {
        readSensorsDigital();
        // Now if we see black, it is definitely the NEW line
        if (sensor_digital[3] || sensor_digital[4]) {
            stopMotors(); 
            break; 
        }
    }
    delay(50);
}




// void turnRight() {
//     left_encoder_ticks = 0;
//     right_encoder_ticks = 0;
//     motorControl(-TURN_SPEED, TURN_SPEED);

//     long min_turn_ticks = TICKS_FOR_90_DEG_TURN * 0.95; //0.9 //0.65
//     while( (left_encoder_ticks + right_encoder_ticks) / 2 < min_turn_ticks);

//     while( (left_encoder_ticks + right_encoder_ticks) / 2 < TICKS_FOR_90_DEG_TURN) {
//         readSensorsDigital();
//         if (sensor_digital[3] || sensor_digital[4]) {
//             #if DEBUG
//             Serial.print("Line detected early in right turn at ");
//             Serial.print((left_encoder_ticks + right_encoder_ticks) / 2);
//             Serial.println(" ticks.");
//             #endif
//             break;
//         }
//     }
//     stopMotors();
// }
void turnRight() {
    // 1. Reset ticks just for debugging (optional)
    left_encoder_ticks = 0;
    right_encoder_ticks = 0;

    // 2. Start Motors blindly
    // INCREASE this speed to ensure it overcomes friction
    motorControl(-TURN_SPEED, TURN_SPEED); 

    // 3. THE FIX: Forget encoders. Just wait.
    // Adjust this number (300) until you get a 90 degree turn.
    delay(300);
    while(true) {
        readSensorsDigital();
        
        // If the CENTER sensors (3 or 4) see the black line...
        if (sensor_digital[3] || sensor_digital[4]) {
            break; // ... STOP immediately. We found the path.
        }
    }
    // 4. Stop
    stopMotors();
    
    // 5. Short pause to let momentum settle
    delay(100);
}




// void fastturnRight() {
//     left_encoder_ticks = 0;
//     right_encoder_ticks = 0;
//     motorControl(-FAST_TURN_SPEED, FAST_TURN_SPEED);

//     long min_turn_ticks = TICKS_FOR_90_DEG_TURN * 0.9;
//     while( (left_encoder_ticks + right_encoder_ticks) / 2 < min_turn_ticks);

//     while( (left_encoder_ticks + right_encoder_ticks) / 2 < TICKS_FOR_90_DEG_TURN) {
//         readSensorsDigital();
//         if (sensor_digital[3] || sensor_digital[4]) {
//             #if DEBUG
//             Serial.print("Line detected early in right turn at ");
//             Serial.print((left_encoder_ticks + right_encoder_ticks) / 2);
//             Serial.println(" ticks.");
//             #endif
//             break;
//         }
//     }
//     stopMotors();
// }
// void fastturnRight() {
//     // 1. Reset ticks just for debugging (optional)
//     left_encoder_ticks = 0;
//     right_encoder_ticks = 0;

//     // 2. Start Motors blindly
//     // INCREASE this speed to ensure it overcomes friction
//     motorControl(-FAST_TURN_SPEED, FAST_TURN_SPEED); 

//     // 3. THE FIX: Forget encoders. Just wait.
//     // Adjust this number (300) until you get a 90 degree turn.
//     delay(300);
//     while(true) {
//         readSensorsDigital();
        
//         // If the CENTER sensors (3 or 4) see the black line...
//         if (sensor_digital[3] || sensor_digital[4]) {
//             break; // ... STOP immediately. We found the path.
//         }
//     }
//     // 4. Stop
//     stopMotors();
    
//     // 5. Short pause to let momentum settle
//     delay(100);
// }
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

    long min_turn_ticks = TICKS_FOR_180_DEG_TURN * 0.8; //0.4 //0.9 //0.6
    while( (left_encoder_ticks + right_encoder_ticks) / 2 < min_turn_ticks);
    
    while( (left_encoder_ticks + right_encoder_ticks) / 2 < TICKS_FOR_180_DEG_TURN) {
        readSensorsDigital();
        if (sensor_digital[3] || sensor_digital[4]) {
            #if DEBUG
            Serial.print("Line detected early in 180 turn at ");
            Serial.print((left_encoder_ticks + right_encoder_ticks) / 2);
            Serial.println(" ticks.");
            #endif
            break;
        }
    }
    stopMotors();
}
// void turnAround() {
//     left_encoder_ticks = 0;
//     right_encoder_ticks = 0;

//     motorControl(-TURN_SPEED, TURN_SPEED); 

//     delay(450); 
//     while(true) {
//         readSensorsDigital();
        
//         if (sensor_digital[3] || sensor_digital[4]) {
//             break; // ... STOP. Turn complete.
//         }
//     }

//     stopMotors();
//     delay(100);
// }





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



void activeBrake() {
    // Reverse motors at full power for a tiny burst to kill momentum
    digitalWrite(MOTOR_A_IN1, HIGH); digitalWrite(MOTOR_A_IN2, LOW); 
    digitalWrite(MOTOR_B_IN1, HIGH); digitalWrite(MOTOR_B_IN2, LOW); 
    analogWrite(MOTOR_A_PWM, 255);
    analogWrite(MOTOR_B_PWM, 255);
    delay(30); // 30ms is usually perfect. Adjust if it jerks back too much.
    stopMotors();
}



void solveMaze() {

    current_kp = KP_TRIAL;
    current_kd = KD_TRIAL;

    while (!goal_reached) {
        followSegment(BASE_SPEED);

        //these 2 lines literally fucked my life
        stopMotors();
        delay(25);  //50      


        readSensorsDigital();
        bool can_go_left = sensor_digital[0] || sensor_digital[1];
        bool can_go_right = sensor_digital[6] || sensor_digital[7];

        //long peek_ticks = TICKS_SENSORS_TO_AXLE; //3 //2 x
        long peek_ticks = 60; 
        forwardEncoderPID(BASE_SPEED, peek_ticks);
        stopMotors();
        delay(50); //150 x

        readSensorsDigital();
        int new_sensor_count = 0;
        for(int i = 0; i < NUM_SENSORS; i++) {
            if(sensor_digital[i]) new_sensor_count++;
        }

        if (new_sensor_count >= 6) {  //7
            #if DEBUG
            Serial.println("Goal confirmed!");
            #endif
            goal_reached = true;
            continue; 
        }
        
        bool can_go_straight = sensor_digital[3] || sensor_digital[4];
        
        long remaining_ticks = TICKS_SENSORS_TO_AXLE - peek_ticks;
        //new
        if (remaining_ticks < 0) remaining_ticks = 0;
        
        if (can_go_left) {
            #if DEBUG
            Serial.println("Decision: Go Left");
            #endif
            path[path_length++] = 'L';
            
            if (remaining_ticks > 0) {
                forwardEncoderPID(BASE_SPEED, remaining_ticks);
            }
            stopMotors();
            turnLeft();

        } else if (can_go_straight) {
            #if DEBUG
            Serial.println("Decision: Go Straight");
            #endif
            path[path_length++] = 'S';
            if (remaining_ticks > 0) {
                forwardEncoderPID(BASE_SPEED, remaining_ticks);
            }

        } else if (can_go_right) {
            #if DEBUG
            Serial.println("Decision: Go Right");
            #endif
            path[path_length++] = 'R';

            if (remaining_ticks > 0) {
                forwardEncoderPID(BASE_SPEED, remaining_ticks);
            }
            stopMotors();
            turnRight();

        } else {
            #if DEBUG
            Serial.println("Decision: Dead End, Turn Around");
            #endif
            path[path_length++] = 'B';

            //new
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


// void runFast() {

//     current_kp = KP_FAST;
//     current_kd = KD_FAST;

//     for (int i = 0; i < simplified_path_length; i++) {
//         followSegment(FAST_SPEED); 

//         // forwardEncoderPID(FAST_SPEED, TICKS_SENSORS_TO_AXLE);
//         forwardEncoderPID(FAST_SPEED, FAST_TICKS_SENSORS_TO_AXLE);
//         stopMotors(); 
//         //delay(50);

//         char direction = simplified_path[i];


//         long peek_ticks = TICKS_SENSORS_TO_AXLE / 2; //3


//         if (direction == 'L') {
//             stopMotors();

//             forwardEncoderPID(BASE_SPEED, peek_ticks);

//             fastturnLeft();
//         } else if (direction == 'R') {
//             stopMotors();

//             forwardEncoderPID(BASE_SPEED, peek_ticks);

//             fastturnRight();
//         } else if (direction == 'S') {
//             // No turn, just keep going
//             //forwardEncoderPID(FAST_SPEED, FAST_TICKS_SENSORS_TO_AXLE);
//         } else if (direction == 'B') {
//             stopMotors();

//             forwardEncoderPID(BASE_SPEED, peek_ticks);

//             turnAround();
//         }
//     }
//     followSegment(FAST_SPEED);
//     //forwardEncoderPID(FAST_SPEED, FAST_TICKS_SENSORS_TO_AXLE);
//     stopMotors();
// }
void runFast() {
    current_kp = KP_FAST;
    current_kd = KD_FAST;

    for (int i = 0; i < simplified_path_length; i++) {
        
        // 1. ZOOM: Run fast on the straight line
        followSegment(FAST_SPEED); 

        // 2. SLOW DOWN: Don't stop, just drop speed for alignment
        // We switch to BASE_SPEED (135) to regain traction before the turn.
        // We move forward to align the wheels with the turn point.
        forwardEncoderPID(BASE_SPEED, FAST_TICKS_SENSORS_TO_AXLE);
        
        // 3. NOW we stop to turn
        stopMotors(); 
        delay(50);
        
        char direction = simplified_path[i];

        if (direction == 'L') {
            fastturnLeft();
        } else if (direction == 'R') {
            fastturnRight();
        } else if (direction == 'S') {
            // Do nothing, just accelerate into the next segment
        } else if (direction == 'B') {
            turnAround();
        }
    }
    // Finish the final straight
    followSegment(FAST_SPEED);
    stopMotors();
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(GOAL_LED_PIN, HIGH);
}




void printPath(char* p, int len) {
    for(int i=0; i<len; i++) Serial.print(p[i]);
    Serial.println();
}

// --- Interrupt Service Routines ---
// void leftEncoderISR() {
//     left_encoder_ticks++;
// }

// void rightEncoderISR() {
//     right_encoder_ticks++;
// }

// --- Interrupt Service Routines (Modified with Debounce) ---

// 1. We create variables to remember the LAST time the encoder ticked
volatile unsigned long last_left_time = 0;
volatile unsigned long last_right_time = 0;

// 2. This is the "ignore" window in microseconds. 
// If a signal comes faster than this, we assume it's noise.
#define DEBOUNCE_TIME 150 

void leftEncoderISR() {
    // Only count the tick if enough time has passed since the last one
    if (micros() - last_left_time > DEBOUNCE_TIME) {
        left_encoder_ticks++;
        last_left_time = micros();
    }
}

void rightEncoderISR() {
    // Only count the tick if enough time has passed since the last one
    if (micros() - last_right_time > DEBOUNCE_TIME) {
        right_encoder_ticks++;
        last_right_time = micros();
    }
}