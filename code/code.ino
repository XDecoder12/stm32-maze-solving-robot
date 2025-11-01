#include "config.h"

// --- DEBUGGING ---
// Set to 1 to enable detailed Serial output. 
// IMPORTANT: Set to 0 for the final run to maximize speed!
#define DEBUG 0

// --- Global Variables ---

// Sensor variables
uint16_t sensor_pins[NUM_SENSORS] = {SENSOR_8_PIN, SENSOR_7_PIN, SENSOR_6_PIN, SENSOR_5_PIN, SENSOR_4_PIN, SENSOR_3_PIN, SENSOR_2_PIN, SENSOR_1_PIN};
uint16_t sensor_values[NUM_SENSORS];
uint16_t sensor_min[NUM_SENSORS];
uint16_t sensor_max[NUM_SENSORS];
uint16_t sensor_thresholds[NUM_SENSORS];
bool sensor_digital[NUM_SENSORS];

// Encoder variables (volatile for use in ISRs)
volatile long left_encoder_ticks = 0;
volatile long right_encoder_ticks = 0;

// PID variables
float error = 0, last_error = 0;
float integral = 0, derivative = 0;
int motor_speed_correction;

// Maze solving variables
char path[MAX_PATH_LENGTH];
int path_length = 0;
bool goal_reached = false;
char simplified_path[MAX_PATH_LENGTH];
int simplified_path_length = 0;

// Robot state machine
enum RobotState { IDLE, CALIBRATING_SENSORS, SOLVING_MAZE, FAST_RUN, STOPPED, FINISHED };
RobotState currentState = IDLE;

// Flags to track calibration progress
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
void forwardEncoder(int speed, long ticks);
void turnLeft();
void turnRight();
void turnAround();
void reacquireLine();
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
    // Wait for serial connection to open (for debugging)
    while(!Serial && millis() < 2000);

    pinMode(LED_PIN, OUTPUT);
    pinMode(USER_BUTTON_PIN, INPUT_PULLDOWN);

    // Motor control pins
    pinMode(MOTOR_A_IN1, OUTPUT);
    pinMode(MOTOR_A_IN2, OUTPUT);
    pinMode(MOTOR_A_PWM, PWM);
    pinMode(MOTOR_B_IN1, OUTPUT);
    pinMode(MOTOR_B_IN2, OUTPUT);
    pinMode(MOTOR_B_PWM, PWM);
    
    // Initialize sensor min/max arrays for calibration
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensor_min[i] = 4095; // Assuming 12-bit ADC on STM32
        sensor_max[i] = 0;
    }

    // Encoder pins with interrupts
    pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
    pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), leftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightEncoderISR, RISING);

    digitalWrite(LED_PIN, LOW);
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
            digitalWrite(LED_PIN, HIGH); // Turn on LED during calibration
            calibrateSensors();
            digitalWrite(LED_PIN, LOW);
            Serial.println("Sensor calibration finished.");
            Serial.println("Press button to start SOLVING the maze.");
            sensor_calibration_done = true;
            currentState = STOPPED; // Wait for next command
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
            currentState = FINISHED;
            break;

        case STOPPED:
            stopMotors();
            if (digitalRead(USER_BUTTON_PIN) == HIGH) {
                waitForButtonPress();
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
            // Blink LED to signify completion
            digitalWrite(LED_PIN, HIGH);
            delay(250);
            digitalWrite(LED_PIN, LOW);
            delay(250);
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
    motorControl(TURN_SPEED, -TURN_SPEED); 
    for (int i = 0; i < 2; i++) {
        for(long t = millis(); millis() - t < 2000; ) { // 2 second rotation
            readSensorsAnalog();
            for (int k = 0; k < NUM_SENSORS; k++) {
                if (sensor_values[k] < sensor_min[k]) sensor_min[k] = sensor_values[k];
                if (sensor_values[k] > sensor_max[k]) sensor_max[k] = sensor_values[k];
            }
        }
    }
    stopMotors();

    Serial.println("--- Calibration Values ---");
    for (int i = 0; i < NUM_SENSORS; i++) {
        if(sensor_max[i] > sensor_min[i]) {
            sensor_thresholds[i] = sensor_min[i] + (uint16_t)((sensor_max[i] - sensor_min[i]) * 0.4);
        } else {
            sensor_thresholds[i] = 2048; // Default if calibration failed for a sensor
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
    // First, read analog values and update the digital array for other functions
    readSensorsAnalog();
    int on_line_count = 0;
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensor_digital[i] = (sensor_values[i] < sensor_thresholds[i]);
        if(sensor_digital[i]) {
            on_line_count++;
        }
    }

    // Handle case where the line is completely lost
    if (on_line_count == 0) {
        return last_error; 
    }

    // --- NEW: Analog-based weighted average for smoother error ---
    float weighted_sum = 0;
    float activation_sum = 0;
    
    for (int i = 0; i < NUM_SENSORS; i++) {
        // Calculate a 'certainty' or 'activation' level from 0 (at threshold) to 1 (at min value)
        // This gives more weight to sensors that are clearly over the line.
        float activation = 0;
        if (sensor_digital[i]) {
            // Ensure we don't divide by zero if min and threshold are the same
            if (sensor_thresholds[i] > sensor_min[i]) {
                 activation = (float)(sensor_thresholds[i] - sensor_values[i]) / (sensor_thresholds[i] - sensor_min[i]);
            } else {
                 activation = 1.0; // Max activation if range is zero
            }
        }
        
        // Use sensor index (0-7) as the weight
        weighted_sum += activation * i;
        activation_sum += activation;
    }
    
    // If activation_sum is somehow zero (e.g., floating point errors), prevent division by zero
    if (activation_sum == 0) {
        return last_error;
    }

    float position = weighted_sum / activation_sum;
    
    // The ideal center is between sensor 3 and 4, which is position 3.5
    // A position > 3.5 means the line is to the right, requiring a negative error to turn left.
    // A position < 3.5 means the line is to the left, requiring a positive error to turn right.
    return 3.5 - position;
}

// --- MODIFIED FUNCTION ---
void pidControl(int base_speed) {
    error = calculateError();
    
    if (KI != 0.0) {
      integral += error;
    }
    
    derivative = error - last_error;
    last_error = error;

    motor_speed_correction = (KP * error) + (KI * integral) + (KD * derivative);

    // Swap the + and - to reverse the turning direction
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

// --- REVERTED TO ORIGINAL ---
void motorControl(int left_speed, int right_speed) {
    left_speed += MOTOR_TRIM;
    right_speed -= MOTOR_TRIM;

    left_speed = constrain(left_speed, -255, 255);
    right_speed = constrain(right_speed, -255, 255);
    
    // Left motor logic (using MOTOR_B pins)
    if (left_speed >= 0) { // Positive speed
        digitalWrite(MOTOR_B_IN1, LOW); digitalWrite(MOTOR_B_IN2, HIGH);
    } else { // Negative speed
        digitalWrite(MOTOR_B_IN1, HIGH); digitalWrite(MOTOR_B_IN2, LOW);
    }
    analogWrite(MOTOR_B_PWM, abs(left_speed));

    // Right motor logic (using MOTOR_A pins)
    if (right_speed >= 0) { // Positive speed
        digitalWrite(MOTOR_A_IN1, LOW); digitalWrite(MOTOR_A_IN2, HIGH);
    } else { // Negative speed
        digitalWrite(MOTOR_A_IN1, HIGH); digitalWrite(MOTOR_A_IN2, LOW);
    }
    analogWrite(MOTOR_A_PWM, abs(right_speed));
}


void stopMotors() {
    motorControl(0, 0);
}

void forwardEncoder(int speed, long ticks) {
    left_encoder_ticks = 0;
    right_encoder_ticks = 0;
    motorControl(speed, speed);
    while( (left_encoder_ticks + right_encoder_ticks) / 2 < ticks);
}

void turnLeft() {
    left_encoder_ticks = 0;
    right_encoder_ticks = 0;
    // Command a RIGHT turn to physically go LEFT
    motorControl(TURN_SPEED, -TURN_SPEED);

    // Turn at least 70% of the way before looking for the line
    long min_turn_ticks = TICKS_FOR_90_DEG_TURN * 0.7;
    while(right_encoder_ticks < min_turn_ticks);

    // Now, continue until the full ticks are reached OR the line is found
    while(right_encoder_ticks < TICKS_FOR_90_DEG_TURN) {
        readSensorsDigital();
        // If the center sensors see the line, stop early
        if (sensor_digital[3] || sensor_digital[4]) {
            #if DEBUG
            Serial.print("Line detected early in left turn at ");
            Serial.print(right_encoder_ticks);
            Serial.println(" ticks.");
            #endif
            break; // Exit the loop
        }
    }
    stopMotors();
}

void turnRight() {
    left_encoder_ticks = 0;
    right_encoder_ticks = 0;
    // Command a LEFT turn to physically go RIGHT
    motorControl(-TURN_SPEED, TURN_SPEED);

    // Turn at least 70% of the way before looking for the line
    long min_turn_ticks = TICKS_FOR_90_DEG_TURN * 0.7;
    while(left_encoder_ticks < min_turn_ticks);

    // Now, continue until the full ticks are reached OR the line is found
    while(left_encoder_ticks < TICKS_FOR_90_DEG_TURN) {
        readSensorsDigital();
        // If the center sensors see the line, stop early
        if (sensor_digital[3] || sensor_digital[4]) {
            #if DEBUG
            Serial.print("Line detected early in right turn at ");
            Serial.print(left_encoder_ticks);
            Serial.println(" ticks.");
            #endif
            break; // Exit the loop
        }
    }
    stopMotors();
}

void turnAround() {
    left_encoder_ticks = 0;
    right_encoder_ticks = 0;
    // This should also be swapped to match the new turnRight logic
    motorControl(-TURN_SPEED, TURN_SPEED);

    // Turn past 90 degrees before looking for the line
    long min_turn_ticks = TICKS_FOR_180_DEG_TURN * 0.6; 
    while(left_encoder_ticks < min_turn_ticks);
    
    // Now, continue until the full ticks are reached OR the line is found
    while(left_encoder_ticks < TICKS_FOR_180_DEG_TURN) {
        readSensorsDigital();
        // If the center sensors see the line, stop early
        if (sensor_digital[3] || sensor_digital[4]) {
            #if DEBUG
            Serial.print("Line detected early in 180 turn at ");
            Serial.print(left_encoder_ticks);
            Serial.println(" ticks.");
            #endif
            break; // Exit the loop
        }
    }
    stopMotors();
}

void reacquireLine() {
    #if DEBUG
    Serial.println("Re-acquiring line after turn...");
    #endif
    motorControl(-TURN_SPEED / 2, TURN_SPEED / 2);
    long startTime = millis();
    while (millis() - startTime < 150) { // Sweep for 150ms
        readSensorsDigital();
        if (sensor_digital[3] || sensor_digital[4]) {
            stopMotors();
            #if DEBUG
            Serial.println("Found line while sweeping.");
            #endif
            return;
        }
    }
    motorControl(TURN_SPEED / 2, -TURN_SPEED / 2);
    startTime = millis();
    while (millis() - startTime < 300) { // Sweep other way for 300ms
        readSensorsDigital();
        if (sensor_digital[3] || sensor_digital[4]) {
            stopMotors();
            #if DEBUG
            Serial.println("Found line while sweeping.");
            #endif
            return;
        }
    }
    stopMotors();
    #if DEBUG
    Serial.println("Warning: Did not re-acquire line.");
    #endif
}

// Replace your old followSegment function with this one
void followSegment(int speed) {
    while (true) {
        pidControl(speed);
        
        int sensor_count = 0;
        for (int i = 0; i < NUM_SENSORS; i++) {
            if (sensor_digital[i]) sensor_count++;
        }
        
        // When we see a complex feature (intersection) or lose the line (dead end),
        // return and let the solveMaze function figure out what it is.
        if (sensor_count >= 5 || sensor_count == 0) { 
            return;
        }
    }
}

// Replace your old solveMaze function with this new one
void solveMaze() {
    while (!goal_reached) {
        followSegment(BASE_SPEED);

        // --- NEW GOAL DETECTION LOGIC IS INTEGRATED BELOW ---

        // Step 1: At the edge of the intersection, check for side paths.
        readSensorsDigital();
        bool can_go_left = sensor_digital[0] || sensor_digital[1];
        bool can_go_right = sensor_digital[NUM_SENSORS - 2] || sensor_digital[NUM_SENSORS - 1];

        // Step 2: Move forward into the potential goal area.
        forwardEncoder(BASE_SPEED, 130); 
        stopMotors();
        delay(50); // Short delay for stability

        // Step 3: Read sensors again from the new, forward position.
        readSensorsDigital();
        int new_sensor_count = 0;
        for(int i = 0; i < NUM_SENSORS; i++) {
            if(sensor_digital[i]) new_sensor_count++;
        }

        // Step 4: **GOAL CHECK!** If it's still a wide line, we have reached the goal.
        if (new_sensor_count >= 7) { // Using 7 out of 8 for robustness
            #if DEBUG
            Serial.println("Goal confirmed!");
            #endif
            goal_reached = true;
            continue; // End this loop iteration; the while(!goal_reached) will terminate.
        }
        
        // Step 5: If it's not the goal, proceed with normal intersection logic.
        bool can_go_straight = sensor_digital[3] || sensor_digital[4];
        
        if (can_go_left) {
            #if DEBUG
            Serial.println("Decision: Go Left");
            #endif
            path[path_length++] = 'L';
            
            long remaining_ticks = TICKS_SENSORS_TO_AXLE - 130;
            if (remaining_ticks > 0) {
                forwardEncoder(BASE_SPEED, remaining_ticks);
            }
            stopMotors();
            turnLeft();
            reacquireLine();

        } else if (can_go_straight) {
            #if DEBUG
            Serial.println("Decision: Go Straight");
            #endif
            path[path_length++] = 'S';

        } else if (can_go_right) {
            #if DEBUG
            Serial.println("Decision: Go Right");
            #endif
            path[path_length++] = 'R';

            long remaining_ticks = TICKS_SENSORS_TO_AXLE - 130;
            if (remaining_ticks > 0) {
                forwardEncoder(BASE_SPEED, remaining_ticks);
            }
            stopMotors();
            turnRight();
            reacquireLine();

        } else {
            #if DEBUG
            Serial.println("Decision: Dead End, Turn Around");
            #endif
            path[path_length++] = 'B';
            turnAround();
            reacquireLine();
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
    for (int i = 0; i < simplified_path_length; i++) {
        followSegment(FAST_SPEED); 
        
        // Move forward to center the robot on the intersection before executing the known turn.
        forwardEncoder(FAST_SPEED, TICKS_SENSORS_TO_AXLE);
        
        char direction = simplified_path[i];
        if (direction == 'L') {
            turnLeft();
            reacquireLine();
        } else if (direction == 'R') {
            turnRight();
            reacquireLine();
        } else if (direction == 'S') {
            // No turn, just keep going
        } else if (direction == 'B') {
            turnAround();
            reacquireLine();
        }
    }
    
    followSegment(FAST_SPEED);
    stopMotors();
}

void printPath(char* p, int len) {
    for(int i=0; i<len; i++) Serial.print(p[i]);
    Serial.println();
}

// --- Interrupt Service Routines ---

void leftEncoderISR() {
    left_encoder_ticks++;
}

void rightEncoderISR() {
    right_encoder_ticks++;
}