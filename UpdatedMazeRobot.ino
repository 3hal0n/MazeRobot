#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>

// Display Configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Direction Constants
#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

// Motor Control Pins
#define PWMA 12
#define PWMB 13
#define AIN1 4
#define AIN2 16
#define BIN1 15
#define BIN2 2

// IMU Configuration
#define IMU_DEVICE_ADDR 0x69
#define IMU_PWR_CTRL 0x6B
#define IMU_ACCEL_X_H 0x3B
#define IMU_GYRO_X_H 0x43
#define IMU_ID_REG 0x75
#define IMU_ACCEL_SENS 16384.0
#define IMU_GYRO_SENS 131.0

// Encoder Configuration
const int encoderPin1A = 5, encoderPin1B = 18;
const int encoderPin2A = 19, encoderPin2B = 23;

// IR Sensor Configuration
const int emitter_FR = 32, receiver_FR = 26;
const int emitter_FL = 33, receiver_FL = 27;
const int receiver_R = 25, receiver_L = 14;

// Competition Constants
#define MAZE_SIZE 16
#define CELL_SIZE 180.0f  // 18cm in mm
#define WALL_THICKNESS 12.0f  // 1.2cm in mm
#define MAX_SPEED 200
#define EXPLORATION_SPEED 120
#define FAST_RUN_SPEED 180

// Enhanced Data Structures
struct Cell {
    uint8_t walls;      // Bit flags: North=1, East=2, South=4, West=8
    uint16_t distance;  // Flood fill distance
    bool visited;       // Exploration tracking
    uint8_t confidence; // Wall detection confidence
};

struct Position {
    int8_t x, y;
    uint8_t heading;
};

struct SensorData {
    float front_left, front_right, left, right;
    float gyro_z;
    bool wall_left, wall_right, wall_front;
    uint32_t timestamp;
};

struct PIDController {
    float kp, ki, kd;
    float error_sum, last_error;
    float output_min, output_max;
    uint32_t last_time;

    void reset() {
        error_sum = 0;
        last_error = 0;
        last_time = millis();
    }

    float compute(float setpoint, float input) {
        uint32_t now = millis();
        float dt = (now - last_time) / 1000.0f;
        if (dt <= 0) dt = 0.001f;

        float error = setpoint - input;
        error_sum += error * dt;

        // Anti-windup
        if (ki != 0) {
            if (error_sum > output_max/ki) error_sum = output_max/ki;
            if (error_sum < output_min/ki) error_sum = output_min/ki;
        }
        
        float d_error = (error - last_error) / dt;
        float output = kp * error + ki * error_sum + kd * d_error;

        output = constrain(output, output_min, output_max);

        last_error = error;
        last_time = now;

        return output;
    }
};

// Global Variables
Cell maze[MAZE_SIZE][MAZE_SIZE];
Position robot_pos = {0, 0, NORTH};
Position goal_positions[] = {{7,7}, {7,8}, {8,7}, {8,8}}; // Center 2x2 goal
SensorData sensors;

// PID Controllers
PIDController wall_follow_pid = {15.0f, 0.1f, 8.0f, -50.0f, 50.0f};
PIDController straight_pid = {25.0f, 0.2f, 12.0f, -40.0f, 40.0f};
PIDController rotation_pid = {8.0f, 0.1f, 2.0f, -100.0f, 100.0f};

// Motion Control Variables
volatile long encoder_left = 0, encoder_right = 0;
float gyro_bias = 0.0f;
float current_angle = 0.0f;
bool exploration_mode = true;
uint32_t run_start_time = 0;

// Performance Tracking
struct RunStats {
    uint32_t best_time;
    uint16_t cells_explored;
    uint8_t successful_runs;
    uint8_t total_runs;
} stats = {0};

// Function Declarations
void initializeSystems();
void calibrateSensors();
void readSensors();
void updateMaze();
void floodFill();
uint8_t getOptimalDirection();
void moveForward();
void turnLeft();
void turnRight();
void turnAround();
void executeTurn(float target_angle);
void setMotorSpeeds(int left_speed, int right_speed);
void stopMotors();
bool isGoalReached();
void optimizeForSpeedRun();
void handleEmergency();
void displayStatus();

// Interrupt Service Routines
void IRAM_ATTR encoderLeftISR() {
    static uint8_t lastState = 0;
    uint8_t state = (digitalRead(encoderPin1A) << 1) | digitalRead(encoderPin1B);
    uint8_t transition = (lastState << 2) | state;

    switch (transition) {
        case 0b0001: case 0b0111: case 0b1000: case 0b1110:
            encoder_left++; break;
        case 0b0010: case 0b0100: case 0b1011: case 0b1101:
            encoder_left--; break;
    }
    lastState = state;
}

void IRAM_ATTR encoderRightISR() {
    static uint8_t lastState = 0;
    uint8_t state = (digitalRead(encoderPin2A) << 1) | digitalRead(encoderPin2B);
    uint8_t transition = (lastState << 2) | state;

    switch (transition) {
        case 0b0001: case 0b0111: case 0b1000: case 0b1110:
            encoder_right++; break;
        case 0b0010: case 0b0100: case 0b1011: case 0b1101:
            encoder_right--; break;
    }
    lastState = state;
}

void setup() {
    Serial.begin(115200);
    Serial.println("Advanced Micromouse v2.0 Starting...");

    initializeSystems();
    calibrateSensors();

    // Load previous run data from EEPROM
    EEPROM.get(0, stats);

    Serial.println("System Ready for Competition");
    run_start_time = millis();
}

void initializeSystems() {
    // Motor pins
    pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
    pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);

    // Sensor pins
    pinMode(receiver_R, INPUT); pinMode(receiver_FR, INPUT);
    pinMode(receiver_FL, INPUT); pinMode(receiver_L, INPUT);
    pinMode(emitter_FL, OUTPUT); pinMode(emitter_FR, OUTPUT);

    // Encoder pins with interrupts
    pinMode(encoderPin1A, INPUT_PULLUP); pinMode(encoderPin1B, INPUT_PULLUP);
    pinMode(encoderPin2A, INPUT_PULLUP); pinMode(encoderPin2B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(encoderPin1A), encoderLeftISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderPin1B), encoderLeftISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderPin2A), encoderRightISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderPin2B), encoderRightISR, CHANGE);

    // I2C and Display
    Wire.begin(21, 22);
    Wire.setClock(400000); // Fast I2C

    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println("OLED init failed");
        while (1);
    }

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Micromouse v2.0");
    display.println("Initializing...");
    display.display();

    // Initialize IMU
    Wire.beginTransmission(IMU_DEVICE_ADDR);
    Wire.write(IMU_ID_REG);
    Wire.endTransmission(false);
    Wire.requestFrom(IMU_DEVICE_ADDR, 1);
    if (Wire.read() != 0x12) {
        Serial.println("IMU not detected!");
        while (1);
    }

    Wire.beginTransmission(IMU_DEVICE_ADDR);
    Wire.write(IMU_PWR_CTRL);
    Wire.write(0x00);
    Wire.endTransmission();

    // Initialize maze with unknown walls
    for (int x = 0; x < MAZE_SIZE; x++) {
        for (int y = 0; y < MAZE_SIZE; y++) {
            maze[x][y].walls = 0;
            maze[x][y].distance = 255;
            maze[x][y].visited = false;
            maze[x][y].confidence = 0;
        }
    }

    // Set boundary walls
    for (int i = 0; i < MAZE_SIZE; i++) {
        maze[i][0].walls |= 4;      // South wall
        maze[i][MAZE_SIZE-1].walls |= 1;  // North wall
        maze[0][i].walls |= 8;      // West wall
        maze[MAZE_SIZE-1][i].walls |= 2;  // East wall
    }

    stopMotors();
}

void calibrateSensors() {
    Serial.println("Calibrating sensors...");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Calibrating...");
    display.display();

    // Calibrate gyroscope bias
    float gyro_sum = 0;
    const int samples = 1000;

    for (int i = 0; i < samples; i++) {
        Wire.beginTransmission(IMU_DEVICE_ADDR);
        Wire.write(IMU_GYRO_X_H + 4);
        Wire.endTransmission(false);
        Wire.requestFrom(IMU_DEVICE_ADDR, 2);
        int16_t raw_gyro_z = (Wire.read() << 8) | Wire.read();
        gyro_sum += raw_gyro_z / IMU_GYRO_SENS;
        delay(2);
    }

    gyro_bias = gyro_sum / samples;
    Serial.print("Gyro bias: "); Serial.println(gyro_bias, 6);

    // Reset PID controllers
    wall_follow_pid.reset();
    straight_pid.reset();
    rotation_pid.reset();

    delay(1000);
}

float adcToDistance(int adc) {
    // Improved distance calibration using lookup table with interpolation
    static const int adc_table[] = {4095, 3500, 2800, 2200, 1800, 1400, 1100, 900, 750, 600};
    static const float dist_table[] = {20, 30, 40, 50, 60, 70, 80, 90, 100, 120};
    static const int table_size = sizeof(adc_table) / sizeof(adc_table[0]);

    if (adc >= adc_table[0]) return dist_table[0];
    if (adc <= adc_table[table_size-1]) return dist_table[table_size-1];

    for (int i = 0; i < table_size-1; i++) {
        if (adc <= adc_table[i] && adc >= adc_table[i+1]) {
            float ratio = (float)(adc - adc_table[i+1]) / (adc_table[i] - adc_table[i+1]);
            return dist_table[i+1] + ratio * (dist_table[i] - dist_table[i+1]);
        }
    }
    return 150.0f; // Default if no match
}

void readSensors() {
    sensors.timestamp = millis();
    const int samples = 10;
    long sum_fr = 0, sum_fl = 0, sum_r = 0, sum_l = 0;

    for (int i = 0; i < samples; i++) {
        digitalWrite(emitter_FR, HIGH);
        delayMicroseconds(50);
        sum_fr += analogRead(receiver_FR);
        digitalWrite(emitter_FR, LOW);

        digitalWrite(emitter_FL, HIGH);
        delayMicroseconds(50);
        sum_fl += analogRead(receiver_FL);
        digitalWrite(emitter_FL, LOW);

        digitalWrite(emitter_FR, HIGH);
        delayMicroseconds(50);
        sum_r += analogRead(receiver_R);
        digitalWrite(emitter_FR, LOW);

        digitalWrite(emitter_FL, HIGH);
        delayMicroseconds(50);
        sum_l += analogRead(receiver_L);
        digitalWrite(emitter_FL, LOW);

        delayMicroseconds(50);
    }

    sensors.front_right = adcToDistance(sum_fr / samples);
    sensors.front_left = adcToDistance(sum_fl / samples);
    sensors.right = adcToDistance(sum_r / samples);
    sensors.left = adcToDistance(sum_l / samples);

    const float wall_threshold = 120.0f;
    sensors.wall_front = (sensors.front_right < wall_threshold) && (sensors.front_left < wall_threshold);
    sensors.wall_right = sensors.right < wall_threshold;
    sensors.wall_left = sensors.left < wall_threshold;

    Wire.beginTransmission(IMU_DEVICE_ADDR);
    Wire.write(IMU_GYRO_X_H + 4);
    Wire.endTransmission(false);
    Wire.requestFrom(IMU_DEVICE_ADDR, 2);
    int16_t raw_gyro_z = (Wire.read() << 8) | Wire.read();
    sensors.gyro_z = (raw_gyro_z / IMU_GYRO_SENS) - gyro_bias;

    static uint32_t last_time = 0;
    uint32_t now = millis();
    float dt = (now - last_time) / 1000.0f;
    if (dt > 0 && dt < 0.1f) {
        current_angle += sensors.gyro_z * dt;
    }
    last_time = now;
}

void updateMaze() {
    int x = robot_pos.x;
    int y = robot_pos.y;
    if (x < 0 || x >= MAZE_SIZE || y < 0 || y >= MAZE_SIZE) return;

    maze[x][y].visited = true;

    uint8_t detected_walls = 0;

    switch (robot_pos.heading) {
        case NORTH:
            if (sensors.wall_front) detected_walls |= 1;  // North
            if (sensors.wall_right) detected_walls |= 2;  // East
            if (sensors.wall_left) detected_walls |= 8;   // West
            break;
        case EAST:
            if (sensors.wall_front) detected_walls |= 2;  // East
            if (sensors.wall_right) detected_walls |= 4;  // South
            if (sensors.wall_left) detected_walls |= 1;   // North
            break;
        case SOUTH:
            if (sensors.wall_front) detected_walls |= 4;  // South
            if (sensors.wall_right) detected_walls |= 8;  // West
            if (sensors.wall_left) detected_walls |= 2;   // East
            break;
        case WEST:
            if (sensors.wall_front) detected_walls |= 8;  // West
            if (sensors.wall_right) detected_walls |= 1;  // North
            if (sensors.wall_left) detected_walls |= 4;   // South
            break;
    }

    maze[x][y].walls = detected_walls;
    maze[x][y].confidence = min(255, maze[x][y].confidence + 1);

    if ((detected_walls & 1) && y < MAZE_SIZE - 1) maze[x][y+1].walls |= 4; // North wall adjacent South
    if ((detected_walls & 2) && x < MAZE_SIZE - 1) maze[x+1][y].walls |= 8; // East wall adjacent West
    if ((detected_walls & 4) && y > 0) maze[x][y-1].walls |= 1;            // South wall adjacent North
    if ((detected_walls & 8) && x > 0) maze[x-1][y].walls |= 2;            // West wall adjacent East

    Serial.printf("Maze updated at (%d,%d): walls=0x%02X\n", x, y, detected_walls);
}

void floodFill() {
    for (int x = 0; x < MAZE_SIZE; x++) {
        for (int y = 0; y < MAZE_SIZE; y++) {
            maze[x][y].distance = 255;
        }
    }

    for (int i = 0; i < 4; i++) {
        int gx = goal_positions[i].x;
        int gy = goal_positions[i].y;
        if (gx >= 0 && gx < MAZE_SIZE && gy >= 0 && gy < MAZE_SIZE) {
            maze[gx][gy].distance = 0;
        }
    }

    int queue_x[MAZE_SIZE * MAZE_SIZE];
    int queue_y[MAZE_SIZE * MAZE_SIZE];
    int queue_front = 0, queue_rear = 0;

    for (int i = 0; i < 4; i++) {
        queue_x[queue_rear] = goal_positions[i].x;
        queue_y[queue_rear] = goal_positions[i].y;
        queue_rear++;
    }

    int dx[] = {0, 1, 0, -1};
    int dy[] = {1, 0, -1, 0};
    uint8_t wall_bits[] = {1, 2, 4, 8};
    uint8_t opp_wall_bits[] = {4, 8, 1, 2};

    while (queue_front < queue_rear) {
        int cx = queue_x[queue_front];
        int cy = queue_y[queue_front];
        queue_front++;

        uint16_t current_dist = maze[cx][cy].distance;

        for (int dir = 0; dir < 4; dir++) {
            int nx = cx + dx[dir];
            int ny = cy + dy[dir];

            if (nx < 0 || nx >= MAZE_SIZE || ny < 0 || ny >= MAZE_SIZE) continue;

            bool wall_current = (maze[cx][cy].walls & wall_bits[dir]) != 0;
            bool wall_neighbor = (maze[nx][ny].walls & opp_wall_bits[dir]) != 0;

            if (wall_current || wall_neighbor) continue;

            if (maze[nx][ny].distance > current_dist + 1) {
                maze[nx][ny].distance = current_dist + 1;
                queue_x[queue_rear] = nx;
                queue_y[queue_rear] = ny;
                queue_rear++;
            }
        }
    }
}

uint8_t getOptimalDirection() {
    floodFill();

    int x = robot_pos.x;
    int y = robot_pos.y;
    uint16_t current_dist = maze[x][y].distance;

    int dx[] = {0, 1, 0, -1};
    int dy[] = {1, 0, -1, 0};
    uint8_t wall_bits[] = {1, 2, 4, 8};

    uint8_t best_direction = robot_pos.heading;
    uint16_t min_distance = current_dist;

    uint8_t priority_order[] = {
        robot_pos.heading,
        (robot_pos.heading + 3) % 4, // Left
        (robot_pos.heading + 1) % 4, // Right
        (robot_pos.heading + 2) % 4  // Back
    };

    for (int p = 0; p < 4; p++) {
        uint8_t dir = priority_order[p];
        int nx = x + dx[dir];
        int ny = y + dy[dir];

        if (nx < 0 || nx >= MAZE_SIZE || ny < 0 || ny >= MAZE_SIZE) continue;
        if (maze[x][y].walls & wall_bits[dir]) continue;

        uint16_t neighbor_dist = maze[nx][ny].distance;

        if (neighbor_dist < min_distance || (neighbor_dist == min_distance && p == 0)) {
            min_distance = neighbor_dist;
            best_direction = dir;
        }
    }

    Serial.printf("Current pos: (%d,%d), dist: %d, best direction: %d\n",
                  x, y, current_dist, best_direction);
    return best_direction;
}

void moveForward() {
    const float target_distance = CELL_SIZE;
    const int base_speed = exploration_mode ? EXPLORATION_SPEED : FAST_RUN_SPEED;

    noInterrupts();
    encoder_left = 0;
    encoder_right = 0;
    interrupts();

    float distance_traveled = 0;
    uint32_t move_start = millis();

    while (distance_traveled < target_distance && (millis() - move_start) < 5000) {
        readSensors();

        noInterrupts();
        long enc_l = encoder_left;
        long enc_r = encoder_right;
        interrupts();

        const float encoder_to_mm = 0.183f; // Calibrated
        float left_dist = enc_l * encoder_to_mm;
        float right_dist = enc_r * encoder_to_mm;
        distance_traveled = (left_dist + right_dist) / 2.0f;

        float correction = 0;
        if (sensors.wall_left && sensors.wall_right) {
            float wall_error = sensors.right - sensors.left;
            correction = wall_follow_pid.compute(0, wall_error);
        } else if (sensors.wall_left) {
            correction = wall_follow_pid.compute(90.0f, sensors.left);
        } else if (sensors.wall_right) {
            correction = -wall_follow_pid.compute(90.0f, sensors.right);
        } else {
            correction = straight_pid.compute(0, current_angle);
        }

        int left_speed = base_speed + correction;
        int right_speed = base_speed - correction;

        left_speed = constrain(left_speed, -MAX_SPEED, MAX_SPEED);
        right_speed = constrain(right_speed, -MAX_SPEED, MAX_SPEED);

        setMotorSpeeds(left_speed, right_speed);

        if (sensors.wall_front && distance_traveled > target_distance * 0.8f) {
            Serial.println("Front wall detected, stopping early");
            break;
        }
        delay(5);
    }

    stopMotors();
    delay(100);

    switch (robot_pos.heading) {
        case NORTH: robot_pos.y++; break;
        case EAST: robot_pos.x++; break;
        case SOUTH: robot_pos.y--; break;
        case WEST: robot_pos.x--; break;
    }

    Serial.printf("Moved forward to (%d,%d), distance: %.1f mm\n",
                  robot_pos.x, robot_pos.y, distance_traveled);
}

void turnLeft() {
    executeTurn(-90.0f);
    robot_pos.heading = (robot_pos.heading + 3) % 4;
}

void turnRight() {
    executeTurn(90.0f);
    robot_pos.heading = (robot_pos.heading + 1) % 4;
}

void turnAround() {
    executeTurn(180.0f);
    robot_pos.heading = (robot_pos.heading + 2) % 4;
}

void executeTurn(float target_angle) {
    current_angle = 0;
    rotation_pid.reset();

    uint32_t turn_start = millis();
    const uint32_t turn_timeout = 3000;
    const float angle_tolerance = 2.0f;

    while (abs(current_angle - target_angle) > angle_tolerance &&
           (millis() - turn_start) < turn_timeout) {
        readSensors();

        float correction = rotation_pid.compute(target_angle, current_angle);
        int turn_speed = constrain(abs(correction), 30, 80);

        if (target_angle > 0) {
            setMotorSpeeds(-turn_speed, turn_speed);
        } else {
            setMotorSpeeds(turn_speed, -turn_speed);
        }
        delay(5);
    }
    stopMotors();
    delay(200);

    Serial.printf("Turn completed: target=%.1f, actual=%.1f\n", target_angle, current_angle);
}

void setMotorSpeeds(int left_speed, int right_speed) {
    if (left_speed >= 0) {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        analogWrite(PWMA, left_speed);
    } else {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
        analogWrite(PWMA, -left_speed);
    }

    if (right_speed >= 0) {
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
        analogWrite(PWMB, right_speed);
    } else {
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
        analogWrite(PWMB, -right_speed);
    }
}

void stopMotors() {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
}

bool isGoalReached() {
    for (int i = 0; i < 4; i++) {
        if (robot_pos.x == goal_positions[i].x && robot_pos.y == goal_positions[i].y) {
            return true;
        }
    }
    return false;
}

void optimizeForSpeedRun() {
    exploration_mode = false;
    wall_follow_pid.kp = 20.0f;
    wall_follow_pid.ki = 0.05f;
    wall_follow_pid.kd = 10.0f;
    straight_pid.kp = 30.0f;
    straight_pid.ki = 0.1f;
    straight_pid.kd = 15.0f;
    rotation_pid.kp = 12.0f;
    rotation_pid.ki = 0.05f;
    rotation_pid.kd = 4.0f;

    wall_follow_pid.reset();
    straight_pid.reset();
    rotation_pid.reset();
}

void handleEmergency() {
    stopMotors();
    Serial.println("Emergency stop triggered!");
    while (1);
}

void displayStatus() {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.printf("Pos: (%d,%d)", robot_pos.x, robot_pos.y);
    display.setCursor(0, 10);
    display.printf("Dir: %d", robot_pos.heading);
    display.setCursor(0, 20);
    display.printf("Front: %.1fmm", sensors.front_left);
    display.setCursor(0, 30);
    display.printf("Left: %.1fmm, Right: %.1fmm", sensors.left, sensors.right);
    display.display();
}

void loop() {
    uint32_t run_time = millis() - run_start_time;
    if (run_time > 480000) { // 8 minutes
        stopMotors();
        Serial.println("Competition time elapsed, stopping robot.");
        while (1);
    }

    readSensors();
    updateMaze();
    displayStatus();

    if (isGoalReached()) {
        if (exploration_mode) {
            Serial.println("Goal reached! Switching to fast run mode.");
            optimizeForSpeedRun();
        } else {
            Serial.println("Completed fast run.");
            stopMotors();
            while (1);
        }
    }

    if (exploration_mode) {
        uint8_t next_dir = getOptimalDirection();
        if (next_dir == robot_pos.heading) {
            moveForward();
        } else if (next_dir == (robot_pos.heading + 1) % 4) {
            turnRight();
        } else if (next_dir == (robot_pos.heading + 3) % 4) {
            turnLeft();
        } else {
            turnAround();
        }
    } else {
        // Fast run - just follow the flood fill path quickly
        uint8_t next_dir = getOptimalDirection();
        if (next_dir == robot_pos.heading) {
            moveForward();
        } else if (next_dir == (robot_pos.heading + 1) % 4) {
            turnRight();
        } else if (next_dir == (robot_pos.heading + 3) % 4) {
            turnLeft();
        } else {
            turnAround();
        }
    }
}
