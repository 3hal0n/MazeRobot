#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

#define FORWARD 0
#define RIGHT 1
#define BACKWARD 2
#define LEFT 3

// Motor pins
#define PWMA 12
#define PWMB 13
#define AIN1 4
#define AIN2 16
#define BIN1 15
#define BIN2 2

#define IMU_DEVICE_ADDR  0x69
#define IMU_PWR_CTRL     0x6B
#define IMU_ACCEL_X_H    0x3B
#define IMU_GYRO_X_H     0x43
#define IMU_ID_REG       0x75
#define IMU_ACCEL_SENS   16384.0
#define IMU_GYRO_SENS    131.0

// Encoder pins
const int encoderPin1A = 5;
const int encoderPin1B = 18;
const int encoderPin2A = 19;
const int encoderPin2B = 23;

// IR sensor pins
const int emitter_FR = 32;
const int receiver_FR = 26;
const int emitter_FL = 33;
const int receiver_FL = 27;
const int receiver_R = 25;
const int receiver_L = 14;

#define QUEUE_SIZE 100

// Sensor values
int TotsensorValue_FR = 0, TotsensorValue_FL = 0;
int TotsensorValue_L = 0, TotsensorValue_R = 0;
int sensorValue_FR = 0, sensorValue_FL = 0, sensorValue_L = 0, sensorValue_R = 0;

// Distance variables
float distance = 0.0;
float distance_L = 0.0;

// Speed & PID
const int maxspeed = 180;
const int normalspeed = 80;
double kp = 20.7, ki = 0.005, kd = -40.5;
double errSum = 0, lastErr = 0, Output = 0;
double yaw_kp = 3.0, yaw_ki = 0.0, yaw_kd = 0.2; // Increased kp, kd for tighter yaw control
double yaw_errSum = 0, yaw_lastErr = 0, yaw_Output = 0;
double Setpoint = 4.0;
double yaw_Setpoint = 0.0;
double deadband = 0.5;
const int rightMotorOffset = 2.5; // Slight boost to right motor to counter right drift
const int leftMotorOffset = 3; // Slight reduction to left motor to counter left veering
unsigned long previousTime = 0;

// Distance lookup
int adcValues[] = {4136, 4012, 3440, 2765, 2240, 1800, 1500, 1240, 1200};
float distances[] = {2, 3, 4, 5, 6, 7, 8, 9, 10};
int n = sizeof(adcValues) / sizeof(adcValues[0]);

int adcValues_L[] = {4136, 4070, 2877, 2200, 1578, 1200, 900, 787, 655};
float distances_L[] = {2, 3, 4, 5, 6, 7, 8, 9, 10};
int n_L = sizeof(adcValues_L) / sizeof(adcValues_L[0]);

// Flood-fill arrays
int cellsArray[7][7] = {0};
int floodArray[7][7] = {
    {6, 5, 4, 3, 4, 5, 6},
    {5, 4, 3, 2, 3, 4, 5},
    {4, 3, 2, 1, 2, 3, 4},
    {3, 2, 1, 0, 1, 2, 3},
    {4, 3, 2, 1, 2, 3, 4},
    {5, 4, 3, 2, 3, 4, 5},
    {6, 5, 4, 3, 4, 5, 6}
};

int current_x = 0, current_y = 0, previous_x = 0, previous_y = 0, orient = FORWARD;
bool goal_reached = false;

// Queue for flood-fill
int queueX[QUEUE_SIZE], queueY[QUEUE_SIZE];
int queueFront = 0, queueRear = -1, queueCount = 0;

// Encoder data
volatile long encoderValue_1 = 0;
volatile long encoderValue_2 = 0;
const int countsPerRev = 1440;
long lastValue_1 = 0, lastValue_2 = 0;
int cwrev_1 = 0, cwwrev_1 = 0;
int cwrev_2 = 0, cwwrev_2 = 0;

// Sequential movement
float encddist = 0.0;
float onecell = 18.0;
bool isStopped = false;
unsigned long stopTime = 0;
const long cellDelay = 2000;
bool cellDelayActive = false;
const long displayInterval = 200;
unsigned long lastDisplayUpdate = 0;

// State machine
enum State { IDLE, MOVING_FORWARD, TURNING_LEFT, TURNING_RIGHT, TURNING_BACK, STOPPED };
State currentState = IDLE;
char pendingDirection = 'F';
bool movementComplete = true;

// Turn tracking
int turnCount = 0;
unsigned long lastTurnTime = 0;

// IMU data
float accel_x_data, accel_y_data, accel_z_data;
float gyro_x_rate, gyro_y_rate, gyro_z_rate;
float roll_angle, pitch_angle, yaw_angle;
float last_yaw = 0, last_time = 0;
float gyro_z_bias = 0;
float filtered_gyro_z = 0;

// Function declarations
void lmf();
void lmb();
void rmf();
void rmb();
void sM();
void read_ir_sensors();
double RPIDcontrol(float Dwall);
double LPIDcontrol(float Dwall);
double yawPIDcontrol(float current_yaw);
void Rspeedy(int speed);
void Lspeedy(int speed);
void print_values();
float getDistanceFromADC_Right(int adcValue);
float getDistanceFromADC_Left(int adcValue_L);
float rightsensor();
float leftsensor();
float frontsensor();
void handleEncoder1();
void handleEncoder2();
bool moveForward();
bool turnLeft();
bool turnRight();
bool readLeftSensor();
bool readRightSensor();
bool readFrontSensor();
void queuePush(int x, int y);
bool queuePop(int *x, int *y);
bool queueEmpty();
void updateCells(int x, int y, int orient, bool left, bool right, bool forward);
bool isAccessible(int current_x, int current_y, int target_x, int target_y);
void getSurroundings(int current_x, int current_y, int *north_x, int *north_y, int *east_x, int *east_y, int *south_x, int *south_y, int *west_x, int *west_y);
bool isIncrementConsistent(int current_x, int current_y);
void makeCellConsistent(int current_x, int current_y);
void floodFillUsingQueue(int start_x, int start_y, int previous_x, int previous_y);
char whereToMove(int current_x, int current_y, int previous_x, int previous_y, int orient);
void logArrays();
int updateOrientation(int current_orient, char direction);
void updateCoordinates(int orient, int *x, int *y);
void displayFlood();
void retrieveIMUData();
void calculateAngles();
void calibrateGyro();
bool handleSpinRecovery();

void setup() {
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  sM();

  Serial.begin(115200);
  Serial.println("Micromouse Starting...");

  pinMode(receiver_R, INPUT);
  pinMode(receiver_FR, INPUT);
  pinMode(receiver_FL, INPUT);
  pinMode(receiver_L, INPUT);
  pinMode(emitter_FL, OUTPUT);
  pinMode(emitter_FR, OUTPUT);

  pinMode(encoderPin1A, INPUT_PULLUP);
  pinMode(encoderPin1B, INPUT_PULLUP);
  pinMode(encoderPin2A, INPUT_PULLUP);
  pinMode(encoderPin2B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderPin1A), handleEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin1B), handleEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin2A), handleEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin2B), handleEncoder2, CHANGE);

  Wire.begin(21, 22);
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("OLED init failed"));
    while (true);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Micromouse Ready");
  display.display();
  delay(2000);

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

  calibrateGyro();
  last_time = millis() / 1000.0;
}

void lmf() { digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); }
void lmb() { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); }
void rmf() { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); }
void rmb() { digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); }
void sM() {
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
  analogWrite(PWMA, 0); analogWrite(PWMB, 0);
}

void read_ir_sensors() {
  TotsensorValue_FR = 0; TotsensorValue_FL = 0;
  TotsensorValue_R = 0; TotsensorValue_L = 0;

  const int samples = 20;
  for (int i = 0; i < samples; i++) {
    digitalWrite(emitter_FR, HIGH);
    delayMicroseconds(100);
    sensorValue_FR = analogRead(receiver_FR);
    TotsensorValue_FR += sensorValue_FR;
    digitalWrite(emitter_FR, LOW);

    digitalWrite(emitter_FL, HIGH);
    delayMicroseconds(100);
    sensorValue_FL = analogRead(receiver_FL);
    TotsensorValue_FL += sensorValue_FL;
    digitalWrite(emitter_FL, LOW);

    digitalWrite(emitter_FR, HIGH);
    delayMicroseconds(100);
    sensorValue_R = analogRead(receiver_R);
    TotsensorValue_R += sensorValue_R;
    digitalWrite(emitter_FR, LOW);

    digitalWrite(emitter_FL, HIGH);
    delayMicroseconds(100);
    sensorValue_L = analogRead(receiver_L);
    TotsensorValue_L += sensorValue_L;
    digitalWrite(emitter_FL, LOW);
    delayMicroseconds(100);
  }

  TotsensorValue_FR /= samples;
  TotsensorValue_FL /= samples;
  TotsensorValue_R /= samples;
  TotsensorValue_L /= samples;

  Serial.print("IR Sensors: FR="); Serial.print(TotsensorValue_FR);
  Serial.print(", FL="); Serial.print(TotsensorValue_FL);
  Serial.print(", R="); Serial.print(TotsensorValue_R);
  Serial.print(", L="); Serial.println(TotsensorValue_L);
}

double RPIDcontrol(float Dwall) {
  unsigned long now = millis();
  double timeChange = (now - previousTime) / 1000.0;
  if (timeChange <= 0) timeChange = 0.001;

  double error = Setpoint - Dwall;
  if (abs(error) < deadband) error = 0;
  if (abs(Output) > maxspeed / 2) errSum = 0;

  errSum += error * timeChange;
  double dErr = (error - lastErr) / timeChange;

  Output = kp * error + ki * errSum + kd * dErr;
  Output = constrain(Output, -20, 20);
  lastErr = error;
  previousTime = now;

  Rspeedy((int)Output);
  return Output;
}

double LPIDcontrol(float Dwall) {
  unsigned long now = millis();
  double timeChange = (now - previousTime) / 1000.0;
  if (timeChange <= 0) timeChange = 0.001;

  double error = Setpoint - Dwall;
  if (abs(error) < deadband) error = 0;
  if (abs(Output) > maxspeed / 2) errSum = 0;

  errSum += error * timeChange;
  double dErr = (error - lastErr) / timeChange;

  Output = kp * error + ki * errSum + kd * dErr;
  Output = constrain(Output, -20, 20);
  lastErr = error;
  previousTime = now;

  Lspeedy((int)Output);
  return Output;
}

double yawPIDcontrol(float current_yaw) {
  unsigned long now = millis();
  double timeChange = (now - previousTime) / 1000.0;
  if (timeChange <= 0) timeChange = 0.001;

  double yaw_error = yaw_Setpoint - current_yaw;
  if (abs(yaw_error) < 1.0) yaw_error = 0;
  yaw_errSum += yaw_error * timeChange;
  double yaw_dErr = (yaw_error - yaw_lastErr) / timeChange;

  yaw_Output = yaw_kp * yaw_error + yaw_ki * yaw_errSum + yaw_kd * yaw_dErr;
  yaw_Output = constrain(yaw_Output, -20, 20);
  yaw_lastErr = yaw_error;

  return yaw_Output;
}

void Rspeedy(int speed) {
  static int lastLeftSpeed = normalspeed;
  static int lastRightSpeed = normalspeed;

  int LeftSpeed = normalspeed - speed + leftMotorOffset; // Added leftMotorOffset
  int RightSpeed = normalspeed + speed + rightMotorOffset; // Offset to boost right motor

  LeftSpeed = constrain(LeftSpeed, -maxspeed, maxspeed);
  RightSpeed = constrain(RightSpeed, -maxspeed, maxspeed);

  int slewRate = 5;
  LeftSpeed = constrain(LeftSpeed, lastLeftSpeed - slewRate, lastLeftSpeed + slewRate);
  RightSpeed = constrain(RightSpeed, lastRightSpeed - slewRate, lastRightSpeed + slewRate);

  lastLeftSpeed = LeftSpeed;
  lastRightSpeed = RightSpeed;

  if (LeftSpeed >= 0) { lmf(); analogWrite(PWMA, LeftSpeed); }
  else { lmb(); analogWrite(PWMA, -LeftSpeed); }

  if (RightSpeed >= 0) { rmf(); analogWrite(PWMB, RightSpeed); }
  else { rmb(); analogWrite(PWMB, -RightSpeed); }
}

void Lspeedy(int speed) {
  static int lastLeftSpeed = normalspeed;
  static int lastRightSpeed = normalspeed;

  int LeftSpeed = normalspeed + speed + leftMotorOffset; // Added leftMotorOffset
  int RightSpeed = normalspeed - speed + rightMotorOffset; // Offset to boost right motor

  LeftSpeed = constrain(LeftSpeed, -maxspeed, maxspeed);
  RightSpeed = constrain(RightSpeed, -maxspeed, maxspeed);

  int slewRate = 5;
  LeftSpeed = constrain(LeftSpeed, lastLeftSpeed - slewRate, lastLeftSpeed + slewRate);
  RightSpeed = constrain(RightSpeed, lastRightSpeed - slewRate, lastRightSpeed + slewRate);

  lastLeftSpeed = LeftSpeed;
  lastRightSpeed = RightSpeed;

  if (LeftSpeed >= 0) { lmf(); analogWrite(PWMA, LeftSpeed); }
  else { lmb(); analogWrite(PWMA, -LeftSpeed); }

  if (RightSpeed >= 0) { rmf(); analogWrite(PWMB, RightSpeed); }
  else { rmb(); analogWrite(PWMB, -RightSpeed); }
}

void print_values() {
  if (millis() - lastDisplayUpdate < displayInterval) return;
  lastDisplayUpdate = millis();

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("State: ");
  switch (currentState) {
    case IDLE: display.print("IDLE"); break;
    case MOVING_FORWARD: display.print("FORWARD"); break;
    case TURNING_LEFT: display.print("LEFT"); break;
    case TURNING_RIGHT: display.print("RIGHT"); break;
    case TURNING_BACK: display.print("BACK"); break;
    case STOPPED: display.print("STOPPED"); break;
  }

  display.setCursor(0, 10);
  display.print("L:"); display.print(distance_L, 1); display.print("cm");
  display.setCursor(64, 10);
  display.print("R:"); display.print(distance, 1); display.print("cm");

  display.setCursor(0, 20);
  display.print("Pos:("); display.print(current_x); display.print(","); display.print(current_y); display.print(")");

  display.setCursor(0, 30);
  display.print("Yaw:"); display.print(yaw_angle, 1); display.print("deg");

  display.display();
}

float getDistanceFromADC_Right(int adcValue) {
  if (adcValue >= adcValues[0]) return distances[0];
  if (adcValue <= adcValues[n - 1]) return distances[n - 1];

  for (int i = 0; i < n - 1; i++) {
    if (adcValue <= adcValues[i] && adcValue >= adcValues[i + 1]) {
      float x0 = adcValues[i], x1 = adcValues[i + 1];
      float y0 = distances[i], y1 = distances[i + 1];
      return y0 + (adcValue - x0) * (y1 - y0) / (x1 - x0);
    }
  }
  return -1;
}

float getDistanceFromADC_Left(int adcValue_L) {
  if (adcValue_L >= adcValues_L[0]) return distances_L[0];
  if (adcValue_L <= adcValues_L[n_L - 1]) return distances_L[n_L - 1];

  for (int i = 0; i < n_L - 1; i++) {
    if (adcValue_L <= adcValues_L[i] && adcValue_L >= adcValues_L[i + 1]) {
      float x0_L = adcValues_L[i], x1_L = adcValues_L[i + 1];
      float y0_L = distances_L[i], y1_L = distances_L[i + 1];
      return y0_L + (adcValue_L - x0_L) * (y1_L - y0_L) / (x1_L - x0_L);
    }
  }
  return -1;
}

float rightsensor() {
  if (TotsensorValue_R > 0) {
    distance = getDistanceFromADC_Right(TotsensorValue_R);
    float constrained_distance = constrain(distance, 2, 20);
    Serial.print("Right Sensor: ADC="); Serial.print(TotsensorValue_R);
    Serial.print(", RawDist="); Serial.print(distance);
    Serial.print(", ConstrainedDist="); Serial.println(constrained_distance);
    return constrained_distance;
  }
  Serial.println("Right Sensor: ADC=0, Returning 9999");
  return 9999;
}

float leftsensor() {
  if (TotsensorValue_L > 0) {
    distance_L = getDistanceFromADC_Left(TotsensorValue_L);
    float constrained_distance = constrain(distance_L, 2, 20);
    Serial.print("Left Sensor: ADC="); Serial.print(TotsensorValue_L);
    Serial.print(", RawDist="); Serial.print(distance_L);
    Serial.print(", ConstrainedDist="); Serial.println(constrained_distance);
    return constrained_distance;
  }
  Serial.println("Left Sensor: ADC=0, Returning 9999");
  return 9999;
}

float frontsensor() {
  if (TotsensorValue_FR > 0) {
    float distance_0 = 70941 * pow(TotsensorValue_FR, -1.20);
    float constrained_distance = constrain(distance_0, 2, 20);
    Serial.print("Front Sensor: ADC_FR="); Serial.print(TotsensorValue_FR);
    Serial.print(", RawDist="); Serial.print(distance_0);
    Serial.print(", ConstrainedDist="); Serial.println(constrained_distance);
    return constrained_distance;
  }
  Serial.println("Front Sensor: ADC=0, Returning 9999");
  return 9999;
}

void IRAM_ATTR handleEncoder1() {
  static uint8_t lastState = 0;
  uint8_t state = (digitalRead(encoderPin1A) << 1) | digitalRead(encoderPin1B);
  uint8_t transition = (lastState << 2) | state;

  if (transition == 0b1101 || transition == 0b0100 || transition == 0b0010 || transition == 0b1011)
    encoderValue_1--;
  else if (transition == 0b1110 || transition == 0b0111 || transition == 0b0001 || transition == 0b1000)
    encoderValue_1++;

  lastState = state;
}

void IRAM_ATTR handleEncoder2() {
  static uint8_t lastState = 0;
  uint8_t state = (digitalRead(encoderPin2A) << 1) | digitalRead(encoderPin2B);
  uint8_t transition = (lastState << 2) | state;

  if (transition == 0b1101 || transition == 0b0100 || transition == 0b0010 || transition == 0b1011)
    encoderValue_2--;
  else if (transition == 0b1110 || transition == 0b0111 || transition == 0b0001 || transition == 0b1000)
    encoderValue_2++;

  lastState = state;
}

bool moveForward() {
  if (isStopped) {
    Serial.println("Stopped permanently");
    return true;
  }

  if (cellDelayActive) {
    if (millis() - stopTime >= cellDelay) {
      cellDelayActive = false;
      Serial.println("Cell delay complete, resuming movement");
      yaw_Setpoint = yaw_angle; // Reset yaw setpoint at cell boundary
      return true; // Movement complete after delay
    }
    return false; // Still in delay
  }

  unsigned long currentTime = millis();
  retrieveIMUData();
  calculateAngles();
  read_ir_sensors();
  float Rdistcm = rightsensor();
  float Ldistcm = leftsensor();

  Serial.print("Moving Forward: Rdist="); Serial.print(Rdistcm);
  Serial.print(", Ldist="); Serial.print(Ldistcm);
  Serial.print(", EncDist="); Serial.print(encddist);
  Serial.print(", Pos=("); Serial.print(current_x); Serial.print(","); Serial.print(current_y);
  Serial.print("), Yaw="); Serial.print(yaw_angle);
  Serial.print(", Enc1="); Serial.print(encoderValue_1);
  Serial.print(", Enc2="); Serial.println(encoderValue_2);

  double pid_output = 0;
  if (Rdistcm > 8.0 && Ldistcm > 8.0) {
    pid_output = yawPIDcontrol(yaw_angle);
    lmf(); rmf();
    analogWrite(PWMA, normalspeed + (int)pid_output + leftMotorOffset); // Added leftMotorOffset for consistency
    analogWrite(PWMB, normalspeed - (int)pid_output + rightMotorOffset);
  } else {
    if (Rdistcm < Ldistcm && Rdistcm < 8.0) {
      pid_output = RPIDcontrol(Rdistcm);
    } else if (Ldistcm <= Rdistcm && Ldistcm < 8.0) {
      pid_output = LPIDcontrol(Ldistcm);
    } else {
      pid_output = yawPIDcontrol(yaw_angle);
      lmf(); rmf();
      analogWrite(PWMA, normalspeed + (int)pid_output + leftMotorOffset); // Added leftMotorOffset for consistency
      analogWrite(PWMB, normalspeed - (int)pid_output + rightMotorOffset);
    }
  }

  noInterrupts();
  long val1 = encoderValue_1;
  long val2 = encoderValue_2;
  interrupts();

  const float encoderDivisor = 98.2;
  encddist = ((abs(val1) + abs(val2)) / 2.0) / encoderDivisor;

  if (val1 - lastValue_1 >= countsPerRev) {
    cwrev_1++; lastValue_1 = val1;
  } else if (val1 - lastValue_1 <= -countsPerRev) {
    cwwrev_1++; lastValue_1 = val1;
  }

  if (val2 - lastValue_2 >= countsPerRev) {
    cwrev_2++; lastValue_2 = val2;
  } else if (val2 - lastValue_2 <= -countsPerRev) {
    cwwrev_2++; lastValue_2 = val2;
  }

  if (encddist >= onecell) {
    sM();
    Serial.println("Stopping at cell, dist=" + String(encddist));
    noInterrupts();
    encoderValue_1 = 0; encoderValue_2 = 0;
    lastValue_1 = 0; lastValue_2 = 0;
    interrupts();
    encddist = 0.0;
    cwrev_1 = 0; cwwrev_1 = 0;
    cwrev_2 = 0; cwwrev_2 = 0;
    stopTime = millis();
    cellDelayActive = true;
    Serial.println("Starting 2-second cell delay");
    return false; // Delay not complete yet
  }
  return false;
}

bool turnLeft() {
  retrieveIMUData();
  calculateAngles();
  last_yaw = 0;
  yaw_angle = 0;
  float initial_yaw = 0;
  unsigned long startTime = millis();
  float target_yaw = 80.0;
  float tolerance = 3.0;

  while (abs(yaw_angle - target_yaw) > tolerance && millis() - startTime < 5000) {
    retrieveIMUData();
    calculateAngles();
    Serial.print("Turning Left, Yaw: "); Serial.print(yaw_angle);
    Serial.print(", Target: "); Serial.println(target_yaw);
    lmf(); rmb();
    analogWrite(PWMA, 50);
    analogWrite(PWMB, 50);
    read_ir_sensors();
    print_values();
  }
  sM();
  Serial.println("Left turn complete, Final Yaw: " + String(yaw_angle));
  return true;
}

bool turnRight() {
  retrieveIMUData();
  calculateAngles();
  last_yaw = 0;
  yaw_angle = 0;
  float initial_yaw = 0;
  unsigned long startTime = millis();
  float target_yaw = -80.0;
  float tolerance = 3.0;

  while (abs(yaw_angle - target_yaw) > tolerance && millis() - startTime < 5000) {
    retrieveIMUData();
    calculateAngles();
    Serial.print("Turning Right, Yaw: "); Serial.print(yaw_angle);
    Serial.print(", Target: "); Serial.print(target_yaw);
    Serial.print(", LeftMotor: "); Serial.print(digitalRead(AIN1)); Serial.print(digitalRead(AIN2));
    Serial.print(", RightMotor: "); Serial.print(digitalRead(BIN1)); Serial.print(digitalRead(BIN2));
    Serial.println();
    lmb(); rmf();
    analogWrite(PWMA, 50);
    analogWrite(PWMB, 50);
    read_ir_sensors();
    print_values();
  }
  sM();
  Serial.println("Right turn complete, Final Yaw: " + String(yaw_angle));
  return true;
}

bool readLeftSensor() {
  float LDist = leftsensor();
  bool wallDetected = LDist < 10.0;
  Serial.print("Left Sensor: ADC="); Serial.print(TotsensorValue_L);
  Serial.print(", Dist="); Serial.print(LDist);
  Serial.print(", WallDetected="); Serial.println(wallDetected);
  return wallDetected;
}

bool readRightSensor() {
  float RDist = rightsensor();
  bool wallDetected = RDist < 10.0;
  Serial.print("Right Sensor: ADC="); Serial.print(TotsensorValue_R);
  Serial.print(", Dist="); Serial.print(RDist);
  Serial.print(", WallDetected="); Serial.println(wallDetected);
  return wallDetected;
}

bool readFrontSensor() {
  float FRDist = frontsensor();
  float FLDist = leftsensor();
  bool wallDetected = FRDist < 10.0 && FLDist < 10.0;
  Serial.print("Front Sensor: ADC_FR="); Serial.print(TotsensorValue_FR);
  Serial.print(", ADC_FL="); Serial.print(TotsensorValue_FL);
  Serial.print(", Dist_FR="); Serial.print(FRDist);
  Serial.print(", WallDetected="); Serial.println(wallDetected);
  return wallDetected;
}

void queuePush(int x, int y) {
  if (queueCount >= QUEUE_SIZE) return;
  queueRear = (queueRear + 1) % QUEUE_SIZE;
  queueX[queueRear] = x;
  queueY[queueRear] = y;
  queueCount++;
}

bool queuePop(int *x, int *y) {
  if (queueCount == 0) return false;
  *x = queueX[queueFront];
  *y = queueY[queueFront];
  queueFront = (queueFront + 1) % QUEUE_SIZE;
  queueCount--;
  return true;
}

bool queueEmpty() {
  return queueCount == 0;
}

void updateCells(int x, int y, int orient, bool left, bool right, bool forward) {
  if (x < 0 || x >= 7 || y < 0 || y >= 7) {
    Serial.println("Error: Coordinates (" + String(x) + "," + String(y) + ") out of bounds");
    return;
  }
  Serial.println("Updating cell at (" + String(x) + "," + String(y) + "), orient=" + String(orient) +
                ", left=" + String(left) + ", right=" + String(right) + ", forward=" + String(forward));

  if (left && right && forward) {
    if (orient == FORWARD) cellsArray[x][y] = 13;
    else if (orient == RIGHT) cellsArray[x][y] = 12;
    else if (orient == BACKWARD) cellsArray[x][y] = 11;
    else if (orient == LEFT) cellsArray[x][y] = 14;
  } else if (left && right && !forward) {
    if (orient == FORWARD || orient == BACKWARD) cellsArray[x][y] = 9;
    else if (orient == RIGHT || orient == LEFT) cellsArray[x][y] = 10;
  } else if (left && !right && forward) {
    if (orient == FORWARD) cellsArray[x][y] = 8;
    else if (orient == RIGHT) cellsArray[x][y] = 7;
    else if (orient == BACKWARD) cellsArray[x][y] = 6;
    else if (orient == LEFT) cellsArray[x][y] = 5;
  } else if (!left && right && forward) {
    if (orient == FORWARD) cellsArray[x][y] = 7;
    else if (orient == RIGHT) cellsArray[x][y] = 6;
    else if (orient == BACKWARD) cellsArray[x][y] = 5;
    else if (orient == LEFT) cellsArray[x][y] = 8;
  } else if (forward) {
    if (orient == FORWARD) cellsArray[x][y] = 2;
    else if (orient == RIGHT) cellsArray[x][y] = 3;
    else if (orient == BACKWARD) cellsArray[x][y] = 4;
    else if (orient == LEFT) cellsArray[x][y] = 1;
  } else if (left) {
    if (orient == FORWARD) cellsArray[x][y] = 1;
    else if (orient == RIGHT) cellsArray[x][y] = 2;
    else if (orient == BACKWARD) cellsArray[x][y] = 3;
    else if (orient == LEFT) cellsArray[x][y] = 4;
  } else if (right) {
    if (orient == FORWARD) cellsArray[x][y] = 3;
    else if (orient == RIGHT) cellsArray[x][y] = 4;
    else if (orient == BACKWARD) cellsArray[x][y] = 1;
    else if (orient == LEFT) cellsArray[x][y] = 2;
  } else {
    cellsArray[x][y] = 0;
  }
  Serial.println("cellsArray[" + String(x) + "," + String(y) + "]=" + String(cellsArray[x][y]));
}

bool isAccessible(int current_x, int current_y, int target_x, int target_y) {
  if (current_x < 0 || current_x >= 7 || current_y < 0 || current_y >= 7 ||
      target_x < 0 || target_x >= 7 || target_y < 0 || target_y >= 7) {
    return false;
  }
  if (current_x == target_x) {
    if (current_y > target_y) {
      if (cellsArray[current_x][current_y] == 4 || cellsArray[current_x][current_y] == 5 ||
          cellsArray[current_x][current_y] == 6 || cellsArray[current_x][current_y] == 10 ||
          cellsArray[current_x][current_y] == 11 || cellsArray[current_x][current_y] == 12 ||
          cellsArray[current_x][current_y] == 14) {
        Serial.println("South blocked at (" + String(current_x) + "," + String(current_y) + ")");
        return false;
      }
      return true;
    } else {
      if (cellsArray[current_x][current_y] == 2 || cellsArray[current_x][current_y] == 7 ||
          cellsArray[current_x][current_y] == 8 || cellsArray[current_x][current_y] == 10 ||
          cellsArray[current_x][current_y] == 12 || cellsArray[current_x][current_y] == 13 ||
          cellsArray[current_x][current_y] == 14) {
        Serial.println("North blocked at (" + String(current_x) + "," + String(current_y) + ")");
        return false;
      }
      return true;
    }
  } else if (current_y == target_y) {
    if (current_x > target_x) {
      if (cellsArray[current_x][current_y] == 1 || cellsArray[current_x][current_y] == 5 ||
          cellsArray[current_x][current_y] == 8 || cellsArray[current_x][current_y] == 9 ||
          cellsArray[current_x][current_y] == 11 || cellsArray[current_x][current_y] == 13 ||
          cellsArray[current_x][current_y] == 14) {
        Serial.println("West blocked at (" + String(current_x) + "," + String(current_y) + ")");
        return false;
      }
      return true;
    } else {
      if (cellsArray[current_x][current_y] == 3 || cellsArray[current_x][current_y] == 6 ||
          cellsArray[current_x][current_y] == 7 || cellsArray[current_x][current_y] == 9 ||
          cellsArray[current_x][current_y] == 11 || cellsArray[current_x][current_y] == 12 ||
          cellsArray[current_x][current_y] == 13) {
        Serial.println("East blocked at (" + String(current_x) + "," + String(current_y) + ")");
        return false;
      }
      return true;
    }
  }
  return false;
}

void getSurroundings(int current_x, int current_y, int *north_x, int *north_y, int *east_x, int *east_y, int *south_x, int *south_y, int *west_x, int *west_y) {
  *north_x = current_y + 1 < 7 ? current_x : -1;
  *north_y = current_y + 1 < 7 ? current_y + 1 : -1;
  *east_x = current_x + 1 < 7 ? current_x + 1 : -1;
  *east_y = current_x + 1 < 7 ? current_y : -1;
  *south_x = current_y - 1 >= 0 ? current_x : -1;
  *south_y = current_y - 1 >= 0 ? current_y - 1 : -1;
  *west_x = current_x - 1 >= 0 ? current_x - 1 : -1;
  *west_y = current_x - 1 >= 0 ? current_y : -1;
}

bool isIncrementConsistent(int current_x, int current_y) {
  if (goal_reached) return true;
  int north_x, north_y, east_x, east_y, south_x, south_y, west_x, west_y;
  getSurroundings(current_x, current_y, &north_x, &north_y, &east_x, &east_y, &south_x, &south_y, &west_x, &west_y);

  int currentValue = floodArray[current_x][current_y];
  int minValues[4] = {1000, 1000, 1000, 1000};
  int minCounts = 0;

  if (north_x >= 0 && isAccessible(current_x, current_y, north_x, north_y))
    minValues[FORWARD] = floodArray[north_x][north_y];
  if (east_x >= 0 && isAccessible(current_x, current_y, east_x, east_y))
    minValues[RIGHT] = floodArray[east_x][east_y];
  if (south_x >= 0 && isAccessible(current_x, current_y, south_x, south_y))
    minValues[BACKWARD] = floodArray[south_x][south_y];
  if (west_x >= 0 && isAccessible(current_x, current_y, west_x, west_y))
    minValues[LEFT] = floodArray[west_x][west_y];

  for (int i = 0; i < 4; i++) {
    if (minValues[i] == currentValue - 1 && minValues[i] != 1000)
      minCounts++;
  }
  return minCounts > 0;
}

void makeCellConsistent(int current_x, int current_y) {
  if (goal_reached) return;
  int north_x, north_y, east_x, east_y, south_x, south_y, west_x, west_y;
  getSurroundings(current_x, current_y, &north_x, &north_y, &east_x, &east_y, &south_x, &south_y, &west_x, &west_y);

  int currentValue = floodArray[current_x][current_y];
  int minValues[4] = {1000, 1000, 1000, 1000};

  if (north_x >= 0 && isAccessible(current_x, current_y, north_x, north_y))
    minValues[FORWARD] = floodArray[north_x][north_y];
  if (east_x >= 0 && isAccessible(current_x, current_y, east_x, east_y))
    minValues[RIGHT] = floodArray[east_x][east_y];
  if (south_x >= 0 && isAccessible(current_x, current_y, south_x, south_y))
    minValues[BACKWARD] = floodArray[south_x][south_y];
  if (west_x >= 0 && isAccessible(current_x, current_y, west_x, west_y))
    minValues[LEFT] = floodArray[west_x][west_y];

  int minimalValue = 1000;
  for (int i = 0; i < 4; i++) {
    if (minValues[i] < minimalValue)
      minimalValue = minValues[i];
  }
  if (minimalValue != 1000) {
    floodArray[current_x][current_y] = minimalValue + 1;
    Serial.println("Updated floodArray[" + String(current_x) + "," + String(current_y) + "] to " + String(floodArray[current_x][current_y]));
  }
}

void floodFillUsingQueue(int start_x, int start_y, int previous_x, int previous_y) {
  if (goal_reached) return;
  queueFront = 0;
  queueRear = -1;
  queueCount = 0;
  queuePush(start_x, start_y);

  while (!queueEmpty()) {
    int current_x, current_y;
    if (!queuePop(&current_x, &current_y)) { // Fixed syntax error
      Serial.println("Queue pop failed");
      return;
    }

    if (!isIncrementConsistent(current_x, current_y)) {
      Serial.println("Inconsistent at (" + String(current_x) + "," + String(current_y) + ")");
      makeCellConsistent(current_x, current_y);
      int north_x, north_y, east_x, east_y, south_x, south_y, west_x, west_y;
      getSurroundings(current_x, current_y, &north_x, &north_y, &east_x, &east_y, &south_x, &south_y, &west_x, &west_y);

      int neighborsX[] = {north_x, east_x, south_x, west_x};
      int neighborsY[] = {north_y, east_y, south_y, west_y};

      int accessibleCount = 0;
      int accessibleDir = -1;
      for (int i = 0; i < 4; i++) {
        if (neighborsX[i] >= 0 && isAccessible(current_x, current_y, neighborsX[i], neighborsY[i])) {
          accessibleCount++;
          accessibleDir = i;
        }
      }
      Serial.print("Accessible neighbors at (" + String(current_x) + "," + String(current_y) + "): ");
      Serial.println(accessibleCount);
      if (accessibleCount == 1 && current_x == start_x && current_y == start_y) {
        Serial.println("Dead-end detected, open direction: " + String(accessibleDir));
      }

      for (int i = 0; i < 4; i++) {
        int neighborX = neighborsX[i];
        int neighborY = neighborsY[i];
        if (neighborX >= 0 && isAccessible(current_x, current_y, neighborX, neighborY)) {
          queuePush(neighborX, neighborY);
        }
      }
    } else {
      Serial.println("Consistent at (" + String(current_x) + "," + String(current_y) + ")");
    }
  }
  logArrays();
}

char whereToMove(int current_x, int current_y, int previous_x, int previous_y, int orient) {
  if (goal_reached) return 'S';
  int north_x, north_y, east_x, east_y, south_x, south_y, west_x, west_y;
  getSurroundings(current_x, current_y, &north_x, &north_y, &east_x, &east_y, &south_x, &south_y, &west_x, &west_y);
  int minValue = 1000;
  int minCell = -1;
  int accessiblePathsNum = 0;
  int accessibleNeighbors[4] = {1000, 1000, 1000, 1000};

  int previous = -1;
  if (south_x == previous_x && south_y == previous_y) previous = BACKWARD;
  else if (north_x == previous_x && north_y == previous_y) previous = FORWARD;
  else if (east_x == previous_x && east_y == previous_y) previous = RIGHT;
  else if (west_x == previous_x && west_y == previous_y) previous = LEFT;

  for (int i = 0; i < 4; i++) {
    int neighborX = 0, neighborY = 0;
    switch (i) {
      case FORWARD: neighborX = north_x; neighborY = north_y; break;
      case RIGHT: neighborX = east_x; neighborY = east_y; break;
      case BACKWARD: neighborX = south_x; neighborY = south_y; break;
      case LEFT: neighborX = west_x; neighborY = west_y; break;
    }
    if (neighborX >= 0 && isAccessible(current_x, current_y, neighborX, neighborY)) {
      accessibleNeighbors[i] = floodArray[neighborX][neighborY];
      accessiblePathsNum++;
    }
  }

  Serial.print("whereToMove at (" + String(current_x) + "," + String(current_y) + "), orient=" + String(orient) + ": ");
  Serial.print("Accessible Paths="); Serial.print(accessiblePathsNum);
  Serial.print(", Neighbors (F,R,B,L)=[");
  Serial.print(accessibleNeighbors[FORWARD]); Serial.print(",");
  Serial.print(accessibleNeighbors[RIGHT]); Serial.print(",");
  Serial.print(accessibleNeighbors[BACKWARD]); Serial.print(",");
  Serial.print(accessibleNeighbors[LEFT]); Serial.print("]");
  Serial.print(", Previous="); Serial.println(previous);

  if (accessiblePathsNum == 1) {
    for (int i = 0; i < 4; i++) {
      if (accessibleNeighbors[i] != 1000) {
        minCell = i;
        Serial.println("Dead-end: Choosing direction " + String(minCell));
        break;
      }
    }
  } else {
    int directions[] = {LEFT, FORWARD, RIGHT, BACKWARD};
    for (int i = 0; i < 4; i++) {
      int dir = directions[i];
      if (accessibleNeighbors[dir] != 1000 && accessibleNeighbors[dir] < minValue) {
        minValue = accessibleNeighbors[dir];
        minCell = dir;
      }
    }
    if (accessiblePathsNum > 1) {
      for (int i = 0; i < 4; i++) {
        int dir = directions[i];
        if (accessibleNeighbors[dir] == minValue && dir != previous && dir == LEFT) {
          minCell = dir;
          Serial.println("Multiple min values, prioritizing LEFT, minCell=" + String(minCell));
          break;
        }
      }
    }
    if (accessiblePathsNum > 1 && minCell == previous) {
      for (int i = 0; i < 4; i++) {
        int dir = directions[i];
        if (accessibleNeighbors[dir] == minValue && dir != previous) {
          minCell = dir;
          Serial.println("Avoiding previous direction, new minCell=" + String(minCell));
          break;
        }
      }
    }
  }

  if (minCell == -1) {
    Serial.println("No accessible neighbors! Defaulting to back.");
    return 'B';
  }
  Serial.print("Chosen minCell: "); Serial.println(minCell);
  if (minCell == orient) return 'F';
  if (minCell == (orient + 3) % 4) return 'L';
  if (minCell == (orient + 1) % 4) return 'R';
  return 'B';
}

void logArrays() {
  Serial.println("Flood Array:");
  for (int i = 6; i >= 0; i--) {
    String row = "";
    for (int j = 0; j < 7; j++) {
      row += String(floodArray[j][i]) + " ";
    }
    Serial.println(row);
  }
  Serial.println("Cells Array:");
  for (int i = 6; i >= 0; i--) {
    String row = "";
    for (int j = 0; j < 7; j++) {
      row += String(cellsArray[j][i]) + " ";
    }
    Serial.println(row);
  }
}

int updateOrientation(int current_orient, char direction) {
  if (direction == 'L') return (current_orient + 3) % 4;
  if (direction == 'R') return (current_orient + 1) % 4;
  return current_orient;
}

void updateCoordinates(int orient, int *x, int *y) {
  if (orient == FORWARD) *y += 1;
  else if (orient == RIGHT) *x += 1;
  else if (orient == BACKWARD) *y -= 1;
  else if (orient == LEFT) *x -= 1;
}

void displayFlood() {
  print_values();
}

void retrieveIMUData() {
  Wire.beginTransmission(IMU_DEVICE_ADDR);
  Wire.write(IMU_ACCEL_X_H);
  Wire.endTransmission(false);
  Wire.requestFrom(IMU_DEVICE_ADDR, 14);

  int16_t raw_accel_x = (Wire.read() << 8) | Wire.read();
  int16_t raw_accel_y = (Wire.read() << 8) | Wire.read();
  int16_t raw_accel_z = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();
  int16_t raw_gyro_x = (Wire.read() << 8) | Wire.read();
  int16_t raw_gyro_y = (Wire.read() << 8) | Wire.read();
  int16_t raw_gyro_z = (Wire.read() << 8) | Wire.read();

  accel_x_data = raw_accel_x / IMU_ACCEL_SENS;
  accel_y_data = raw_accel_y / IMU_ACCEL_SENS;
  accel_z_data = raw_accel_z / IMU_ACCEL_SENS;
  gyro_x_rate = raw_gyro_x / IMU_GYRO_SENS;
  gyro_y_rate = raw_gyro_y / IMU_GYRO_SENS;
  gyro_z_rate = (raw_gyro_z / IMU_GYRO_SENS) - gyro_z_bias;

  filtered_gyro_z = 0.9 * filtered_gyro_z + 0.1 * gyro_z_rate;
  gyro_z_rate = filtered_gyro_z;
}

void calculateAngles() {
  roll_angle = atan2(accel_y_data, accel_z_data) * 180.0 / PI;
  pitch_angle = atan2(-accel_x_data, sqrt(accel_y_data * accel_y_data + accel_z_data * accel_z_data)) * 180.0 / PI;

  float current_time = millis() / 1000.0;
  float dt = current_time - last_time;
  last_time = current_time;

  yaw_angle = last_yaw + (gyro_z_rate * dt);
  last_yaw = yaw_angle;
}

void calibrateGyro() {
  long sum = 0;
  const int samples = 500; // Increased samples for better accuracy
  Serial.println("Calibrating gyro... Keep IMU still");

  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(IMU_DEVICE_ADDR);
    Wire.write(IMU_GYRO_X_H + 4);
    Wire.endTransmission(false);
    Wire.requestFrom(IMU_DEVICE_ADDR, 2);
    int16_t raw_gyro_z = (Wire.read() << 8) | Wire.read();
    sum += raw_gyro_z;
    delay(5);
  }

  gyro_z_bias = (sum / (float)samples) / IMU_GYRO_SENS;
  Serial.print("Gyro Z Bias: "); Serial.println(gyro_z_bias, 5);
}

bool handleSpinRecovery() {
  if (turnCount >= 4 && (millis() - lastTurnTime) < 10000) {
    Serial.println("Spin detected: turnCount=" + String(turnCount) + ", initiating recovery");
    if (turnLeft()) {
      orient = updateOrientation(orient, 'L');
      delay(1000);
      if (turnLeft()) {
        orient = updateOrientation(orient, 'L');
        Serial.println("Recovery: 180-degree turn complete, Orientation=" + String(orient));
        while (!moveForward()) {
        }
        previous_x = current_x;
        previous_y = current_y;
        int new_x = current_x, new_y = current_y;
        updateCoordinates(orient, &new_x, &new_y);
        if (new_x >= 0 && new_x < 7 && new_y >= 0 && new_y < 7) {
          current_x = new_x;
          current_y = new_y;
          Serial.println("Recovery: Moved to (" + String(current_x) + "," + String(current_y) + ")");
        } else {
          Serial.println("Recovery: Out of bounds at (" + String(new_x) + "," + String(new_y) + ")");
          currentState = STOPPED;
          return false;
        }
        turnCount = 0;
        lastTurnTime = 0;
        return true;
      }
    }
    Serial.println("Recovery failed, stopping");
    currentState = STOPPED;
    return false;
  }
  return false;
}

void loop() {
  retrieveIMUData();
  calculateAngles();

  bool left = false;
  bool right = false;
  bool forward = false;

  switch (currentState) {
    case IDLE: {
      if (goal_reached) {
        Serial.println("Goal reached, stopping");
        currentState = STOPPED;
        break;
      }
      read_ir_sensors();
      left = readLeftSensor();
      right = readRightSensor();
      forward = readFrontSensor();
      Serial.println("IDLE: left=" + String(left) + ", right=" + String(right) + ", forward=" + String(forward));
      updateCells(current_x, current_y, orient, left, right, forward);
      Serial.println("cellsArray[" + String(current_x) + "," + String(current_y) + "]=" + String(cellsArray[current_x][current_y]));

      if (handleSpinRecovery()) {
        break;
      }

      int wallCount = (left ? 1 : 0) + (right ? 1 : 0) + (forward ? 1 : 0);
      if (wallCount == 3) {
        Serial.println("Dead-end detected, forcing back turn");
        pendingDirection = 'B';
        currentState = TURNING_BACK;
        movementComplete = false;
        previous_x = current_x;
        previous_y = current_y;
        break;
      }

      if (floodArray[current_x][current_y] == 0) {
        Serial.println("Goal reached at (" + String(current_x) + "," + String(current_y) + ")");
        goal_reached = true;
        sM();
        currentState = STOPPED;
        break;
      }

      floodFillUsingQueue(current_x, current_y, previous_x, previous_y);
      logArrays();
      pendingDirection = whereToMove(current_x, current_y, previous_x, previous_y, orient);
      Serial.println("Direction=" + String(pendingDirection));

      if (pendingDirection == 'F') {
        currentState = MOVING_FORWARD;
      } else if (pendingDirection == 'L') {
        currentState = TURNING_LEFT;
      } else if (pendingDirection == 'R') {
        currentState = TURNING_RIGHT;
      } else if (pendingDirection == 'B') {
        currentState = TURNING_BACK;
      } else {
        Serial.println("Invalid direction, stopping");
        currentState = STOPPED;
        break;
      }
      movementComplete = false;
      break;
    }

    case MOVING_FORWARD:
      if (moveForward()) {
        movementComplete = true;
        if (isStopped) {
          currentState = STOPPED;
        } else {
          previous_x = current_x;
          previous_y = current_y;
          int new_x = current_x, new_y = current_y;
          updateCoordinates(orient, &new_x, &new_y);
          if (new_x >= 0 && new_x < 7 && new_y >= 0 && new_y < 7) {
            current_x = new_x;
            current_y = new_y;
            Serial.println("At (" + String(current_x) + "," + String(current_y) + ")");
          } else {
            Serial.println("Out of bounds: (" + String(new_x) + "," + String(new_y) + ")");
            currentState = STOPPED;
          }
          currentState = IDLE;
        }
      }
      break;

    case TURNING_LEFT:
      if (turnLeft()) {
        orient = updateOrientation(orient, 'L');
        turnCount++;
        lastTurnTime = millis();
        Serial.println("Orientation=" + String(orient) + ", turnCount=" + String(turnCount));
        movementComplete = true;
        currentState = IDLE;
      }
      break;

    case TURNING_RIGHT:
      if (turnRight()) {
        orient = updateOrientation(orient, 'R');
        turnCount++;
        lastTurnTime = millis();
        Serial.println("Orientation=" + String(orient) + ", turnCount=" + String(turnCount));
        movementComplete = true;
        currentState = IDLE;
      }
      break;

    case TURNING_BACK: {
      static int backTurnCount = 0;
      static unsigned long startTime = 0;

      if (startTime == 0) {
        startTime = millis();
        last_yaw = 0;
        yaw_angle = 0;
      }

      if (millis() - startTime > 10000) {
        Serial.println("Back turn timeout, stopping");
        backTurnCount = 0;
        startTime = 0;
        currentState = STOPPED;
        break;
      }

      if (turnLeft()) {
        orient = updateOrientation(orient, 'L');
        backTurnCount++;
        turnCount++;
        lastTurnTime = millis();

        Serial.println("Back turn step " + String(backTurnCount));

        if (backTurnCount >= 2) {
          Serial.println("Back turn complete, Orientation=" + String(orient));
          delay(500);
          while (!moveForward()) {
            delay(10);
          }

          previous_x = current_x;
          previous_y = current_y;
          int new_x = current_x, new_y = current_y;
          updateCoordinates(orient, &new_x, &new_y);
          if (new_x >= 0 && new_x < 7 && new_y >= 0 && new_y < 7) {
            current_x = new_x;
            current_y = new_y;
            Serial.println("BackTurn: Moved to (" + String(current_x) + "," + String(current_y) + ")");
          } else {
            Serial.println("BackTurn: Out of bounds! Stopping.");
            currentState = STOPPED;
            break;
          }

          backTurnCount = 0;
          startTime = 0;
          movementComplete = true;
          currentState = IDLE;
        } else {
          delay(1000);
        }
      }
      break;
    }

    case STOPPED:
      sM();
      break;
  }

  print_values();
  delay(10);
}