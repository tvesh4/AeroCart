/*
 * AutoCart - Autonomous Passenger Following Luggage System
 * Arduino Mega 2560 Main Code
 * 
 * Integrated with:
 * - 4x RFID Readers (RC522)
 * - 4x Ultrasonic Sensors (HC-SR04) 
 * - 4x Motors (Mecanum Wheel Drive)
 * - OLED Display (SSD1306)
 * - MPU6050 Gyroscope
 * - Jetson Nano Communication via Serial
 * 
 * Author: Group F2
 * Date: March 2026
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050_light.h>
#include <MFRC522.h>

// ==================== OLED DISPLAY ====================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET 28
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ==================== GYROSCOPE ====================
MPU6050 mpu(Wire);
float target_heading = 0;
float current_heading = 0;
float heading_error = 0;

// ==================== RFID READERS (4x RC522) ====================
// Reader 1 - Front Left
#define RST_PIN_1   22
#define SS_PIN_1    53

// Reader 2 - Front Right  
#define RST_PIN_2   24
#define SS_PIN_2    52

// Reader 3 - Rear Left
#define RST_PIN_3   26
#define SS_PIN_3    51

// Reader 4 - Rear Right
#define RST_PIN_4   28
#define SS_PIN_4    50

MFRC522 mfrc522_1(SS_PIN_1, RST_PIN_1);
MFRC522 mfrc522_2(SS_PIN_2, RST_PIN_2);
MFRC522 mfrc522_3(SS_PIN_3, RST_PIN_3);
MFRC522 mfrc522_4(SS_PIN_4, RST_PIN_4);
MFRC522* readers[4] = {&mfrc522_1, &mfrc522_2, &mfrc522_3, &mfrc522_4};

// ==================== MOTOR DRIVER (L298N) ====================
// Motor A (Front Left)
#define PWMA 12
#define DIRA1 34
#define DIRA2 35

// Motor B (Front Right)
#define PWMB 8
#define DIRB1 37
#define DIRB2 36

// Motor C (Rear Left)
#define PWMC 9
#define DIRC1 43
#define DIRC2 42

// Motor D (Rear Right)
#define PWMD 5
#define DIRD1 A4  // 26
#define DIRD2 A5  // 27

// Motor Macros for Mecanum Wheel Drive
#define MOTORA_FORWARD(pwm)    do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,HIGH); analogWrite(PWMA,pwm);}while(0)
#define MOTORA_BACKOFF(pwm)    do{digitalWrite(DIRA1,HIGH); digitalWrite(DIRA2,LOW); analogWrite(PWMA,pwm);}while(0)
#define MOTORA_STOP(x)         do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,LOW); analogWrite(PWMA,0);}while(0)

#define MOTORB_FORWARD(pwm)    do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,HIGH); analogWrite(PWMB,pwm);}while(0)
#define MOTORB_BACKOFF(pwm)    do{digitalWrite(DIRB1,HIGH); digitalWrite(DIRB2,LOW); analogWrite(PWMB,pwm);}while(0)
#define MOTORB_STOP(x)         do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,LOW); analogWrite(PWMB,0);}while(0)

#define MOTORC_FORWARD(pwm)    do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,HIGH); analogWrite(PWMC,pwm);}while(0)
#define MOTORC_BACKOFF(pwm)    do{digitalWrite(DIRC1,HIGH); digitalWrite(DIRC2,LOW); analogWrite(PWMC,pwm);}while(0)
#define MOTORC_STOP(x)         do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,LOW); analogWrite(PWMC,0);}while(0)

#define MOTORD_FORWARD(pwm)    do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,HIGH); analogWrite(PWMD,pwm);}while(0)
#define MOTORD_BACKOFF(pwm)    do{digitalWrite(DIRD1,HIGH); digitalWrite(DIRD2,LOW); analogWrite(PWMD,pwm);}while(0)
#define MOTORD_STOP(x)         do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,LOW); analogWrite(PWMD,0);}while(0)

// ==================== ULTRASONIC SENSORS ====================
// Front Left Ultrasonic
#define trigPin_Lfront 62
#define echoPin_Lfront 61

// Front Right Ultrasonic
#define trigPin_Rfront 64
#define echoPin_Rfront 63

// Left Ultrasonic
#define trigPin_left 66
#define echoPin_left 65

// Right Ultrasonic
#define trigPin_right 68
#define echoPin_right 67

// ==================== LED INDICATORS ====================
#define LED_BLUE 11
#define LED_GREEN 13
#define LED_RED 14
#define BUZZER_PIN 15

// ==================== COLOR SENSOR (TCS3200) ====================
#define COLOR_OUT 25
#define COLOR_S2 28
#define COLOR_S3 29
#define COLOR_S0 30
#define COLOR_S1 32

// ==================== CONFIGURATION ====================
#define MAX_SPEED 200
#define MIN_SPEED 80
#define OBSTACLE_THRESHOLD 30.0  // cm
#define TARGET_FOLLOW_DISTANCE 100.0  // cm
#define TAG_LOST_TIMEOUT 10000  // 10 seconds

// ==================== CALIBRATION STRUCTURE ====================
struct SensorCalibration {
  float offset;
  float scale;
  bool isCalibrated;
};

SensorCalibration cal_Lfront = {0, 1.0, false};
SensorCalibration cal_Rfront = {0, 1.0, false};
SensorCalibration cal_left = {0, 1.0, false};
SensorCalibration cal_right = {0, 1.0, false};

// ==================== STATE MACHINE ====================
enum RobotState {
  STATE_SEARCHING,      // S1: Looking for RFID tag
  STATE_FOLLOWING,      // S2: Following passenger
  STATE_OBSTACLE,       // S3: Obstacle detected
  STATE_HOVER,          // S4: Hover mode - passenger stopped
  STATE_MANUAL,         // Manual control mode
  STATE_ERROR           // Error state
};

RobotState currentState = STATE_SEARCHING;
RobotState previousState = STATE_SEARCHING;

// ==================== GLOBAL VARIABLES ====================
// RFID Data
byte targetTagUID[4] = {0x00, 0x00, 0x00, 0x00};
bool tagFound = false;
unsigned long lastTagReadTime = 0;
unsigned long tagLostStartTime = 0;
int rssiValues[4] = {0, 0, 0, 0};

// Ultrasonic distances
float distances[4] = {0, 0, 0, 0};  // 0:Front, 1:Left, 2:Right, 3:Rear

// Motor control
int motorSpeed = 150;
int turnSpeed = 120;

// Timing
unsigned long lastSensorReadTime = 0;
unsigned long lastOLEDUpdateTime = 0;
unsigned long lastStateChangeTime = 0;
const unsigned long SENSOR_READ_INTERVAL = 50;
const unsigned long OLED_UPDATE_INTERVAL = 200;

// Jetson Nano Communication
String jetsonCommand = "";
bool faceDetectionActive = false;

// ==================== MOTOR CONTROL FUNCTIONS ====================

// Forward movement (all motors forward)
void moveForward(int speed) {
  speed = constrain(speed, 0, MAX_SPEED);
  MOTORA_FORWARD(speed);
  MOTORB_FORWARD(speed);
  MOTORC_FORWARD(speed);
  MOTORD_FORWARD(speed);
}

// Backward movement
void moveBackward(int speed) {
  speed = constrain(speed, 0, MAX_SPEED);
  MOTORA_BACKOFF(speed);
  MOTORB_BACKOFF(speed);
  MOTORC_BACKOFF(speed);
  MOTORD_BACKOFF(speed);
}

// Strafe Left
void strafeLeft(int speed) {
  speed = constrain(speed, 0, MAX_SPEED);
  MOTORA_BACKOFF(speed);
  MOTORB_FORWARD(speed);
  MOTORC_BACKOFF(speed);
  MOTORD_FORWARD(speed);
}

// Strafe Right
void strafeRight(int speed) {
  speed = constrain(speed, 0, MAX_SPEED);
  MOTORA_FORWARD(speed);
  MOTORB_BACKOFF(speed);
  MOTORC_FORWARD(speed);
  MOTORD_BACKOFF(speed);
}

// Turn Left (rotate counter-clockwise)
void turnLeft(int speed) {
  speed = constrain(speed, 0, MAX_SPEED);
  MOTORA_BACKOFF(speed);
  MOTORB_FORWARD(speed);
  MOTORC_BACKOFF(speed);
  MOTORD_FORWARD(speed);
}

// Turn Right (rotate clockwise)
void turnRight(int speed) {
  speed = constrain(speed, 0, MAX_SPEED);
  MOTORA_FORWARD(speed);
  MOTORB_BACKOFF(speed);
  MOTORC_FORWARD(speed);
  MOTORD_BACKOFF(speed);
}

// Rotate in place
void rotateInPlace(int direction, int speed) {
  speed = constrain(speed, 0, MAX_SPEED);
  if (direction > 0) {
    // Clockwise rotation
    MOTORA_FORWARD(speed);
    MOTORB_BACKOFF(speed);
    MOTORC_FORWARD(speed);
    MOTORD_BACKOFF(speed);
  } else {
    // Counter-clockwise rotation
    MOTORA_BACKOFF(speed);
    MOTORB_FORWARD(speed);
    MOTORC_BACKOFF(speed);
    MOTORD_FORWARD(speed);
  }
}

// Stop all motors
void stopMotors() {
  MOTORA_STOP(0);
  MOTORB_STOP(0);
  MOTORC_STOP(0);
  MOTORD_STOP(0);
}

// ==================== ULTRASONIC FUNCTIONS ====================

float measureDistance(int trigPin, int echoPin, SensorCalibration &cal) {
  float duration, distance_raw, distance_calibrated;
  
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  duration = pulseIn(echoPin, HIGH, 30000);
  
  if (duration == 0) {
    return 999.0;
  }
  
  distance_raw = (duration * 0.0343) / 2;
  distance_calibrated = (distance_raw * cal.scale) + cal.offset;
  
  return distance_calibrated;
}

void readAllUltrasonicSensors() {
  distances[0] = measureDistance(trigPin_Lfront, echoPin_Lfront, cal_Lfront);
  distances[1] = measureDistance(trigPin_left, echoPin_left, cal_left);
  distances[2] = measureDistance(trigPin_right, echoPin_right, cal_right);
  distances[3] = measureDistance(trigPin_Rfront, echoPin_Rfront, cal_Rfront);
}

bool checkObstacles() {
  // Check front obstacle
  if (distances[0] < OBSTACLE_THRESHOLD && distances[0] > 0) {
    return true;
  }
  return false;
}

// ==================== RFID FUNCTIONS ====================

void initRFIDReaders() {
  for (int i = 0; i < 4; i++) {
    readers[i]->PCD_Init();
    delay(10);
  }
}

void readAllRFIDReaders() {
  tagFound = false;
  
  for (int i = 0; i < 4; i++) {
    if (readRFIDReader(readers[i], i)) {
      tagFound = true;
    }
  }
  
  if (tagFound) {
    lastTagReadTime = millis();
    tagLostStartTime = 0;
  } else if (tagLostStartTime == 0 && (currentState == STATE_FOLLOWING || currentState == STATE_HOVER)) {
    tagLostStartTime = millis();
    beepDouble();
  }
}

bool readRFIDReader(MFRC522* reader, int readerIndex) {
  if (!reader->PICC_IsNewCardPresent()) {
    rssiValues[readerIndex] = 0;
    return false;
  }
  
  if (!reader->PICC_ReadCardSerial()) {
    rssiValues[readerIndex] = 0;
    return false;
  }
  
  bool isTarget = true;
  for (byte i = 0; i < 4; i++) {
    if (reader->uid.uidByte[i] != targetTagUID[i]) {
      isTarget = false;
      break;
    }
  }
  
  if (isTarget) {
    int signalStrength = reader->uid.size * 25;
    rssiValues[readerIndex] = constrain(signalStrength, 0, 100);
    reader->PICC_HaltA();
    return true;
  }
  
  reader->PICC_HaltA();
  return false;
}

void setTargetTag(byte* uid) {
  for (int i = 0; i < 4; i++) {
    targetTagUID[i] = uid[i];
  }
}

// ==================== FOLLOW LOGIC ====================

void followTag() {
  if (!tagFound) return;
  
  // Calculate direction from RSSI
  int frontAvg = (rssiValues[0] + rssiValues[1]) / 2;
  int rearAvg = (rssiValues[2] + rssiValues[3]) / 2;
  int leftAvg = (rssiValues[0] + rssiValues[2]) / 2;
  int rightAvg = (rssiValues[1] + rssiValues[3]) / 2;
  
  // Estimate distance from tag
  int maxRSSI = max(max(rssiValues[0], rssiValues[1]), max(rssiValues[2], rssiValues[3]));
  float estimatedDistance = map(maxRSSI, 20, 80, 150, 30);
  estimatedDistance = constrain(estimatedDistance, 30, 150);
  
  // Check if passenger is very close (enter hover)
  int strongCount = 0;
  for (int i = 0; i < 4; i++) {
    if (rssiValues[i] > 60) strongCount++;
  }
  
  if (strongCount >= 3 || distances[0] < 20) {
    currentState = STATE_HOVER;
    stopMotors();
    return;
  }
  
  // Follow logic
  if (estimatedDistance > TARGET_FOLLOW_DISTANCE + 20) {
    // Too far - move forward
    int speed = map(estimatedDistance, TARGET_FOLLOW_DISTANCE, 200, MIN_SPEED, MAX_SPEED);
    speed = constrain(speed, MIN_SPEED, MAX_SPEED);
    
    int turnAdjust = rightAvg - leftAvg;
    
    if (abs(turnAdjust) > 20) {
      if (turnAdjust < 0) {
        strafeLeft(turnSpeed);
      } else {
        strafeRight(turnSpeed);
      }
    } else {
      moveForward(speed);
    }
  } 
  else if (estimatedDistance < TARGET_FOLLOW_DISTANCE - 20) {
    // Too close - stop or move back
    if (estimatedDistance < 50) {
      stopMotors();
    } else {
      moveBackward(MIN_SPEED);
    }
  } 
  else {
    // Within range - maintain position
    int turnAdjust = rightAvg - leftAvg;
    if (abs(turnAdjust) > 15) {
      if (turnAdjust < 0) {
        strafeLeft(turnSpeed / 2);
      } else {
        strafeRight(turnSpeed / 2);
      }
    } else {
      stopMotors();
    }
  }
}

void searchForTag() {
  if (tagFound) {
    currentState = STATE_FOLLOWING;
    beep(50);
    return;
  }
  
  // Rotate slowly to scan for tag
  static unsigned long lastRotationTime = 0;
  static bool rotatingCW = true;
  
  if (millis() - lastRotationTime > 30) {
    if (rotatingCW) {
      rotateInPlace(1, 80);
    } else {
      rotateInPlace(-1, 80);
    }
    lastRotationTime = millis();
  }
  
  // Change direction every 2 seconds
  if (millis() - lastRotationTime > 2000) {
    rotatingCW = !rotatingCW;
  }
  
  // Check timeout
  if (tagLostStartTime > 0 && (millis() - tagLostStartTime) > TAG_LOST_TIMEOUT) {
    currentState = STATE_ERROR;
  }
}

void handleObstacle() {
  stopMotors();
  
  // Evasive action
  if (distances[1] > OBSTACLE_THRESHOLD) {
    strafeLeft(turnSpeed);
    delay(500);
    stopMotors();
  } 
  else if (distances[2] > OBSTACLE_THRESHOLD) {
    strafeRight(turnSpeed);
    delay(500);
    stopMotors();
  }
  else if (distances[3] > OBSTACLE_THRESHOLD) {
    moveBackward(100);
    delay(300);
    stopMotors();
  }
  else {
    beepContinuous();
  }
}

// ==================== LED AND BUZZER ====================

void setLEDState(RobotState state) {
  switch (state) {
    case STATE_SEARCHING:
      digitalWrite(LED_BLUE, HIGH);
      digitalWrite(LED_GREEN, LOW);
      digitalWrite(LED_RED, LOW);
      break;
    case STATE_FOLLOWING:
      digitalWrite(LED_GREEN, HIGH);
      digitalWrite(LED_BLUE, LOW);
      digitalWrite(LED_RED, LOW);
      break;
    case STATE_OBSTACLE:
      digitalWrite(LED_RED, HIGH);
      digitalWrite(LED_GREEN, LOW);
      digitalWrite(LED_BLUE, LOW);
      break;
    case STATE_HOVER:
      digitalWrite(LED_GREEN, HIGH);
      digitalWrite(LED_BLUE, LOW);
      digitalWrite(LED_RED, LOW);
      break;
    case STATE_ERROR:
      digitalWrite(LED_RED, HIGH);
      digitalWrite(LED_GREEN, LOW);
      digitalWrite(LED_BLUE, LOW);
      break;
    default:
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_GREEN, LOW);
      digitalWrite(LED_BLUE, LOW);
  }
}

void beep(int duration) {
  tone(BUZZER_PIN, 2000, duration);
  delay(duration);
  noTone(BUZZER_PIN);
}

void beepDouble() {
  beep(50);
  delay(100);
  beep(50);
}

void beepContinuous() {
  tone(BUZZER_PIN, 2000);
  delay(500);
  noTone(BUZZER_PIN);
}

// ==================== OLED DISPLAY ====================

void updateOLED() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  
  // State
  display.print("State: ");
  switch(currentState) {
    case STATE_SEARCHING: display.print("SEARCHING"); break;
    case STATE_FOLLOWING: display.print("FOLLOWING"); break;
    case STATE_OBSTACLE: display.print("OBSTACLE"); break;
    case STATE_HOVER: display.print("HOVER"); break;
    case STATE_ERROR: display.print("ERROR"); break;
    default: display.print("?");
  }
  
  // Tag status
  display.print(" Tag: ");
  display.println(tagFound ? "YES" : "NO");
  
  // Front distance
  display.print("F:");
  if (distances[0] < 500) display.print((int)distances[0]);
  else display.print("---");
  display.print("cm L:");
  if (distances[1] < 500) display.print((int)distances[1]);
  else display.print("---");
  display.print("cm");
  
  display.setCursor(0, 24);
  display.print("R:");
  if (distances[2] < 500) display.print((int)distances[2]);
  else display.print("---");
  display.print("cm B:");
  if (distances[3] < 500) display.print((int)distances[3]);
  else display.print("---");
  display.print("cm");
  
  display.display();
}

// ==================== JETSON NANO COMMUNICATION ====================

void processJetsonCommands() {
  if (Serial1.available()) {
    String command = Serial1.readStringUntil('\n');
    command.trim();
    
    if (command.startsWith("FACE:")) {
      faceDetectionActive = true;
    }
    else if (command.startsWith("TAG:")) {
      byte uid[4];
      String tagStr = command.substring(4);
      int idx = 0;
      char* p = strtok((char*)tagStr.c_str(), " ");
      while (p != NULL && idx < 4) {
        uid[idx++] = strtol(p, NULL, 16);
        p = strtok(NULL, " ");
      }
      setTargetTag(uid);
      Serial.println("Target tag set");
    }
    else if (command.startsWith("MODE:")) {
      String mode = command.substring(5);
      if (mode == "MANUAL") {
        currentState = STATE_MANUAL;
      } else if (mode == "AUTO") {
        currentState = STATE_SEARCHING;
      }
    }
    else if (command == "STOP") {
      stopMotors();
    }
  }
}

void sendToJetson() {
  Serial1.print("DATA:");
  Serial1.print(distances[0]);
  Serial1.print(",");
  Serial1.print(rssiValues[0]);
  Serial1.print(",");
  Serial1.print(rssiValues[1]);
  Serial1.print(",");
  Serial1.print(rssiValues[2]);
  Serial1.print(",");
  Serial1.print(rssiValues[3]);
  Serial1.print(",");
  Serial1.print(currentState);
  Serial1.print(",");
  Serial1.println(tagFound ? "1" : "0");
}

// ==================== SETUP ====================

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  
  // Initialize I2C
  Wire.begin();
  
  // Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED allocation failed");
  }
  
  // Initialize MPU6050
  mpu.begin();
  mpu.calcGyroOffsets();
  
  // Initialize RFID
  SPI.begin();
  initRFIDReaders();
  
  // Initialize Ultrasonic pins
  pinMode(trigPin_Lfront, OUTPUT);
  pinMode(echoPin_Lfront, INPUT);
  pinMode(trigPin_Rfront, OUTPUT);
  pinMode(echoPin_Rfront, INPUT);
  pinMode(trigPin_left, OUTPUT);
  pinMode(echoPin_left, INPUT);
  pinMode(trigPin_right, OUTPUT);
  pinMode(echoPin_right, INPUT);
  
  // Initialize Motor pins
  pinMode(PWMA, OUTPUT);
  pinMode(DIRA1, OUTPUT);
  pinMode(DIRA2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(DIRB1, OUTPUT);
  pinMode(DIRB2, OUTPUT);
  pinMode(PWMC, OUTPUT);
  pinMode(DIRC1, OUTPUT);
  pinMode(DIRC2, OUTPUT);
  pinMode(PWMD, OUTPUT);
  pinMode(DIRD1, OUTPUT);
  pinMode(DIRD2, OUTPUT);
  
  // Initialize LED and Buzzer
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  // Set default calibration
  cal_Lfront = {0, 1.0, true};
  cal_Rfront = {0, 1.0, true};
  cal_left = {0, 1.0, true};
  cal_right = {0, 1.0, true};
  
  // Display startup message
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println("AutoCart");
  display.setTextSize(1);
  display.println("Ready");
  display.display();
  
  beep(100);
  delay(500);
  beep(100);
  
  Serial.println("AutoCart System Initialized");
}

// ==================== MAIN LOOP ====================

void loop() {
  unsigned long currentTime = millis();
  
  // Read sensors at intervals
  if (currentTime - lastSensorReadTime >= SENSOR_READ_INTERVAL) {
    readAllUltrasonicSensors();
    readAllRFIDReaders();
    lastSensorReadTime = currentTime;
  }
  
  // Process Jetson commands
  processJetsonCommands();
  
  // Check for obstacles
  bool obstacleDetected = checkObstacles();
  
  // Update state machine
  previousState = currentState;
  
  if (obstacleDetected && currentState != STATE_OBSTACLE) {
    currentState = STATE_OBSTACLE;
  } else if (!obstacleDetected && currentState == STATE_OBSTACLE) {
    currentState = tagFound ? STATE_FOLLOWING : STATE_SEARCHING;
  }
  
  // State transitions
  if (currentState == STATE_SEARCHING && tagFound) {
    currentState = STATE_FOLLOWING;
  }
  if ((currentState == STATE_FOLLOWING || currentState == STATE_HOVER) && !tagFound) {
    currentState = STATE_SEARCHING;
  }
  
  // Execute state actions
  switch (currentState) {
    case STATE_SEARCHING:
      searchForTag();
      break;
    case STATE_FOLLOWING:
      followTag();
      break;
    case STATE_OBSTACLE:
      handleObstacle();
      break;
    case STATE_HOVER:
      stopMotors();
      break;
    case STATE_ERROR:
      stopMotors();
      beepContinuous();
      break;
    default:
      break;
  }
  
  // Update LEDs
  setLEDState(currentState);
  
  // Update OLED display
  if (currentTime - lastOLEDUpdateTime >= OLED_UPDATE_INTERVAL) {
    updateOLED();
    lastOLEDUpdateTime = currentTime;
  }
  
  // Send data to Jetson
  sendToJetson();
  
  delay(10);
}
