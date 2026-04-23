#include <Arduino.h>
#include <Servo.h>
#include <NewPing.h>

// ====================== PIN DEFINITIONS ======================
#define PWMA  12
#define DIRA1 34
#define DIRA2 35
#define PWMB  8
#define DIRB1 37
#define DIRB2 36
#define PWMC  9
#define DIRC1 43
#define DIRC2 42
#define PWMD  5
#define DIRD1 A4
#define DIRD2 A5

#define IRpin1 40   // LEFT_IR  (obstacle in Mode 0 / human sensor in Mode 1)
#define IRpin2 47   // RIGHT_IR (obstacle in Mode 0 / human sensor in Mode 1)

#define EMERGENCY_BTN 28  // Emergency stop (INPUT_PULLUP, LOW when pressed)
#define MODE_BTN      25  // Mode toggle    (INPUT_PULLUP, LOW when pressed)

// ====================== IR LINE SENSORS (5-channel) ======================
#define ir1 A0  // Left Most
#define ir2 A2  // Left
#define ir3 A3  // Middle
#define ir4 A8  // Right
#define ir5 A9  // Right Most

#define LINE_THRESHOLD 500

// ====================== ULTRASONIC SENSOR ======================
#define TRIGGER_PIN  A6
#define ECHO_PIN     A7
#define MAX_DISTANCE 100

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// ====================== SERVO ======================
const int servoPin = 48;
Servo boxServo;
bool boxOpen = false;

// ====================== MOTOR SPEED ======================
int Motor_PWM = 350;

// ====================== UWB DISTANCE VARIABLES ======================
float currentDistance   = 0.0;
float targetDistance    = 0.0;
bool  targetDistanceSet = false;
bool  uwbDataValid      = false;
float stopTolerance     = 0.05;

// ====================== STATE ======================
String lastUserCommand = "stop";
bool   emergencyActive = false;

// ====================== MODE ======================
// 0 = Line Tracking + UWB
// 1 = Ultrasonic Human Following
int  currentMode      = 0;
bool lastModeBtnState = HIGH;

// ====================== MOTOR MACROS ======================
#define MOTORA_FORWARD(pwm)  do{digitalWrite(DIRA1,LOW);  digitalWrite(DIRA2,HIGH); analogWrite(PWMA,pwm);}while(0)
#define MOTORA_BACKOFF(pwm)  do{digitalWrite(DIRA1,HIGH); digitalWrite(DIRA2,LOW);  analogWrite(PWMA,pwm);}while(0)
#define MOTORA_STOP()        do{digitalWrite(DIRA1,HIGH); digitalWrite(DIRA2,HIGH); analogWrite(PWMA,0);  }while(0)

#define MOTORB_FORWARD(pwm)  do{digitalWrite(DIRB1,LOW);  digitalWrite(DIRB2,HIGH); analogWrite(PWMB,pwm);}while(0)
#define MOTORB_BACKOFF(pwm)  do{digitalWrite(DIRB1,HIGH); digitalWrite(DIRB2,LOW);  analogWrite(PWMB,pwm);}while(0)
#define MOTORB_STOP()        do{digitalWrite(DIRB1,HIGH); digitalWrite(DIRB2,HIGH); analogWrite(PWMB,0);  }while(0)

#define MOTORC_FORWARD(pwm)  do{digitalWrite(DIRC1,LOW);  digitalWrite(DIRC2,HIGH); analogWrite(PWMC,pwm);}while(0)
#define MOTORC_BACKOFF(pwm)  do{digitalWrite(DIRC1,HIGH); digitalWrite(DIRC2,LOW);  analogWrite(PWMC,pwm);}while(0)
#define MOTORC_STOP()        do{digitalWrite(DIRC1,HIGH); digitalWrite(DIRC2,HIGH); analogWrite(PWMC,0);  }while(0)

#define MOTORD_FORWARD(pwm)  do{digitalWrite(DIRD1,LOW);  digitalWrite(DIRD2,HIGH); analogWrite(PWMD,pwm);}while(0)
#define MOTORD_BACKOFF(pwm)  do{digitalWrite(DIRD1,HIGH); digitalWrite(DIRD2,LOW);  analogWrite(PWMD,pwm);}while(0)
#define MOTORD_STOP()        do{digitalWrite(DIRD1,HIGH); digitalWrite(DIRD2,HIGH); analogWrite(PWMD,0);  }while(0)

// ====================== EMERGENCY STOP ======================
void EMERGENCY_STOP() {
  MOTORA_STOP(); MOTORB_STOP(); MOTORC_STOP(); MOTORD_STOP();

  if (boxOpen) {
    for (int pos = 90; pos >= 0; pos -= 2) { boxServo.write(pos); delay(20); }
    boxOpen = false;
  }

  lastUserCommand   = "stop";
  targetDistanceSet = false;
  uwbDataValid      = false;
  Motor_PWM         = 450;
  emergencyActive   = true;

  Serial.println("!!! EMERGENCY STOP ACTIVATED !!!");
  Serial.println("Please Go Find Airport Staff");
}

// ====================== GENERAL MOVEMENT ======================
void ADVANCE() {
  MOTORA_FORWARD(Motor_PWM); MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM); MOTORD_BACKOFF(Motor_PWM);
}
void BACK() {
  lastUserCommand = "back";
  MOTORA_BACKOFF(Motor_PWM); MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM); MOTORD_FORWARD(Motor_PWM);
}
void FULL_STOP() {
  lastUserCommand = "stop";
  MOTORA_STOP(); MOTORB_STOP(); MOTORC_STOP(); MOTORD_STOP();
}
void TEMPORARY_STOP() {
  MOTORA_STOP(); MOTORB_STOP(); MOTORC_STOP(); MOTORD_STOP();
}
void ROTATE_LEFT() {
  lastUserCommand = "left";
  MOTORA_BACKOFF(Motor_PWM); MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM); MOTORD_FORWARD(Motor_PWM);
}
void ROTATE_RIGHT() {
  lastUserCommand = "right";
  MOTORA_FORWARD(Motor_PWM); MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM); MOTORD_BACKOFF(Motor_PWM);
}

// ====================== LINE TRACKING TURNS ======================
void trackTurnRight()     { MOTORA_FORWARD(Motor_PWM); MOTORB_FORWARD(Motor_PWM); MOTORC_STOP();              MOTORD_STOP();              }
void trackTurnRightHard() { MOTORA_FORWARD(Motor_PWM); MOTORB_FORWARD(Motor_PWM); MOTORC_FORWARD(Motor_PWM); MOTORD_FORWARD(Motor_PWM); }
void trackTurnLeft()      { MOTORA_BACKOFF(Motor_PWM); MOTORB_BACKOFF(Motor_PWM); MOTORC_STOP();              MOTORD_STOP();              }
void trackTurnLeftHard()  { MOTORA_BACKOFF(Motor_PWM); MOTORB_BACKOFF(Motor_PWM); MOTORC_BACKOFF(Motor_PWM); MOTORD_BACKOFF(Motor_PWM); }

// ====================== LINE TRACKING LOGIC ======================
void lineTrack() {
  int s1 = analogRead(ir1) < LINE_THRESHOLD ? 0 : 1;
  int s2 = analogRead(ir2) < LINE_THRESHOLD ? 0 : 1;
  int s3 = analogRead(ir3) < LINE_THRESHOLD ? 0 : 1;
  int s4 = analogRead(ir4) < LINE_THRESHOLD ? 0 : 1;
  int s5 = analogRead(ir5) < LINE_THRESHOLD ? 0 : 1;

  if      (s1==1 && s2==1 && s3==0 && s4==1 && s5==1) { ADVANCE(); }
  else if (s1==1 && s2==0 && s3==1 && s4==1 && s5==1) { trackTurnRight(); }
  else if (s1==0 && s2==1 && s3==1 && s4==1 && s5==1) { trackTurnRightHard(); }
  else if (s1==1 && s2==0 && s3==0 && s4==1 && s5==1) { trackTurnRight(); }
  else if (s1==0 && s2==0 && s3==0 && s4==1 && s5==1) { trackTurnRightHard(); }
  else if (s1==1 && s2==1 && s3==1 && s4==0 && s5==1) { trackTurnLeft(); }
  else if (s1==1 && s2==1 && s3==1 && s4==1 && s5==0) { trackTurnLeftHard(); }
  else if (s1==1 && s2==1 && s3==0 && s4==0 && s5==1) { trackTurnLeft(); }
  else if (s1==1 && s2==1 && s3==0 && s4==0 && s5==0) { trackTurnLeftHard(); }
  else if (s1==0 && s2==0 && s3==0 && s4==0 && s5==0) { ADVANCE(); }
  else if (s1==1 && s2==1 && s3==1 && s4==1 && s5==1) { ADVANCE(); }
}

// ====================== UWB DISTANCE MONITORING ======================
void checkUWBAndStop() {
  if (targetDistanceSet && uwbDataValid && (lastUserCommand == "forward")) {
    float remaining = targetDistance - currentDistance;
    if (remaining <= stopTolerance) {
      FULL_STOP();
      Serial.print("TARGET_REACHED: Distance ");
      Serial.print(currentDistance, 3);
      Serial.print("m (Target: ");
      Serial.print(targetDistance, 3);
      Serial.println("m)");
      targetDistanceSet = false;
    } else if (remaining <= 0.5 && remaining > 0) {
      int newSpeed = map(remaining * 100, 0, 50, 150, 300);
      Motor_PWM = constrain(newSpeed, 150, 300);
    } else {
      Motor_PWM = 350;
    }
  }
}

// ====================== SERVO ======================
void openBox() {
  if (boxOpen) return;
  Serial.println("Opening Box...");
  for (int pos = 0; pos <= 90; pos += 2) { boxServo.write(pos); delay(20); }
  boxOpen = true;
  Serial.println("Box Opened");
}
void closeBox() {
  if (!boxOpen) return;
  Serial.println("Closing Box...");
  for (int pos = 90; pos >= 0; pos -= 2) { boxServo.write(pos); delay(20); }
  boxOpen = false;
  Serial.println("Box Closed");
}

// ====================== OBSTACLE CHECK (Mode 0) ======================
bool obstacleDetected() {
  return (digitalRead(IRpin1) == 0) || (digitalRead(IRpin2) == 0);
}

// ====================== HUMAN FOLLOWING LOGIC (Mode 1) ======================
void humanFollowing() {
  unsigned int distance = sonar.ping_cm();
  int rightValue        = digitalRead(IRpin2);  // pin 47
  int leftValue         = digitalRead(IRpin1);  // pin 40

  Serial.print("[FOLLOW] Dist: "); Serial.print(distance);
  Serial.print(" cm | R: "); Serial.print(rightValue);
  Serial.print(" | L: "); Serial.println(leftValue);

  // Too close → back off
  if (distance > 1 && distance < 5) {
    Serial.println("STATUS: Too close — Moving BACKWARD");
    MOTORA_BACKOFF(300); MOTORB_FORWARD(300);
    MOTORC_BACKOFF(300); MOTORD_FORWARD(300);
    return;
  }

  // Both sensors + within range → forward
  if (rightValue == 1 && leftValue == 1 && distance >= 10 && distance <= 30) {
    Serial.println("STATUS: Person ahead — Moving FORWARD");
    ADVANCE();
    return;
  }

  // Person on right only → turn right
  if (rightValue == 0 && leftValue == 1) {
    Serial.println("STATUS: Person on RIGHT — Turning RIGHT");
    MOTORA_FORWARD(300); MOTORB_FORWARD(300);
    MOTORC_BACKOFF(300); MOTORD_BACKOFF(300);
    return;
  }

  // Person on left only → turn left
  if (rightValue == 1 && leftValue == 0) {
    Serial.println("STATUS: Person on LEFT — Turning LEFT");
    MOTORA_BACKOFF(300); MOTORB_BACKOFF(300);
    MOTORC_FORWARD(300); MOTORD_FORWARD(300);
    return;
  }

  // IR detects but too far → speed up
  if (rightValue == 1 && leftValue == 1 && distance > 30 && distance < MAX_DISTANCE) {
    Serial.println("STATUS: Person too far — Speeding up");
    MOTORA_FORWARD(400); MOTORB_BACKOFF(400);
    MOTORC_FORWARD(400); MOTORD_BACKOFF(400);
    return;
  }

  // Nothing → stop
  Serial.println("STATUS: No person — STOPPED");
  TEMPORARY_STOP();
}

// ====================== MODE TOGGLE ======================
void checkModeButton() {
  bool currentState = digitalRead(MODE_BTN);

  // Falling edge = button just pressed
  if (lastModeBtnState == HIGH && currentState == LOW) {
    delay(50); // debounce
    if (digitalRead(MODE_BTN) == LOW) {
      currentMode = (currentMode == 0) ? 1 : 0;
      FULL_STOP();
      lastUserCommand = "stop";

      if (currentMode == 0) {
        Serial.println("=== MODE: Line Tracking + UWB ===");
        Serial.println("MODE:0");
      } else {
        Serial.println("=== MODE: Human Following ===");
        Serial.println("MODE:1");
      }
    }
  }

  lastModeBtnState = currentState;
}

// ====================== SETUP ======================
void setup() {
  pinMode(PWMA, OUTPUT); pinMode(DIRA1, OUTPUT); pinMode(DIRA2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(DIRB1, OUTPUT); pinMode(DIRB2, OUTPUT);
  pinMode(PWMC, OUTPUT); pinMode(DIRC1, OUTPUT); pinMode(DIRC2, OUTPUT);
  pinMode(PWMD, OUTPUT); pinMode(DIRD1, OUTPUT); pinMode(DIRD2, OUTPUT);

  pinMode(IRpin1, INPUT_PULLUP);
  pinMode(IRpin2, INPUT_PULLUP);

  pinMode(ir1, INPUT); pinMode(ir2, INPUT); pinMode(ir3, INPUT);
  pinMode(ir4, INPUT); pinMode(ir5, INPUT);

  pinMode(EMERGENCY_BTN, INPUT_PULLUP);
  pinMode(MODE_BTN,      INPUT_PULLUP);

  boxServo.attach(servoPin);
  boxServo.write(0);

  FULL_STOP();

  Serial.begin(115200);
  Serial.println("=== Dual-Mode Robot Ready ===");
  Serial.println("PIN-25 button: toggle mode");
  Serial.println("Mode 0: Line Tracking + UWB");
  Serial.println("Mode 1: Human Following");
  Serial.println("Serial cmds (Mode 0): forward/f  back/b  left/l  right/r  stop/s  open  close  reset");
  Serial.println("UWB cmds: TARGET:<dist>  DIST:<current>");
  Serial.println("Emergency: Red button pin 28");
  Serial.println("==============================");
}

// ====================== MAIN LOOP ======================
void loop() {

  // ---- MODE BUTTON (checked every loop) ----
  checkModeButton();

  // ---- EMERGENCY BUTTON (highest priority) ----
  if (digitalRead(EMERGENCY_BTN) == LOW) {
    if (!emergencyActive) {
      EMERGENCY_STOP();
      delay(300);
    } else {
      emergencyActive = false;
      Serial.println("Emergency reset. System resumed.");
      delay(300);
    }
  }

  // ---- Block all motion during emergency ----
  if (emergencyActive) {
    MOTORA_STOP(); MOTORB_STOP(); MOTORC_STOP(); MOTORD_STOP();
    delay(30);
    return;
  }

  // ====================================================
  //  MODE 1 — Human Following
  // ====================================================
  if (currentMode == 1) {
    humanFollowing();
    delay(50);
    return;
  }

  // ====================================================
  //  MODE 0 — Line Tracking + UWB
  // ====================================================

  // Serial commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("TARGET:")) {
      float newTarget = cmd.substring(7).toFloat();
      if (newTarget > 0) {
        targetDistance    = newTarget;
        targetDistanceSet = true;
        Serial.print("Target distance set to: ");
        Serial.print(targetDistance);
        Serial.println(" meters");
      }
    }
    else if (cmd.startsWith("DIST:")) {
      float dist = cmd.substring(5).toFloat();
      if (dist >= 0) {
        currentDistance = dist;
        uwbDataValid    = true;
      }
    }
    else {
      cmd.toLowerCase();
      if      (cmd == "forward" || cmd == "f") { lastUserCommand = "forward"; Serial.println("Line Tracking FORWARD"); }
      else if (cmd == "back"    || cmd == "b") { BACK();         Serial.println("Moving BACKWARD"); }
      else if (cmd == "left"    || cmd == "l") { ROTATE_LEFT();  Serial.println("Rotating LEFT"); }
      else if (cmd == "right"   || cmd == "r") { ROTATE_RIGHT(); Serial.println("Rotating RIGHT"); }
      else if (cmd == "stop"    || cmd == "s") { FULL_STOP();    Serial.println("STOPPED"); }
      else if (cmd == "open")                  { openBox(); }
      else if (cmd == "close")                 { closeBox(); }
      else if (cmd == "reset")                 { emergencyActive = false; Serial.println("Emergency reset via serial."); }
    }
  }

  // Normal Mode 0 operation
  if (lastUserCommand == "forward") {
    if (obstacleDetected()) {
      TEMPORARY_STOP();
      Serial.println("Obstacle Detected - Stopped");
    } else {
      checkUWBAndStop();
      if (lastUserCommand == "forward") {
        lineTrack();
      }
    }
  }

  delay(30);
}