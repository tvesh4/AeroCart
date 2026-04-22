#include <Arduino.h>
#include <Servo.h>

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

#define IRpin1 40
#define IRpin2 47

#define EMERGENCY_BTN 28   // Emergency stop button

// ====================== IR LINE SENSORS (5-channel) ======================
#define ir1 A0  // Left Most
#define ir2 A2  // Left
#define ir3 A3  // Middle
#define ir4 A8  // Right
#define ir5 A9  // Right Most

#define LINE_THRESHOLD 500

int Motor_PWM = 300;

// ====================== SERVO ======================
const int servoPin = 48;
Servo boxServo;
bool boxOpen = false;

// ====================== UWB DISTANCE VARIABLES ======================
float currentDistance = 0.0;
float targetDistance = 0.0;
bool targetDistanceSet = false;
bool uwbDataValid = false;
float stopTolerance = 0.05;

// ====================== STATE ======================
String lastUserCommand = "stop";
bool emergencyActive = false;   // Emergency state flag

// ====================== MOTOR MACROS ======================
#define MOTORA_FORWARD(pwm)   do{digitalWrite(DIRA1,LOW);  digitalWrite(DIRA2,HIGH); analogWrite(PWMA,pwm);}while(0)
#define MOTORA_BACKOFF(pwm)   do{digitalWrite(DIRA1,HIGH); digitalWrite(DIRA2,LOW);  analogWrite(PWMA,pwm);}while(0)
#define MOTORA_STOP()         do{digitalWrite(DIRA1,HIGH); digitalWrite(DIRA2,HIGH); analogWrite(PWMA,0);}while(0)

#define MOTORB_FORWARD(pwm)   do{digitalWrite(DIRB1,LOW);  digitalWrite(DIRB2,HIGH); analogWrite(PWMB,pwm);}while(0)
#define MOTORB_BACKOFF(pwm)   do{digitalWrite(DIRB1,HIGH); digitalWrite(DIRB2,LOW);  analogWrite(PWMB,pwm);}while(0)
#define MOTORB_STOP()         do{digitalWrite(DIRB1,HIGH); digitalWrite(DIRB2,HIGH); analogWrite(PWMB,0);}while(0)

#define MOTORC_FORWARD(pwm)   do{digitalWrite(DIRC1,LOW);  digitalWrite(DIRC2,HIGH); analogWrite(PWMC,pwm);}while(0)
#define MOTORC_BACKOFF(pwm)   do{digitalWrite(DIRC1,HIGH); digitalWrite(DIRC2,LOW);  analogWrite(PWMC,pwm);}while(0)
#define MOTORC_STOP()         do{digitalWrite(DIRC1,HIGH); digitalWrite(DIRC2,HIGH); analogWrite(PWMC,0);}while(0)

#define MOTORD_FORWARD(pwm)   do{digitalWrite(DIRD1,LOW);  digitalWrite(DIRD2,HIGH); analogWrite(PWMD,pwm);}while(0)
#define MOTORD_BACKOFF(pwm)   do{digitalWrite(DIRD1,HIGH); digitalWrite(DIRD2,LOW);  analogWrite(PWMD,pwm);}while(0)
#define MOTORD_STOP()         do{digitalWrite(DIRD1,HIGH); digitalWrite(DIRD2,HIGH); analogWrite(PWMD,0);}while(0)

// ====================== EMERGENCY STOP ======================
void EMERGENCY_STOP() {
  // Kill all motors immediately
  MOTORA_STOP(); MOTORB_STOP(); MOTORC_STOP(); MOTORD_STOP();

  // Close servo if open
  if (boxOpen) {
    for (int pos = 90; pos >= 0; pos -= 2) { boxServo.write(pos); delay(20); }
    boxOpen = false;
  }

  // Reset all state
  lastUserCommand = "stop";
  targetDistanceSet = false;
  uwbDataValid = false;
  Motor_PWM = 300;
  emergencyActive = true;

  Serial.println("!!! EMERGENCY STOP ACTIVATED !!!");
  Serial.println("Please Go Find Airport Staff");
}

// ====================== MOVEMENT ======================
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
void trackTurnRight() {
  MOTORA_FORWARD(Motor_PWM); MOTORB_FORWARD(Motor_PWM);
  MOTORC_STOP();              MOTORD_STOP();
}
void trackTurnRightHard() {
  MOTORA_FORWARD(Motor_PWM); MOTORB_FORWARD(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM); MOTORD_FORWARD(Motor_PWM);
}
void trackTurnLeft() {
  MOTORA_BACKOFF(Motor_PWM); MOTORB_BACKOFF(Motor_PWM);
  MOTORC_STOP();              MOTORD_STOP();
}
void trackTurnLeftHard() {
  MOTORA_BACKOFF(Motor_PWM); MOTORB_BACKOFF(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM); MOTORD_BACKOFF(Motor_PWM);
}

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
    }
    else if (remaining <= 0.5 && remaining > 0) {
      int newSpeed = map(remaining * 100, 0, 50, 150, 300);
      Motor_PWM = constrain(newSpeed, 150, 300);
    } else {
      Motor_PWM = 300;
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

// ====================== OBSTACLE DETECTION ======================
bool obstacleDetected() {
  return (digitalRead(IRpin1) == 0) || (digitalRead(IRpin2) == 0);
}

// ====================== SETUP ======================
void setup() {
  pinMode(PWMA, OUTPUT); pinMode(DIRA1, OUTPUT); pinMode(DIRA2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(DIRB1, OUTPUT); pinMode(DIRB2, OUTPUT);
  pinMode(PWMC, OUTPUT); pinMode(DIRC1, OUTPUT); pinMode(DIRC2, OUTPUT);
  pinMode(PWMD, OUTPUT); pinMode(DIRD1, OUTPUT); pinMode(DIRD2, OUTPUT);

  pinMode(IRpin1, INPUT_PULLUP);
  pinMode(IRpin2, INPUT_PULLUP);

  pinMode(ir1, INPUT);
  pinMode(ir2, INPUT);
  pinMode(ir3, INPUT);
  pinMode(ir4, INPUT);
  pinMode(ir5, INPUT);

  pinMode(EMERGENCY_BTN, INPUT_PULLUP);  // Internal pull-up, button connects pin to GND

  boxServo.attach(servoPin);
  boxServo.write(0);

  FULL_STOP();

  Serial.begin(115200);
  Serial.println("=== System Ready ===");
  Serial.println("Commands: forward, back, left, right, stop, open, close, reset");
  Serial.println("UWB Commands: TARGET:<distance>, DIST:<current_distance>");
  Serial.println("Emergency: Press red button");
}

// ====================== MAIN LOOP ======================
void loop() {

  // ---- EMERGENCY BUTTON CHECK (highest priority) ----
  if (digitalRead(EMERGENCY_BTN) == LOW) {
    if (!emergencyActive) {
      EMERGENCY_STOP();
      delay(300);  // Debounce
    } else {
      // Button pressed again while emergency active → reset
      emergencyActive = false;
      Serial.println("Emergency reset. System resumed.");
      delay(300);  // Debounce
    }
  }

  // ---- Block everything if emergency is active ----
  if (emergencyActive) {
    MOTORA_STOP(); MOTORB_STOP(); MOTORC_STOP(); MOTORD_STOP();
    delay(30);
    return;
  }

  // ---- Serial commands ----
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("TARGET:")) {
      float newTarget = cmd.substring(7).toFloat();
      if (newTarget > 0) {
        targetDistance = newTarget;
        targetDistanceSet = true;
        Serial.print("Target distance set to: ");
        Serial.print(targetDistance);
        Serial.println(" meters");
      }
    }
    else if (cmd.startsWith("DIST:")) {
      float distance = cmd.substring(5).toFloat();
      if (distance >= 0) {
        currentDistance = distance;
        uwbDataValid = true;
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

  // ---- Normal operation ----
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