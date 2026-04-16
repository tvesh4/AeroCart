#include <Arduino.h>

// ====================== MOTOR PINS ======================
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

int Motor_PWM = 1900;     // Default speed (0-2550 for Mega)

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

// ====================== MOVEMENT FUNCTIONS ======================
void ADVANCE() {
  MOTORA_FORWARD(Motor_PWM);
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);
  MOTORD_BACKOFF(Motor_PWM);
}

void BACK() {
  MOTORA_BACKOFF(Motor_PWM);
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM);
  MOTORD_FORWARD(Motor_PWM);
}

void STOP() {
  MOTORA_STOP();
  MOTORB_STOP();
  MOTORC_STOP();
  MOTORD_STOP();
}

void LEFT() {
  MOTORA_BACKOFF(Motor_PWM/2);
  MOTORB_BACKOFF(Motor_PWM/2);
  MOTORC_FORWARD(Motor_PWM/2);
  MOTORD_FORWARD(Motor_PWM/2);
}

void RIGHT() {
  MOTORA_FORWARD(Motor_PWM/2);
  MOTORB_FORWARD(Motor_PWM/2);
  MOTORC_BACKOFF(Motor_PWM/2);
  MOTORD_BACKOFF(Motor_PWM/2);
}

// ====================== SETUP ======================
void setup() {
  // Set all pins as output
  pinMode(PWMA, OUTPUT);  pinMode(DIRA1, OUTPUT);  pinMode(DIRA2, OUTPUT);
  pinMode(PWMB, OUTPUT);  pinMode(DIRB1, OUTPUT);  pinMode(DIRB2, OUTPUT);
  pinMode(PWMC, OUTPUT);  pinMode(DIRC1, OUTPUT);  pinMode(DIRC2, OUTPUT);
  pinMode(PWMD, OUTPUT);  pinMode(DIRD1, OUTPUT);  pinMode(DIRD2, OUTPUT);

  STOP();   // Make sure motors are stopped at start

  Serial.begin(115200);        // ← Recommended for Jetson communication
  Serial.println("=== Arduino Motor Control Ready ===");
  Serial.println("Waiting for commands from Jetson...");
}

// ====================== MAIN LOOP ======================
void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toLowerCase();

    if (cmd == "forward" || cmd == "f" || cmd == "advance") {
      ADVANCE();
      Serial.println("Moving FORWARD");
    }
    else if (cmd == "back" || cmd == "b" || cmd == "backward") {
      BACK();
      Serial.println("Moving BACKWARD");
    }
    else if (cmd == "left" || cmd == "l") {
      LEFT();
      Serial.println("Turning LEFT");
    }
    else if (cmd == "right" || cmd == "r") {
      RIGHT();
      Serial.println("Turning RIGHT");
    }
    else if (cmd == "stop" || cmd == "s") {
      STOP();
      Serial.println("STOPPED");
    }
    else if (cmd.startsWith("speed ")) {
      int newSpeed = cmd.substring(6).toInt();
      if (newSpeed >= 0 && newSpeed <= 2550) {
        Motor_PWM = newSpeed;
        Serial.print("Speed set to: ");
        Serial.println(Motor_PWM);
      }
    }
    else {
      Serial.println("Unknown command");
    }
  }
}
