// =============================================
// Simplified Follow-Me Robot Code
// Only Movement + 2 IR Sensors (Stop on Obstacle)
// =============================================

#define IRpin1  40     // Left Front IR Sensor
#define IRpin2  47     // Right Front IR Sensor

// Motor Pins
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

int Motor_PWM = 1900;     // Default speed (you can change)

// Motor Macros
#define MOTORA_FORWARD(pwm) do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,HIGH); analogWrite(PWMA,pwm);}while(0)
//#define MOTORA_STOP()       do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,LOW); analogWrite(PWMA,0);}while(0)
#define MOTORA_BACKOFF(pwm) do{digitalWrite(DIRA1,HIGH);digitalWrite(DIRA2,LOW); analogWrite(PWMA,pwm);}while(0)

#define MOTORB_FORWARD(pwm) do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,HIGH);analogWrite(PWMB,pwm);}while(0)
//#define MOTORB_STOP()       do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,LOW); analogWrite(PWMB,0);}while(0)
#define MOTORB_BACKOFF(pwm) do{digitalWrite(DIRB1,HIGH);digitalWrite(DIRB2,LOW); analogWrite(PWMB,pwm);}while(0)

#define MOTORC_FORWARD(pwm) do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,HIGH);analogWrite(PWMC,pwm);}while(0)
//#define MOTORC_STOP()       do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,LOW); analogWrite(PWMC,0);}while(0)
#define MOTORC_BACKOFF(pwm) do{digitalWrite(DIRC1,HIGH);digitalWrite(DIRC2,LOW); analogWrite(PWMC,pwm);}while(0)

#define MOTORD_FORWARD(pwm) do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,HIGH);analogWrite(PWMD,pwm);}while(0)
//#define MOTORD_STOP()       do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,LOW); analogWrite(PWMD,0);}while(0)
#define MOTORD_BACKOFF(pwm) do{digitalWrite(DIRD1,HIGH);digitalWrite(DIRD2,LOW); analogWrite(PWMD,pwm);}while(0)

#define MOTORA_STOP() do{digitalWrite(DIRA1,HIGH); digitalWrite(DIRA2,HIGH); analogWrite(PWMA,0);}while(0)
#define MOTORB_STOP() do{digitalWrite(DIRB1,HIGH); digitalWrite(DIRB2,HIGH); analogWrite(PWMB,0);}while(0)
#define MOTORC_STOP() do{digitalWrite(DIRC1,HIGH); digitalWrite(DIRC2,HIGH); analogWrite(PWMC,0);}while(0)
#define MOTORD_STOP() do{digitalWrite(DIRD1,HIGH); digitalWrite(DIRD2,HIGH); analogWrite(PWMD,0);}while(0)
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

// ====================== IR OBSTACLE AVOIDANCE ======================
bool obstacleDetected() {
  return digitalRead(IRpin1) == 0 || digitalRead(IRpin2) == 0;  // Return true if any sensor detects obstacle
}

// ====================== SETUP ======================
void setup() {
  Serial.begin(115200);
  
  // IR Sensors
  pinMode(IRpin1, INPUT);
  pinMode(IRpin2, INPUT);

  // Motor Pins
  pinMode(PWMA, OUTPUT);  pinMode(DIRA1, OUTPUT);  pinMode(DIRA2, OUTPUT);
  pinMode(PWMB, OUTPUT);  pinMode(DIRB1, OUTPUT);  pinMode(DIRB2, OUTPUT);
  pinMode(PWMC, OUTPUT);  pinMode(DIRC1, OUTPUT);  pinMode(DIRC2, OUTPUT);
  pinMode(PWMD, OUTPUT);  pinMode(DIRD1, OUTPUT);  pinMode(DIRD2, OUTPUT);

  STOP();   // Make sure motors are stopped at start
  
  Serial.println("Robot Ready - Movement + IR Obstacle Stop");
  Serial.println("IR1 (Pin 2) | IR2 (Pin 3)");
}

// ====================== MAIN LOOP ======================
void loop() {

  if (obstacleDetected()) {
    STOP();

    while (obstacleDetected()) {
      STOP();   // keep braking
    }
  } else {
    ADVANCE();
  }
}
