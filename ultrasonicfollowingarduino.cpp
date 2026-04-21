#include <Arduino.h>
#include <NewPing.h>
#include <Servo.h>

// ====================== PIN DEFINITIONS FOR ARDUINO MEGA ======================
// Motor Driver Pins (Using your original pin assignments)
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

// IR Sensors for Human Following
#define RIGHT_IR A2
#define LEFT_IR A3

// Ultrasonic Sensor Pins
#define TRIGGER_PIN A6
#define ECHO_PIN A7
#define MAX_DISTANCE 100

// Servo for Ultrasonic Sweeping
#define SERVO_PIN 28

// Motor Speed Settings
int baseSpeed = 150;  // Base speed for following

// ====================== OBJECTS ======================
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
Servo ultrasonicServo;

// ====================== MOTOR CONTROL FUNCTIONS ======================
// Using your exact motor macros
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
void moveForward(int speed) {
  Serial.println("Moving FORWARD");
  MOTORA_FORWARD(speed);
  MOTORB_BACKOFF(speed);
  MOTORC_FORWARD(speed);
  MOTORD_BACKOFF(speed);
}

void moveBackward(int speed) {
  Serial.println("Moving BACKWARD");
  MOTORA_BACKOFF(speed);
  MOTORB_FORWARD(speed);
  MOTORC_BACKOFF(speed);
  MOTORD_FORWARD(speed);
}

void turnRight(int speed) {
  Serial.println("Turning RIGHT");
  MOTORA_FORWARD(speed);
  MOTORB_FORWARD(speed);
  MOTORC_BACKOFF(speed);
  MOTORD_BACKOFF(speed);
}

void turnLeft(int speed) {
  Serial.println("Turning LEFT");
  MOTORA_BACKOFF(speed);
  MOTORB_BACKOFF(speed);
  MOTORC_FORWARD(speed);
  MOTORD_FORWARD(speed);
}

void stopMotors() {
  Serial.println("STOPPING");
  MOTORA_STOP();
  MOTORB_STOP();
  MOTORC_STOP();
  MOTORD_STOP();
}

// ====================== SERVO SWEEP FUNCTION ======================
void sweepServo() {
  Serial.println("Initializing Servo Sweep...");
  
  // Sweep from 90 to 180 degrees
  for(int pos = 90; pos <= 180; pos += 1) {
    ultrasonicServo.write(pos);
    delay(15);
  }
  
  // Sweep from 180 to 0 degrees
  for(int pos = 180; pos >= 0; pos -= 1) {
    ultrasonicServo.write(pos);
    delay(15);
  }
  
  // Sweep from 0 to 90 degrees
  for(int pos = 0; pos <= 90; pos += 1) {
    ultrasonicServo.write(pos);
    delay(15);
  }
  
  Serial.println("Servo Sweep Complete");
}

// ====================== SETUP ======================
void setup() {
  // Initialize motor pins
  pinMode(PWMA, OUTPUT); pinMode(DIRA1, OUTPUT); pinMode(DIRA2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(DIRB1, OUTPUT); pinMode(DIRB2, OUTPUT);
  pinMode(PWMC, OUTPUT); pinMode(DIRC1, OUTPUT); pinMode(DIRC2, OUTPUT);
  pinMode(PWMD, OUTPUT); pinMode(DIRD1, OUTPUT); pinMode(DIRD2, OUTPUT);
  
  // Initialize IR sensor pins
  pinMode(RIGHT_IR, INPUT);
  pinMode(LEFT_IR, INPUT);
  
  // Initialize servo
  ultrasonicServo.attach(SERVO_PIN);
  
  // Perform initial servo sweep
  sweepServo();
  
  // Set servo to center position
  ultrasonicServo.write(90);
  
  // Stop all motors
  stopMotors();
  
  // Initialize serial communication
  Serial.begin(9600);
  Serial.println("=== HUMAN FOLLOWING ROBOT READY ===");
  Serial.println("Arduino Mega 2560 - AFMotor Replacement");
  Serial.println("=====================================");
  delay(1000);
}

// ====================== MAIN LOOP ======================
void loop() {
  delay(50);
  
  // Read ultrasonic distance
  unsigned int distance = sonar.ping_cm();
  
  // Read IR sensor values
  int Right_Value = digitalRead(RIGHT_IR);
  int Left_Value = digitalRead(LEFT_IR);
  
  // Print debug information
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print(" cm | RIGHT: ");
  Serial.print(Right_Value);
  Serial.print(" | LEFT: ");
  Serial.println(Left_Value);
  
  // ====================== HUMAN FOLLOWING LOGIC ======================
  
  // Case 1: Person detected in front (both IR sensors active AND distance between 10-30cm)
  if((Right_Value == 1) && (distance >= 10 && distance <= 30) && (Left_Value == 1)) {
    Serial.println("STATUS: Person detected in front - Moving FORWARD");
    moveForward(120);
  }
  
  // Case 2: Person detected on right side (right IR only)
  else if((Right_Value == 0) && (Left_Value == 1)) {
    Serial.println("STATUS: Person on RIGHT - Turning RIGHT");
    turnRight(150);  // Using 150 speed for turning
  }
  
  // Case 3: Person detected on left side (left IR only)
  else if((Right_Value == 1) && (Left_Value == 0)) {
    Serial.println("STATUS: Person on LEFT - Turning LEFT");
    turnLeft(150);   // Using 150 speed for turning
  }
  
  // Case 4: No person detected (both IR sensors inactive)
  else if((Right_Value == 1) && (Left_Value == 1)) {
    Serial.println("STATUS: No person detected - STOPPED");
    stopMotors();
  }
  
  // Case 5: Person too close (distance less than 10cm)
  else if(distance > 1 && distance < 10) {
    Serial.println("STATUS: Person TOO CLOSE - STOPPED for safety");
    stopMotors();
  }
  
  // Case 6: Person too far (distance > 30cm but IR sensors detect)
  else if((Right_Value == 1) && (Left_Value == 1) && (distance > 30 && distance < MAX_DISTANCE)) {
    Serial.println("STATUS: Person too far - Moving FORWARD to catch up");
    moveForward(160);  // Slightly faster to catch up
  }
  
  // Default case: Stop if nothing else matches
  else {
    stopMotors();
  }
}
