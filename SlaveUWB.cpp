// ============================================================
// ADVANCED UWB FOLLOWER - With direction detection
// Uses 2 anchors for position tracking
// ============================================================

#include <math.h>

// Motor pins (same as above)
#define ENA 9
#define ENB 10
#define IN1 4
#define IN2 5
#define IN3 6
#define IN4 7

// PID constants
float Kp_linear = 2.5;
float Kp_angular = 3.0;
float Ki_linear = 0.05;
float Ki_angular = 0.08;
float Kd_linear = 1.2;
float Kd_angular = 1.5;

float TARGET_DISTANCE = 1.0;  // meters
float DEADZONE_DIST = 0.1;
float DEADZONE_ANGLE = 0.1;   // radians (~5.7 degrees)

// Position tracking
float robot_x = 0;
float robot_y = 0;
float target_x = 0;
float target_y = 0;
float distance_to_target = 0;
float angle_to_target = 0;

// Anchor positions (in meters)
float anchor1_x = 0;
float anchor1_y = 0;
float anchor2_x = 1;
float anchor2_y = 0;

// UWB data
float dist_to_anchor1 = 0;
float dist_to_anchor2 = 0;
bool have_anchor1 = false;
bool have_anchor2 = false;
unsigned long last_update = 0;

// Motor control
int left_speed = 0;
int right_speed = 0;
float linear_error = 0;
float angular_error = 0;

// PID variables
float prev_linear_error = 0;
float prev_angular_error = 0;
float integral_linear = 0;
float integral_angular = 0;
unsigned long last_time = 0;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);  // DWM1001
  
  // Motor setup
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  stopMotors();
  
  Serial.println("Advanced UWB Follower Started");
  Serial.println("Using 2 anchors for position tracking");
}

void loop() {
  readUWBData();
  calculatePosition();
  
  if (have_anchor1 && have_anchor2) {
    calculateTargetPosition();
    calculateFollowingErrors();
    
    float linear_output = computePIDLinear();
    float angular_output = computePIDAngular();
    
    calculateMotorSpeeds(linear_output, angular_output);
    moveRobot();
    
    printDebug();
  } else {
    stopMotors();
    Serial.println("Waiting for both anchors...");
  }
  
  delay(50);
}

// ============================================================
// Read distances from both anchors
// ============================================================
void readUWBData() {
  while (Serial1.available()) {
    String line = Serial1.readStringUntil('\n');
    line.trim();
    
    if (line.startsWith("R")) {
      int comma1 = line.indexOf(',');
      int comma2 = line.indexOf(',', comma1 + 1);
      int comma3 = line.indexOf(',', comma2 + 1);
      int comma4 = line.indexOf(',', comma3 + 1);
      
      if (comma1 != -1 && comma2 != -1 && comma3 != -1 && comma4 != -1) {
        int anchor_id = line.substring(comma1 + 1, comma2).toInt();
        float distance = line.substring(comma2 + 1, comma3).toFloat();
        float quality = line.substring(comma3 + 1, comma4).toFloat();
        
        if (quality < 0.1) {  // Good quality
          if (anchor_id == 0) {
            dist_to_anchor1 = distance;
            have_anchor1 = true;
          } else if (anchor_id == 1) {
            dist_to_anchor2 = distance;
            have_anchor2 = true;
          }
        }
      }
    }
  }
}

// ============================================================
// Calculate robot position using trilateration
// ============================================================
void calculatePosition() {
  if (have_anchor1 && have_anchor2) {
    // Simple triangulation
    float dx = anchor2_x - anchor1_x;
    float dy = anchor2_y - anchor1_y;
    float d = sqrt(dx*dx + dy*dy);
    
    if (d > 0) {
      float a = (dist_to_anchor1*dist_to_anchor1 - dist_to_anchor2*dist_to_anchor2 + d*d) / (2*d);
      float h = sqrt(abs(dist_to_anchor1*dist_to_anchor1 - a*a));
      
      float x = anchor1_x + a * (dx/d);
      float y = anchor1_y + h * (dy/d);
      
      robot_x = x;
      robot_y = y;
    }
  }
}

// ============================================================
// Calculate where target should be (behind master)
// ============================================================
void calculateTargetPosition() {
  // For simple following, target is at fixed offset from anchor 1
  target_x = anchor1_x + TARGET_DISTANCE;
  target_y = anchor1_y;
  
  // Calculate distance and angle
  float dx = target_x - robot_x;
  float dy = target_y - robot_y;
  distance_to_target = sqrt(dx*dx + dy*dy);
  angle_to_target = atan2(dy, dx);
}

// ============================================================
// Calculate following errors
// ============================================================
void calculateFollowingErrors() {
  linear_error = distance_to_target;
  angular_error = angle_to_target;
  
  // Apply deadzones
  if (fabs(linear_error) < DEADZONE_DIST) linear_error = 0;
  if (fabs(angular_error) < DEADZONE_ANGLE) angular_error = 0;
}

// ============================================================
// PID controllers
// ============================================================
float computePIDLinear() {
  unsigned long now = millis();
  float dt = (now - last_time) / 1000.0;
  if (dt <= 0) dt = 0.05;
  
  // Proportional
  float proportional = Kp_linear * linear_error;
  
  // Integral
  integral_linear += linear_error * dt;
  integral_linear = constrain(integral_linear, -50, 50);
  float integral_term = Ki_linear * integral_linear;
  
  // Derivative
  float derivative = (linear_error - prev_linear_error) / dt;
  float derivative_term = Kd_linear * derivative;
  
  float output = proportional + integral_term + derivative_term;
  output = constrain(output, -255, 255);
  
  prev_linear_error = linear_error;
  return output;
}

float computePIDAngular() {
  float proportional = Kp_angular * angular_error;
  
  integral_angular += angular_error * 0.05;
  integral_angular = constrain(integral_angular, -50, 50);
  float integral_term = Ki_angular * integral_angular;
  
  float derivative = (angular_error - prev_angular_error) / 0.05;
  float derivative_term = Kd_angular * derivative;
  
  float output = proportional + integral_term + derivative_term;
  output = constrain(output, -255, 255);
  
  prev_angular_error = angular_error;
  return output;
}

// ============================================================
// Motor control with direction
// ============================================================
void calculateMotorSpeeds(float linear, float angular) {
  // Differential drive: left = linear - angular, right = linear + angular
  left_speed = constrain(linear - angular, -255, 255);
  right_speed = constrain(linear + angular, -255, 255);
  
  // Apply minimum speed to overcome friction
  if (abs(left_speed) > 0 && abs(left_speed) < 60) {
    left_speed = (left_speed > 0) ? 60 : -60;
  }
  if (abs(right_speed) > 0 && abs(right_speed) < 60) {
    right_speed = (right_speed > 0) ? 60 : -60;
  }
}

void moveRobot() {
  // Left motor
  if (left_speed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, abs(left_speed));
  } else if (left_speed < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, abs(left_speed));
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
  
  // Right motor
  if (right_speed > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, abs(right_speed));
  } else if (right_speed < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, abs(right_speed));
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}

void stopMotors() {
  left_speed = 0;
  right_speed = 0;
  moveRobot();
}

void printDebug() {
  static unsigned long last_print = 0;
  
  if (millis() - last_print > 200) {
    Serial.print("Pos: (");
    Serial.print(robot_x, 2);
    Serial.print(", ");
    Serial.print(robot_y, 2);
    Serial.print(") | Dist: ");
    Serial.print(distance_to_target, 2);
    Serial.print("m | Angle: ");
    Serial.print(angle_to_target, 2);
    Serial.print(" rad | L:");
    Serial.print(left_speed);
    Serial.print(" R:");
    Serial.println(right_speed);
    
    last_print = millis();
  }
}
