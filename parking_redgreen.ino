#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050_light.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32

// OLED
#define OLED_RESET 28
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


#include <Servo.h>
Servo servo_pan;
Servo servo_tilt;
int servo_min = 20;
int servo_max = 160;
int pan = 90;
int tilt = 120;
int window_size = 0;

// Gyroscope
MPU6050 mpu(Wire);
float target_heading = 0;
float current_heading = 0;
float heading_error = 0;

// Distance var
unsigned long start_time_front = 0;
unsigned long start_time_left = 0;
unsigned long start_time_right = 0;
int done_front = 1;
int done_left = 1;
int done_right = 1;
long distance_front_cm;
long distance_left_cm;
long distance_right_cm;

//colour
#define out 25
#define s2 28
#define s3 29
#define s0 30
#define s1 32


#define MAX_PWM   2000
#define MIN_PWM   300

int Motor_PWM = 1900;
int Tolerance = 100;
int D_PWM = 28;
int pervious_front = 1000;

#define PWMA 12    //Motor A PWM
#define DIRA1 34
#define DIRA2 35  //Motor A Direction
#define PWMB 8    //Motor B PWM
#define DIRB1 37
#define DIRB2 36  //Motor B Direction
#define PWMC 9   //Motor C PWM --> from 6 to 9
#define DIRC1 43
#define DIRC2 42  //Motor C Direction
#define PWMD 5    //Motor D PWM
#define DIRD1 A4  //26  
#define DIRD2 A5  //27  //Motor D Direction

#define MOTORA_FORWARD(pwm)    do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,HIGH);analogWrite(PWMA,pwm);}while(0)
#define MOTORA_STOP(x)         do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,LOW); analogWrite(PWMA,0);}while(0)
#define MOTORA_BACKOFF(pwm)    do{digitalWrite(DIRA1,HIGH);digitalWrite(DIRA2,LOW); analogWrite(PWMA,pwm);}while(0)

#define MOTORB_FORWARD(pwm)    do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,HIGH);analogWrite(PWMB,pwm);}while(0)
#define MOTORB_STOP(x)         do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,LOW); analogWrite(PWMB,0);}while(0)
#define MOTORB_BACKOFF(pwm)    do{digitalWrite(DIRB1,HIGH);digitalWrite(DIRB2,LOW); analogWrite(PWMB,pwm);}while(0)

#define MOTORC_FORWARD(pwm)    do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,HIGH);analogWrite(PWMC,pwm);}while(0)
#define MOTORC_STOP(x)         do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,LOW); analogWrite(PWMC,0);}while(0)
#define MOTORC_BACKOFF(pwm)    do{digitalWrite(DIRC1,HIGH);digitalWrite(DIRC2,LOW); analogWrite(PWMC,pwm);}while(0)

#define MOTORD_FORWARD(pwm)    do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,HIGH);analogWrite(PWMD,pwm);}while(0)
#define MOTORD_STOP(x)         do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,LOW); analogWrite(PWMD,0);}while(0)
#define MOTORD_BACKOFF(pwm)    do{digitalWrite(DIRD1,HIGH);digitalWrite(DIRD2,LOW); analogWrite(PWMD,pwm);}while(0)

#define SERIAL Serial
#define BTSERIAL Serial3

// Light Sensors
#define Light_left A6
#define Light_right A6
#define Light_front A2  // Front light sensor for LED source

//variables for light intensity to ADC reading equations 
int int_adc0, int_adc0_m, int_adc0_c;
int int_adc1, int_adc1_m, int_adc1_c;     
int int_left, int_right;

// Ultrasonic Sensors
#define echoPin_Lfront 61
#define trigPin_Lfront 62
#define echoPin_Rfront 63
#define trigPin_Rfront 64
#define echoPin_left 65
#define trigPin_left 66
#define echoPin_right 67
#define trigPin_right 68

// Calibration structure for ultrasonic sensors
struct SensorCalibration {
  float offset;      // Offset in cm
  float scale;       // Scale factor
  bool isCalibrated;
};

// Calibration data for each ultrasonic sensor
SensorCalibration cal_Lfront = {0, 1.0, false};
SensorCalibration cal_Rfront = {0, 1.0, false};
SensorCalibration cal_left = {0, 1.0, false};
SensorCalibration cal_right = {0, 1.0, false};

// State machine
enum RobotState {
  STATE_SEEKING_LIGHT,
  STATE_APPROACHING_FRONT,
  STATE_ALIGNING_WALLS,
  STATE_COLOR_DETECTION,
  STATE_READY_TO_CHOOSE
};

RobotState current_state = STATE_SEEKING_LIGHT;
// RobotState current_state = STATE_COLOR_DETECTION;

// Motor control functions
void BACK()
{
  MOTORA_BACKOFF(Motor_PWM); 
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM); 
  MOTORD_FORWARD(Motor_PWM);
}

//    ↓A-----B↓
//     |  |  |
//     |  ↓  |
//    ↓C-----D↓
void ADVANCE()
{
  MOTORA_FORWARD(Motor_PWM); 
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM); 
  MOTORD_BACKOFF(Motor_PWM);
}
//    =A-----B↑
//     |   ↖️ |
//     | ↖️   |
//    ↑C-----D=
void LEFT_1()
{
  MOTORA_STOP(Motor_PWM); 
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM); 
  MOTORD_STOP(Motor_PWM);
}

//    ↓A-----B↑
//     |  ←  |
//     |  ←  |
//    ↑C-----D↓
void RIGHT_2()
{
  MOTORA_FORWARD(Motor_PWM); 
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM); 
  MOTORD_BACKOFF(Motor_PWM);
}
//    ↓A-----B=
//     | ↙️   |
//     |   ↙️ |
//    =C-----D↓
void LEFT_3()
{
  MOTORA_FORWARD(Motor_PWM); 
  MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM); 
  MOTORD_BACKOFF(Motor_PWM);
}
//    ↑A-----B=
//     | ↗️   |
//     |   ↗️ |
//    =C-----D↑
void RIGHT_1()
{
  MOTORA_BACKOFF(Motor_PWM); 
  MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM); 
  MOTORD_FORWARD(Motor_PWM);
}
//    ↑A-----B↓
//     |  →  |
//     |  →  |
//    ↓C-----D↑
void LEFT_2()
{
  // MOTORA_BACKOFF(300); 
  // MOTORB_BACKOFF(300);
  // MOTORC_FORWARD(310); 
  // MOTORD_STOP(Motor_PWM);
  MOTORA_BACKOFF(Motor_PWM); 
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM); 
  MOTORD_FORWARD(Motor_PWM);

}
//    =A-----B↓
//     |   ↘️ |
//     | ↘️   |
//    ↓C-----D=
void RIGHT_3()
{
  MOTORA_STOP(Motor_PWM); 
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM); 
  MOTORD_STOP(Motor_PWM);
}

//    ↑A-----B↓
//     | ↗️ ↘️ |
//     | ↖️ ↙️ |
//    ↑C-----D↓
void rotate_1()  //tate_1(uint8_t pwm_A,uint8_t pwm_B,uint8_t pwm_C,uint8_t pwm_D)
{
  MOTORA_BACKOFF(Motor_PWM); 
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM); 
  MOTORD_BACKOFF(Motor_PWM);
}

//    ↓A-----B↑
//     | ↙️ ↖️ |
//     | ↘️ ↗️ |
//    ↓C-----D↑
void rotate_2()  // rotate_2(uint8_t pwm_A,uint8_t pwm_B,uint8_t pwm_C,uint8_t pwm_D)
{
  MOTORA_FORWARD(Motor_PWM);
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);
  MOTORD_FORWARD(Motor_PWM);
}
//    =A-----B=
//     |  =  |
//     |  =  |
//    =C-----D=
void STOP()
{
  MOTORA_STOP(Motor_PWM);
  MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM);
  MOTORD_STOP(Motor_PWM);
}

// Calibrated distance
float measure_distance(int trigPin, int echoPin, SensorCalibration &cal) {
  float duration, distance_raw, distance_calibrated;
  
  // Send 10 µs trigger pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH, 30000); // Timeout after 30ms (about 5 meters)
  
  if (duration == 0) {
    return 999.0; // Return large number for no echo
  }
  
  // Convert duration to distance in cm
  distance_raw = (duration * 0.0343) / 2;
  
  // Apply calibration
  distance_calibrated = (distance_raw * cal.scale) + cal.offset;
  
  return distance_calibrated;
}

// Uncalibrated distance measurement for calibration process
float measure_distance_uncalibrated(int trigPin, int echoPin) {
  float duration, distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return 999.0;
  distance = (duration * 0.0343) / 2;
  return distance;
}

float long_measure_distance(int trigPin, int echoPin) {
  float total, duration, distance;
  total = 0;
  // Send 10 µs trigger pulse
  for (int i=0;i<10;i++){
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH);
    distance = (duration*.0343)/2;
    total += distance;
    delay(5);
  }
  total = total/10;
  
  return total;
}

// Calibration function for a specific sensor
void calibrate_sensor(int trigPin, int echoPin, SensorCalibration &cal, float known_distance, const char* sensor_name) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Calibrating:");
  display.println(sensor_name);
  display.print("Place at ");
  display.print(known_distance);
  display.println("cm");
  display.display();
  Serial.print("Calibrating ");
  Serial.print(sensor_name);
  Serial.print(" at ");
  Serial.print(known_distance);
  Serial.println(" cm...");
  
  delay(3000);
  
  // Take multiple readings for accuracy
  float sum = 0;
  int readings = 10;
  int valid_readings = 0;
  
  for (int i = 0; i < readings; i++) {
    float distance_raw = measure_distance_uncalibrated(trigPin, echoPin);
    if (distance_raw < 500 && distance_raw > 0) { // Valid reading
      sum += distance_raw;
      valid_readings++;
    }
    
    // Show progress
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Reading:");
    display.print(i+1);
    display.print("/");
    display.println(readings);
    if (distance_raw < 500) {
      display.print("Raw: ");
      display.print(distance_raw, 1);
      display.println("cm");
    } else {
      display.print("No echo");
    }
    display.display();
    
    delay(100);
  }
  
  if (valid_readings > 0) {
    float average_raw = sum / valid_readings;
    
    // Calculate scale factor
    if (average_raw > 0) {
      cal.scale = known_distance / average_raw;
      cal.offset = 0;
      cal.isCalibrated = true;
      
      Serial.print(sensor_name);
      Serial.print(" - Raw: ");
      Serial.print(average_raw, 1);
      Serial.print("cm, Scale: ");
      Serial.println(cal.scale, 3);
      
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.println("Calibrated!");
      display.print(sensor_name);
      display.println("");
      display.print("Scale: ");
      display.println(cal.scale, 3);
      display.display();
      delay(2000);
    }
  } else {
    Serial.print(sensor_name);
    Serial.println(" - Calibration failed: no valid readings");
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Calibration");
    display.println("Failed!");
    display.println(sensor_name);
    display.display();
    delay(2000);
  }
}

// Function to equalize all ultrasonic sensors
void equalize_ultrasonic_sensors() {
  Serial.println("\n=== Ultrasonic Sensor Equalization ===");
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Sensor");
  display.println("Equalization");
  display.println("Starting...");
  display.display();
  delay(2000);
  
  // First calibration at 100cm
  float far_distance = 75.0;
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Step 1: Far Calibration");
  display.print("Place ALL sensors");
  display.println("");
  display.print("at ");
  display.print(far_distance);
  display.println(" cm");
  display.display();
  Serial.println("Place ALL sensors at exactly 75cm from a flat wall/object");
  delay(5000);
  
  // calibrate_sensor(trigPin_Lfront, echoPin_Lfront, cal_Lfront, far_distance, "L Front");
  // calibrate_sensor(trigPin_Rfront, echoPin_Rfront, cal_Rfront, far_distance, "R Front");
  calibrate_sensor(trigPin_left, echoPin_left, cal_left, far_distance, "Left");
  calibrate_sensor(trigPin_right, echoPin_right, cal_right, far_distance, "Right");
  
  // Second calibration at 30cm for offset adjustment
  float close_distance = 30.0;
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Step 2: Near Calibration");
  display.print("Place ALL sensors");
  display.println("");
  display.print("at ");
  display.print(close_distance);
  display.println(" cm");
  display.display();
  Serial.println("\nNow place ALL sensors at exactly 30cm from the wall");
  delay(5000);
  
  calibrate_sensor(trigPin_Lfront, echoPin_Lfront, cal_Lfront, close_distance, "L Front");
  calibrate_sensor(trigPin_Rfront, echoPin_Rfront, cal_Rfront, close_distance, "R Front");
  // calibrate_sensor(trigPin_left, echoPin_left, cal_left, close_distance, "Left");
  // calibrate_sensor(trigPin_right, echoPin_right, cal_right, close_distance, "Right");
  
  Serial.println("\n=== Equalization Complete ===");
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Equalization");
  display.println("Complete!");
  display.display();
  delay(2000);
  
  // Print calibration results
  display_calibration_results();
}

// Display calibration results on OLED
void display_calibration_results() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Calibration Data:");
  display.print("LF:");
  display.print(cal_Lfront.scale, 2);
  display.print(" RF:");
  display.println(cal_Rfront.scale, 2);
  display.print("L:");
  display.print(cal_left.scale, 2);
  display.print(" R:");
  display.println(cal_right.scale, 2);
  display.display();
  
  Serial.println("\n--- Calibration Results ---");
  Serial.print("Left Front: Scale=");
  Serial.print(cal_Lfront.scale, 3);
  Serial.print(", Offset=");
  Serial.println(cal_Lfront.offset, 1);
  
  Serial.print("Right Front: Scale=");
  Serial.print(cal_Rfront.scale, 3);
  Serial.print(", Offset=");
  Serial.println(cal_Rfront.offset, 1);
  
  Serial.print("Left: Scale=");
  Serial.print(cal_left.scale, 3);
  Serial.print(", Offset=");
  Serial.println(cal_left.offset, 1);
  
  Serial.print("Right: Scale=");
  Serial.print(cal_right.scale, 3);
  Serial.print(", Offset=");
  Serial.println(cal_right.offset, 1);
  Serial.println("------------------------\n");
}

void moving_forward(){
  Serial.print("moving forward");
  ADVANCE();
  delay(1100);
  STOP();
}

void first_align(){
  bool aligned = 0;
  Motor_PWM = 300;
  while (aligned == 0){
    float Lfront = measure_distance(trigPin_Lfront, echoPin_Lfront, cal_Lfront);
    float Rfront = measure_distance(trigPin_Rfront, echoPin_Rfront, cal_Rfront);
    float left  = measure_distance(trigPin_left, echoPin_left, cal_left);
    float right = measure_distance(trigPin_right, echoPin_right, cal_right);

    dis_ultra();
    delay(150);
    if ( (Lfront < 10) || (Rfront < 10) ){
      BACK();
      delay(250);
      STOP();
    }
    // while <200 is rudundant here, this is to make it clear that the sensor detects a wall
    if ( (left < 200) & (right < 200) & (left != 0) & (right != 0) & (left< 50) & (right< 50)){
      if (Lfront - Rfront > 1.5 ){
        // rotate_1();
        rotate_2();
        delay(200);
        STOP();
      }
      else if (Rfront - Lfront > 1.5) {
        // rotate_2();
        rotate_1();
        delay(300);
        STOP();
      }
    }
    else{
      delay(1000);
      float Lfront = measure_distance(trigPin_Lfront, echoPin_Lfront, cal_Lfront);
      float Rfront = measure_distance(trigPin_Rfront, echoPin_Rfront, cal_Rfront);
      float left  = measure_distance(trigPin_left, echoPin_left, cal_left);
      float right = measure_distance(trigPin_right, echoPin_right, cal_right);

      if ( (left>right) & (left>80) & (right!= 0) ) rotate_1();
      else if ( (right>left) & (right>80) & (left!= 0) )rotate_2();
      delay(200);
      STOP();
    }

    // below is the original and logically feasible way to detect distance from left or right wall. However, the ultrasonic sensors sometimes give weird results, so we ditch this way of implementation.

    // else if ((left > 100) || (right>100)){
    //   if (left>right) rotate_1();
    //   else rotate_2();
    //   delay(200);
    //   STOP();
    // }
    delay(600);

    if ( (Rfront - Lfront <2) || (Lfront - Rfront <2) ){
      aligned=1;
    }
  }

  float Lfront = measure_distance(trigPin_Lfront, echoPin_Lfront, cal_Lfront);
  float Rfront = measure_distance(trigPin_Rfront, echoPin_Rfront, cal_Rfront);
  float left  = measure_distance(trigPin_left, echoPin_left, cal_left);
  float right = measure_distance(trigPin_right, echoPin_right, cal_right);
  // if ( ((Lfront - Rfront <= 1) || (Rfront - Lfront <= 1)) && ((Lfront + Rfront) /2 <= 32) && (left <= 100) && (right <= 100)){
  //   aligned = 1;
    display.clearDisplay();
    display.print("aligned!");
  // }
}

void second_align(){
  // current_state = STATE_APPROACHING_FRONT;
  bool aligned = 0;
  Motor_PWM = 300;
  while (aligned == 0){
    float Lfront = measure_distance(trigPin_Lfront, echoPin_Lfront, cal_Lfront);
    float Rfront = measure_distance(trigPin_Rfront, echoPin_Rfront, cal_Rfront);
    float left  = measure_distance(trigPin_left, echoPin_left, cal_left);
    float right = measure_distance(trigPin_right, echoPin_right, cal_right);

    dis_ultra();
    delay(150);

    if(Lfront + Rfront >60) {
      ADVANCE();
      delay(150);
      STOP();
    }
    else if (Lfront + Rfront <56){
      BACK();
      delay(200);
      STOP();
    }
    else aligned = 1;

    delay(150);
    
  }

  float Lfront = measure_distance(trigPin_Lfront, echoPin_Lfront, cal_Lfront);
  float Rfront = measure_distance(trigPin_Rfront, echoPin_Rfront, cal_Rfront);
  float left  = measure_distance(trigPin_left, echoPin_left, cal_left);
  float right = measure_distance(trigPin_right, echoPin_right, cal_right);
  // if ( ((Lfront - Rfront <= 1) || (Rfront - Lfront <= 1)) && ((Lfront + Rfront) /2 <= 32) && (left <= 100) && (right <= 100)){
  //   aligned = 1;
    display.clearDisplay();
    display.print("aligned!");
  // }
}

void third_align(){
  // current_state = STATE_ALIGNING_WALLS;
  Serial.print("third align");
  bool aligned = 0;
  Motor_PWM = 300;
  while (aligned == 0){
    float Lfront = measure_distance(trigPin_Lfront, echoPin_Lfront, cal_Lfront);
    float Rfront = measure_distance(trigPin_Rfront, echoPin_Rfront, cal_Rfront);
    float left  = measure_distance(trigPin_left, echoPin_left, cal_left);
    float right = measure_distance(trigPin_right, echoPin_right, cal_right);

    dis_ultra();
    delay(150);

    if(left > 64) {
      LEFT_2();
      delay(300);
      STOP();
    }
    // else if (right > 57){
      else if (left < 60){
      RIGHT_2();
      delay(300);
      STOP();
    }
    // else if(right<= 57 & left<=57){
      else if(left>= 60 & left<=64){
      aligned = 1;
    }

    delay(150);
    
  }

  float Lfront = measure_distance(trigPin_Lfront, echoPin_Lfront, cal_Lfront);
  float Rfront = measure_distance(trigPin_Rfront, echoPin_Rfront, cal_Rfront);
  float left  = measure_distance(trigPin_left, echoPin_left, cal_left);
  float right = measure_distance(trigPin_right, echoPin_right, cal_right);
  // if ( (Lfront - Rfront >= 2) || (Rfront - Lfront >=2)){
  //   current_state = STATE_APPROACHING_FRONT;
  // }
  // if ( ((Lfront - Rfront <= 1) || (Rfront - Lfront <= 1)) && ((Lfront + Rfront) /2 <= 32) && (left <= 100) && (right <= 100)){
  //   aligned = 1;
    // display.clearDisplay();
    // display.print("aligned!");
  // }
}

String readColor() {
  int red, green;
  
  // Read RED
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);
  delay(100);
  // red = pulseIn(out, LOW, 50000);
  red = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH); 
  delay(20);
  
  // Read GREEN
  digitalWrite(s2, HIGH);
  digitalWrite(s3, HIGH);
  delay(100);
  // green = pulseIn(out, LOW, 50000);
  green = pulseIn(out,  digitalRead(out) == HIGH ? LOW : HIGH);
  delay(20);
  
  Serial.print("R:");
  Serial.print(red);
  Serial.print(" G:");
  Serial.print(green);
  
  // Lower value = more of that color
  if (red < green) {
    Serial.println(" -> RED");
    return "RED";
  } else if (green < red) {
    Serial.println(" -> GREEN");
    return "GREEN";
  } else {
    Serial.println(" -> UNKNOWN");
    return "UNKNOWN";
  }
}

void turnRight(){
  Motor_PWM = 300;
  ADVANCE();
  delay(350);
  STOP();
  Motor_PWM=320;
  rotate_2();
  delay(700);
  STOP();
}

void turnLeft(){
  Motor_PWM = 300;
  ADVANCE();
  delay(300);
  STOP();
  delay(500);
  Motor_PWM=320;
  rotate_1();
  delay(700);
  STOP();
  delay(800);
}

void decidecolor() {
  current_state = STATE_COLOR_DETECTION;
  int red1, green1, red2, green2;
  String color1, color2;
  
  // FIRST READING
  Serial.println("\n--- READING 1 ---");
  // display.clearDisplay();
  // display.println("\n--- READING 1 ---");
  // display.display();
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);
  delay(100);
  red1 = pulseIn(out, LOW, 50000);
  // red1 = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH, 50000); 
  
  digitalWrite(s2, HIGH);
  digitalWrite(s3, HIGH);
  delay(100);
  green1 = pulseIn(out, LOW, 50000);
  // green1 = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH, 50000); 
  
  if (red1 < 1000 && green1 < 1000) color1 = "UNKNOWN";
  else if(red1 < green1) color1 = "RED";
  else color1 = "GREEN";
  Serial.print("R:");
  Serial.print(red1);
  Serial.print(" G:");
  Serial.print(green1);
  Serial.print(" -> ");
  Serial.println(color1);

  // display.clearDisplay();
  // display.print("R:");
  // display.print(red1);
  // display.print(" G:");
  // display.print(green1);
  // display.print(" -> ");
  // display.println(color1);
  // display.display();
  
  // Wait 5 seconds
  Serial.println("Waiting 5 seconds...");
  delay(5000);
  
  // SECOND READING
  Serial.println("\n--- READING 2 ---");
  // display.clearDisplay();
  // display.println("\n--- READING 2 ---");
  // display.display();
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);
  delay(100);
  red2 = pulseIn(out, LOW, 50000);
  // red2 = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH, 50000); 
  // delay(20);
  
  digitalWrite(s2, HIGH);
  digitalWrite(s3, HIGH);
  delay(100);
  green2 = pulseIn(out, LOW, 50000);
  // green2 = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH, 50000); 
  
  // color2 = (red2 < green2) ? "RED" : "GREEN";
  if (red2 < 1000 && green2 < 1000) color2 = "UNKNOWN";
  else if(red2 < green2) color2 = "RED";
  else color2 = "GREEN";
  Serial.print("R:");
  Serial.print(red2);
  Serial.print(" G:");
  Serial.print(green2);
  Serial.print(" -> ");
  Serial.println(color2);

  // display.clearDisplay();
  // display.print("R:");
  // display.print(red2);
  // display.print(" G:");
  // display.print(green2);
  // display.print(" -> ");
  // display.println(color2);
  // display.display();
  
  // FINAL DECISION AND MOVEMENT
  Serial.println("\n=== FINAL DECISION ===");
  Serial.print("Reading 1: ");
  Serial.println(color1);
  Serial.print("Reading 2: ");
  Serial.println(color2);

  // display.clearDisplay();
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("\n=== FINAL DECISION ===");
  display.print("Reading 1: ");
  display.println(color1);
  display.print("Reading 2: ");
  display.println(color2);
  display.display();
  
  if (color1 == "RED" && color2 == "RED") {
    Serial.println("Both RED - Turning RIGHT");
    turnRight();
    current_state = STATE_READY_TO_CHOOSE;
  }
  else if (color1 == "GREEN" && color2 == "GREEN") {
    Serial.println("Both GREEN - Turning LEFT");
    turnLeft();
    current_state = STATE_READY_TO_CHOOSE;
  }
  else if (color1 == "RED" && color2 == "GREEN") {
    Serial.println("Mixed (RED then GREEN) - Turning RIGHT");
    turnRight();
    current_state = STATE_READY_TO_CHOOSE;
  }
  else if (color1 == "GREEN" && color2 == "RED") {
    Serial.println("Mixed (GREEN then RED) - Turning LEFT");
    turnLeft();
    current_state = STATE_READY_TO_CHOOSE;
  }
  else {
    Serial.println("Unknown colors - No movement");
    current_state = STATE_APPROACHING_FRONT;
  }
  
  Serial.println("=====================");
  
  // Wait before next cycle
  dis_ultra();
  delay(1000);
}

void movefront() {
  Motor_PWM = 300;
  ADVANCE();
  delay(2000);
  STOP();
  delay(500);
  // first_align();
}

// Improved dis_ultra function with better OLED formatting
void dis_ultra() {
  // Use calibrated measurements
  float Lfront = measure_distance(trigPin_Lfront, echoPin_Lfront, cal_Lfront);
  float Rfront = measure_distance(trigPin_Rfront, echoPin_Rfront, cal_Rfront);
  float left = measure_distance(trigPin_left, echoPin_left, cal_left);
  float right = measure_distance(trigPin_right, echoPin_right, cal_right);
  
  // Light sensor readings
  // int_left = (analogRead(A5) - int_adc0_c) / int_adc0_m;
  int_right = (analogRead(A6) - int_adc1_c) / int_adc1_m;
  
  // Format distances for display
  int distance_Lfront_cm = (Lfront < 500) ? (int)Lfront : 999;
  int distance_Rfront_cm = (Rfront < 500) ? (int)Rfront : 999;
  int distance_left_cm = (left < 500) ? (int)left : 999;
  int distance_right_cm = (right < 500) ? (int)right : 999;
  
  // Print to Serial Monitor
  Serial.print("FL: ");
  if (distance_Lfront_cm < 500) Serial.print(distance_Lfront_cm);
  else Serial.print("---");
  Serial.print("cm | FR: ");
  if (distance_Rfront_cm < 500) Serial.print(distance_Rfront_cm);
  else Serial.print("---");
  Serial.print("cm | L: ");
  if (distance_left_cm < 500) Serial.print(distance_left_cm);
  else Serial.print("---");
  Serial.print("cm | R: ");
  if (distance_right_cm < 500) Serial.print(distance_right_cm);
  else Serial.print("---");
  Serial.print("cm | Light L: ");
  // Serial.print(analogRead(A5));
  Serial.print(" R: ");
  Serial.print(analogRead(A6));
  Serial.print(" | State: ");
  Serial.println(current_state);
  
  // Update OLED Display with better formatting
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  
  // Row 1: Front sensors
  display.print("FL:");
  if (distance_Lfront_cm < 500) {
    display.print(distance_Lfront_cm);
    display.print("cm ");
  } else {
    display.print("--- ");
  }
  
  display.print("FR:");
  if (distance_Rfront_cm < 500) {
    display.print(distance_Rfront_cm);
    display.println("cm");
  } else {
    display.println("---");
  }
  
  // Row 2: Side sensors
  display.print("L:");
  if (distance_left_cm < 500) {
    display.print(distance_left_cm);
    display.print("cm ");
  } else {
    display.print("--- ");
  }
  
  display.print("R:");
  if (distance_right_cm < 500) {
    display.print(distance_right_cm);
    display.println("cm");
  } else {
    display.println("---");
  }
  
  // Row 3: Light sensors
  display.print("Lgt L:");
  // display.print(analogRead(A5));
  display.print(" R:");
  display.println(analogRead(A6));
  
  // Row 4: State and calibration status
  display.print("State: ");
  switch(current_state) {
    case STATE_SEEKING_LIGHT:
      display.print("SEEKING");
      break;
    case STATE_APPROACHING_FRONT:
      display.print("APPROACH");
      break;
    case STATE_ALIGNING_WALLS:
      display.print("ALIGN");
      break;
    case STATE_COLOR_DETECTION:
      display.print("COLOR");
      break;
    case STATE_READY_TO_CHOOSE:
      display.print("READY");
      break;
    default:
      display.print("?");
  }

  
  display.display();
}

// Setup function
void setup() {
  Serial.begin(115200);
  Serial.print("hello!");
  
  // Initialize ultrasonic sensor pins
  pinMode(trigPin_Lfront, OUTPUT);
  pinMode(echoPin_Lfront, INPUT);
  pinMode(trigPin_Rfront, OUTPUT);
  pinMode(echoPin_Rfront, INPUT);
  pinMode(trigPin_left, OUTPUT);
  pinMode(echoPin_left, INPUT);
  pinMode(trigPin_right, OUTPUT);
  pinMode(echoPin_right, INPUT);
  
  Serial.print("testing");
  
  // Initialize I2C and MPU6050
  Wire.begin();
  //mpu.begin();
  //mpu.calcGyroOffsets();
  
  // Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.cp437(true);
  display.setCursor(0, 0);
  display.println("Ready");
  display.display();

  //color sensor
  pinMode(s0,OUTPUT);
  pinMode(s1,OUTPUT);
  pinMode(s2,OUTPUT);
  pinMode(s3,OUTPUT);
  pinMode(out, INPUT_PULLUP);
  digitalWrite(s0,LOW);
  digitalWrite(s1,HIGH);

  
  // Ultrasonic sensor calibration option
  Serial.println("\n=== Ultrasonic Sensor Setup ===");
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Ultrasonic");
  display.println("Calibration?");
  display.println("Send 'C' over");
  display.println("Serial to calibrate");
  display.display();
  
  // Wait for calibration command
  Serial.println("Send 'C' to calibrate ultrasonic sensors, or wait 5 seconds to continue");
  long start_time = millis();
  bool calibrate_ultra = false;

  
  if (calibrate_ultra) {
    equalize_ultrasonic_sensors();
  } else {
    // Use default calibration values
    cal_Lfront = {0, 1.0, true};
    cal_Rfront = {0, 1.0, true};
    cal_left = {0, 1.0, true};
    cal_right = {0, 1.0, true};
    Serial.println("Using default calibration (1.0)");
    display_calibration_results();
    delay(2000);
  }
  
  Serial.print("System ready");
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println("System");
  display.println("Ready!");
  display.display();
  // delay(2000);
}

// Main loop
void loop() {
  switch(current_state) {
    case STATE_SEEKING_LIGHT:
      moving_forward();
      delay(3000);
      // first_align();
      // delay(1000);
      // float Lfront = measure_distance(trigPin_Lfront, echoPin_Lfront, cal_Lfront);
      // float Rfront = measure_distance(trigPin_Rfront, echoPin_Rfront, cal_Rfront);
      // float left = measure_distance(trigPin_left, echoPin_left, cal_left);
      // float right = measure_distance(trigPin_right, echoPin_right, cal_right);
      // if( (left>=100 || left==0) || (right>=100 || right==0)){
      //   first_align();
      // }
      // delay(1000);
      current_state = STATE_APPROACHING_FRONT;
      break;
    case STATE_APPROACHING_FRONT:
      first_align();
      delay(1000);
      second_align();
      delay(1000);
      // first_align();
      // delay(1000);
      current_state = STATE_ALIGNING_WALLS;
      break;
    case STATE_ALIGNING_WALLS:
      third_align();
      delay(2000);
      current_state = STATE_COLOR_DETECTION;
      break;
    case STATE_COLOR_DETECTION:
    display.print("Finding color...");
      decidecolor();
      delay(1000);
      break;
    case STATE_READY_TO_CHOOSE:
      Serial.print("ready to choose");
      movefront();
      delay(1000);
      while(1){
        dis_ultra();
        delay(100);
      }
      delay(10000);
      break;
    default:
      display.print("?");
  }
  // Motor_PWM=300;
  // LEFT_2();
  // delay(3000);
  // moving_forward();
  // delay(3000);
  // first_align();
  // delay(1000);
  // second_align();
  // delay(1000);
  // first_align();
  // delay(1000);
  // third_align();
  // delay(2000);
  // decidecolor();
  // delay(5000);
}

// OLED display function
void print_OLED(String text, long int text2) {
  display.clearDisplay();
  display.setTextSize(1.5);
  display.setTextColor(SSD1306_WHITE);
  display.cp437(true);
  display.setCursor(0, 0);
  display.println(text);
  if (text2 != 0) {
    display.println(text2);
  }
  display.display();
}

void print_OLED(String text, int text2) {
  display.clearDisplay();
  display.setTextSize(1.5);
  display.setTextColor(SSD1306_WHITE);
  display.cp437(true);
  display.setCursor(0, 0);
  display.println(text);
  display.println(text2);
  display.display();
}
