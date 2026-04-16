#include <Servo.h>

// ================== PIN & SETTINGS ==================
const int servoPin = 48;        // PL1 on your servo shield

Servo boxServo;

bool boxOpen = false;

void setup() {
  boxServo.attach(servoPin);
  boxServo.write(0);           // Start with box closed
  
  Serial.begin(9600);
  Serial.println("=== Box Control System Ready ===");
  Serial.println("Send 'open' or 'close' in Serial Monitor");
}

void loop() {
  
  // Check for commands from Serial Monitor (for testing)
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "open" && !boxOpen) {
      openBox();
    }
    else if (command == "close" && boxOpen) {
      closeBox();
    }
  }
}

// ================== CONTROL FUNCTIONS ==================

void openBox() {
  Serial.println("Opening Box...");
  
  // Smooth opening
  for (int pos = 0; pos <= 90; pos += 2) {
    boxServo.write(pos);
    delay(20);               // Change 20 to adjust speed
  }
  
  boxOpen = true;
  Serial.println("Box Opened");
}

void closeBox() {
  Serial.println("Closing Box...");
  
  // Smooth closing
  for (int pos = 90; pos >= 0; pos -= 2) {
    boxServo.write(pos);
    delay(20);
  }
  
  boxOpen = false;
  Serial.println(“Box Closed");
}
