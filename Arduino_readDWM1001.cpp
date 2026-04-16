// ============================================================
// Arduino Mega - Read DWM1001 via Serial1
// ============================================================

void setup() {
  // Serial to computer (USB)
  Serial.begin(115200);
  
  // Serial1 to DWM1001 (pins 19=RX1, 18=TX1)
  Serial1.begin(115200);
  
  Serial.println("Arduino Mega ready");
  Serial.println("Waiting for DWM1001 data...");
  
  // Optional: Configure DWM1001 from Arduino
  delay(2000);
  Serial1.println("le");
  delay(100);
  Serial1.println("tn");
  delay(100);
  Serial1.println("save");
  delay(100);
  Serial1.println("reset");
}

void loop() {
  // Forward data from DWM1001 to Serial Monitor
  if (Serial1.available()) {
    String data = Serial1.readStringUntil('\n');
    Serial.print("UWB: ");
    Serial.println(data);
    
    // Parse distance if it's ranging data
    if (data.startsWith("R")) {
      parseRangingData(data);
    }
  }
  
  // Send commands from Serial Monitor to DWM1001
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    Serial1.println(cmd);
    Serial.print("Sent: ");
    Serial.println(cmd);
  }
}

void parseRangingData(String data) {
  // Format: R,0,1.234,0.001,0,0
  int comma1 = data.indexOf(',');
  int comma2 = data.indexOf(',', comma1 + 1);
  int comma3 = data.indexOf(',', comma2 + 1);
  
  if (comma1 != -1 && comma2 != -1 && comma3 != -1) {
    String distance_str = data.substring(comma2 + 1, comma3);
    float distance = distance_str.toFloat();
    
    Serial.print("Distance to anchor: ");
    Serial.print(distance, 3);
    Serial.println(" meters");
  }
}
