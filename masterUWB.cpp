// ============================================================
// MASTER ROBOT - Just exists, UWB module does the work
// No motor control needed
// ============================================================

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  
  // Configure DWM1001 as tag
  Serial1.println("le");
  delay(100);
  Serial1.println("tn");
  delay(100);
  Serial1.println("save");
  delay(100);
  Serial1.println("reset");
  
  Serial.println("Master Robot Active");
  Serial.println("UWB Tag Broadcasting");
}

void loop() {
  // Forward any UWB data to serial for debugging
  if (Serial1.available()) {
    Serial.write(Serial1.read());
  }
  
  // Master doesn't need to do anything else
  delay(100);
}
