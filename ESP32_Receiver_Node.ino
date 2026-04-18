void setup() {
  Serial.begin(115200);
  
  Serial2.begin(9600);

  Serial.println("Receiver ESP32 Waiting for data...");
}

void loop() {
  if (Serial2.available()) {
    String incomingData = Serial2.readStringUntil('\n');
    
    int commaIndex = incomingData.indexOf(',');
    
    if (commaIndex > 0) {
      String tempPart = incomingData.substring(0, commaIndex);
      String distPart = incomingData.substring(commaIndex + 1);

      float finalTemp = tempPart.toFloat();
      float finalDist = distPart.toFloat();

      Serial.println("========== DATA RECEIVED ==========");
      Serial.print("Water Temperature: "); Serial.print(finalTemp); Serial.println(" C");
      Serial.print("Water Distance   : "); Serial.print(finalDist); Serial.println(" cm");
      Serial.println("===================================");
    }
  }
}