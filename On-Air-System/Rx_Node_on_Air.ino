#include <esp_now.h>
#include <WiFi.h>

typedef struct struct_message {
    int distance;
} struct_message;

struct_message incomingData;

void OnDataRecv(const esp_now_recv_info_t * recv_info, const uint8_t *incomingDataRaw, int len) {
  
  memcpy(&incomingData, incomingDataRaw, sizeof(incomingData));
  
  Serial.print("Distance Received: ");
  Serial.print(incomingData.distance);
  Serial.println(" cm");
  
  if(incomingData.distance < 20 && incomingData.distance > 0) {
      Serial.println("⚠️ Warning: Object too close!");
  }
}

void setup() {
  Serial.begin(115200);
  
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_recv_cb((esp_now_recv_cb_t)OnDataRecv);
  
  Serial.println("Receiver Ready. Waiting for data...");
}
 
void loop() {
}