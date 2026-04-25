#include <OneWire.h>
#include <DallasTemperature.h>

#define TRIG_PIN 12
#define ECHO_PIN 13

#define ONE_WIRE_BUS 21
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

void setup() {
  Serial.begin(115200);
  
  Serial2.begin(9600, SERIAL_8N1, 16, 17);
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  sensors.begin();

  Serial.println("Sender ESP32 Started...");
}

float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return -1;
  
  return (duration * 0.1500) / 2;
}

void loop() {
  sensors.requestTemperatures();
  float temp = sensors.getTempCByIndex(0);

  float dist = getDistance();

  Serial2.print(temp);
  Serial2.print(",");
  Serial2.println(dist);

  Serial.print("Sending Data -> Temp: ");
  Serial.print(temp);
  Serial.print(" C, Dist: ");
  Serial.print(dist);
  Serial.println(" cm");

  delay(2000);   
}