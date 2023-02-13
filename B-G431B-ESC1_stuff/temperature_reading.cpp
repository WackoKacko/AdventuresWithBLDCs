#include <Arduino.h>
#include <SimpleFOC.h>

double temp = 0;
unsigned long time_stamp = 0;

void setup() {
  Serial.begin(115200);
  pinMode(A_TEMPERATURE, INPUT);
}

void loop() {
  if(millis()-time_stamp > 500) {
    temp = map(analogRead(A_TEMPERATURE),0,1024,-49,125);
    Serial.println(temp);
    time_stamp = millis();
  }
}