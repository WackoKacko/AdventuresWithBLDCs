#include <Arduino.h>
#include <SimpleFOC.h>

float target = 0.0;

void serialLoop() {
  static String received_chars;

  if (Serial.available() > 0) {
    String command = Serial.readString();
    target = command.toFloat();
    Serial.print("Target = "); Serial.println(target);
    command = ""  ;
    }
}

HallSensor sensor(A_HALL1, A_HALL2, A_HALL3, 7); // hallpin1, hallpin2, hallpin3, #ofPolePairs
// Interrupt routine initialization + callbacks
void doA() { sensor.handleA(); }
void doB() { sensor.handleB(); }
void doC() { sensor.handleC(); }


void setup() {
  sensor.init();                          // initialise sensor hardware
  sensor.enableInterrupts(doA, doB, doC); // hardware interrupt enable

  Serial.begin(115200);
  delay(100);
  Serial.println("setup done");
}

void loop() {
  serialLoop();
  Serial.print(sensor.getAngle());
  Serial.print("\t");
  Serial.println(sensor.getVelocity());
}
