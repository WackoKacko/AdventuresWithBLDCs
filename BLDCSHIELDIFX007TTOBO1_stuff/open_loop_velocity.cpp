#include <Arduino.h>
#include <SimpleFOC.h>

BLDCMotor motor(7, 0.08, 300); //true KV 1500. 300 = 200 + 50%, as recommended by SimpleFOC docs
BLDCDriver3PWM driver(11,10,9,6,5,3); //INU, INV, INW, INHU, INHV, INHW
// BLDCDriver3PWM driver(11,6,10,5,9,3); //INU, INHU, INV, INHV, INW, INHW
// InlineCurrentSense current_sense(???); //board has high-side current sensing, which SimpleFOC doesn't support at the moment.
/* "Due to the limited pinout of the Arduino shield all IS pins of the three devices are connected together.
    According to different applications the value of R4 could be adjusted to achieve a better current sense
    performance" */ // ^Useless anyways when IS cannot be read with SimpleFOC.

HallSensor sensor(PC3, PC2, PC1, 7); // hallpin1, hallpin2, hallpin3, #ofPolePairs
// Interrupt routine initialization + callbacks
void doA() { sensor.handleA(); }
void doB() { sensor.handleB(); }
void doC() { sensor.handleC(); }

Commander commander(Serial);
void doMotor(char *cmd) { commander.motion(&motor, cmd); }
void doTarget(char *cmd) { commander.motion(&motor, cmd); }
void doLimitCurrent(char *cmd) { commander.scalar(&motor.current_limit, cmd); }

void setup() {
  Serial.begin(115200);
  delay(3500);
  motor.useMonitoring(Serial);

  sensor.init();                          // initialise sensor hardware
  sensor.enableInterrupts(doA, doB, doC); // hardware interrupt enable
  motor.linkSensor(&sensor);   

  driver.voltage_power_supply = 12;
  driver.voltage_limit = 0.8; //V=IR -> (10A)*(0.08ohms)=0.8V
  // driver.pwm_frequency = 15000; //15kHz limit because of IFX007 limited speed...I don't think setting pwm works for the Arduino Uno at all, default is 32kHz
  driver.init(); //essential
  motor.linkDriver(&driver); //essential

  // current_sense.linkDriver(&driver); //essential
  // current_sense.init(); //essential
  // motor.linkCurrentSense(&current_sense); //essential

  // try with this later
  // motor.PID_velocity.P = 0.2; //this should be fine, I think.
  // motor.PID_velocity.I = 11; //keep above 10, I think.
  // motor.LPF_velocity.Tf = 0.01; //smooths out noisy sensor signal
  // motor.voltage_sensor_align = 0.03;
  motor.voltage_limit = 0.1; //V=IR -> (1.25A)*(0.08ohms)=0.1V 
  // motor.current_limit = 5; // measured in Amps. IF THIS IS TOO LOW, YOUR MOTOR WON'T SPIN.
  motor.torque_controller = TorqueControlType::voltage; //try foc_current next; there IS inline current sensing
  motor.controller = MotionControlType::velocity;
  motor.initFOC(); //essential
  motor.init(); //essential

  commander.add('M', doMotor, (char *)"motor");   // this is the connection control command SimpleFOCStudio (GUI) needs to send to connect and work...haven't gotten it to work yet :) 
  commander.add('T', doTarget, (char *)"target"); // type in "T20" to set the target to 20. Type in "T50" to set the target to 50, etc.
  commander.add('C', doLimitCurrent, (char *)"current"); //type in "C1" to set the current limit to 1A. Type in "C10" to set the current limit to 10A, etc.

  Serial.println("setup done");
}

void loop() {
  motor.loopFOC(); //this needs to run. always.
  motor.move(); //this needs to run. always.
  // motor.monitor(); //enable if you wish. with downsampling of 500, shouldn't be too epileptic and shouldn't affect performance. lower downsampling if you want more data (and more latency)
  commander.run(); //this never hurts. much neater than writing one's own "if(Serial.available()>0) { String command = Serial.readString(); ...}"
}
