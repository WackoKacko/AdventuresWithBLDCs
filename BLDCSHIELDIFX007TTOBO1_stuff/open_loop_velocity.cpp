#include <Arduino.h>
#include <SimpleFOC.h>

BLDCMotor motor(7, 0.025, 2400); //true KV 1500, 2400 = 1500 + 60%, as recommended by SimpleFOC docs
BLDCDriver3PWM driver(11,6,10,5,9,3); //INU, INHU, INV, INHV, INW, INHW
InlineCurrentSense current_sense(???); 
/* "Due to the limited pinout of the Arduino shield all IS pins of the three devices are connected together.
    According to different applications the value of R4 could be adjusted to achieve a better current sense
    performance" */

Commander commander(Serial);
void doMotor(char *cmd) { commander.motion(&motor, cmd); }
void doTarget(char *cmd) { commander.motion(&motor, cmd); }
void doLimitCurrent(char *cmd) { commander.scalar(&motor.current_limit, cmd); }

void setup() {
  Serial.begin(115200);
  delay(3500);
  motor.useMonitoring(Serial);

  driver.voltage_power_supply = 12;
  driver.voltage_limit = 0.5; //V=IR -> (20A)*(0.025ohms)=0.5V
  // driver.pwm_frequency = 15000; //15kHz limit because of IFX007 limited speed...I don't think setting pwm works for the Arduino Uno at all, default is 32kHz
  driver.init(); //essential
  motor.linkDriver(&driver); //essential

  current_sense.linkDriver(&driver); //essential
  current_sense.init(); //essential
  motor.linkCurrentSense(&current_sense); //essential

  // try with this later
  // motor.PID_velocity.P = 0.2; //this should be fine, I think.
  // motor.PID_velocity.I = 11; //keep above 10, I think.
  // motor.LPF_velocity.Tf = 0.01; //smooths out noisy sensor signal

  motor.current_limit = 1.2; // measured in Amps. IF THIS IS TOO LOW, YOUR MOTOR WON'T SPIN.
  motor.torque_controller = TorqueControlType::voltage; //try foc_current next; there IS inline current sensing
  motor.controller = MotionControlType::velocity_openloop;
  
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