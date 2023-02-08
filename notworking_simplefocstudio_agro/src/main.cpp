#include <Arduino.h>
#include <SimpleFOC.h>

BLDCMotor motor(7);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003, -64.0/7.0, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);
HallSensor sensor(A_HALL1, A_HALL2, A_HALL3, 7);
void doA(){sensor.handleA();}
void doB(){sensor.handleB();}
void doC(){sensor.handleC();}

Commander command(Serial); // include commander interface
void doMotor(char* cmd) { command.motor(&motor, cmd); }

void setup(){

  // add the motor to the commander interface
  // The letter (here 'M') you will provide to the SimpleFOCStudio
  command.add('M',doMotor, (char*)"motor");
  motor.useMonitoring(Serial); // tell the motor to use the monitoring
  motor.monitor_downsample = 0; // disable monitor at first - optional
  motor.init();
  motor.initFOC();

}
void loop(){
  motor.monitor(); // real-time monitoring calls
  command.run(); // real-time commander calls
}