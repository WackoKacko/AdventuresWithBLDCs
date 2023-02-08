#include <Arduino.h>
#include <SimpleFOC.h>

// #define   _MON_TARGET 0b1000000  // monitor target value
// #define   _MON_VOLT_Q 0b0100000  // monitor voltage q value
// #define   _MON_VOLT_D 0b0010000  // monitor voltage d value
// #define   _MON_CURR_Q 0b0001000  // monitor current q value - if measured
// #define   _MON_CURR_D 0b0000100  // monitor current d value - if measured
// #define   _MON_VEL    0b0000010  // monitor velocity value
// #define   _MON_ANGLE  0b0000001  // monitor angle value

// float target = 500.0;

// void serialLoop()
// {
//   static String received_chars;

//   if (Serial.available() > 0)
//   {
//     String command = Serial.readString();
//     target = command.toFloat();
//     Serial.print("Target = ");
//     Serial.println(target);
//     command = "";
//   }
// }

BLDCMotor motor(7, 0.0361, 272); //7 is correct. got info that KV should be typed in as 50% to 70% higher than its provided value. Find here: https://docs.simplefoc.com/bldcmotor
// BLDCMotor motor(7); // this works, too
BLDCDriver6PWM driver(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL); //got this from here: https://github.com/simplefoc/Arduino-FOC/blob/master/examples/hardware_specific_examples/B_G431B_ESC1/B_G431B_ESC1.ino
LowsideCurrentSense currentSense(0.003, -64.0 / 7.0, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT); //got this from here: https://github.com/simplefoc/Arduino-FOC/blob/master/examples/hardware_specific_examples/B_G431B_ESC1/B_G431B_ESC1.ino

HallSensor sensor(A_HALL1, A_HALL2, A_HALL3, 7); // hallpin1, hallpin2, hallpin3, #ofPolePairs
// Interrupt routine initialization + callbacks
void doA() { sensor.handleA(); }
void doB() { sensor.handleB(); }
void doC() { sensor.handleC(); }

float prev_angle=0;
float prev_velocity=0;
float prev_target=0;

// Commander command(Serial);
// void doTarget(char *cmd) { command.motion(&motor, cmd); }
// void doMotor(char *cmd) { command.motion(&motor, cmd); }

void setup()
{
  Serial.begin(115200);
  delay(100);
  motor.useMonitoring(Serial);
  motor.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE; //specifies which variables get output
  motor.monitor_downsample = 100; // downsampling, or how many loops between each communication with Serial? default 10. Seems like higher is better, so less latency.

  sensor.enableInterrupts(doA, doB, doC); // hardware interrupt enable
  sensor.init();   
  motor.linkSensor(&sensor);                       // initialise sensor hardware

  driver.voltage_power_supply = 24;
  driver.init();
  motor.linkDriver(&driver);
  currentSense.linkDriver(&driver);

  currentSense.skip_align = true; // important?
  currentSense.init();
  motor.linkCurrentSense(&currentSense);

  // motor.voltage_sensor_align = 3;  // aligning voltage [V] ???
  motor.voltage_limit = 0.0361; //???
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::velocity_openloop;
  motor.init();
  motor.initFOC();

  // command.add('M', doMotor, (char*)"motor");
  // command.add('T', doTarget, (char*)"target");
  
  Serial.println("setup done");
}

void loop()
{
  if(motor.shaft_angle != prev_angle | motor.shaft_velocity != prev_velocity | motor.target != prev_target) {
    prev_angle = motor.shaft_angle; prev_velocity = motor.shaft_velocity; prev_target = motor.target;
    Serial.print(prev_angle); Serial.print("\t"); Serial.print(prev_velocity); Serial.print("\t"); Serial.println(prev_target);
  }
  // if(sensor.getAngle())
  // motor.monitor(); //updates motor monitor
  // command.run(); // not working. ignore for now

  // serialLoop();
  // motor.move(target); // why won't this work??
  // motor.loopFOC();    // why won't this work??
  // Serial.print(sensor.getAngle());      // this works fine
  // Serial.print("\t");                   // this works fine
  // Serial.println(sensor.getVelocity()); // this works fine
}
