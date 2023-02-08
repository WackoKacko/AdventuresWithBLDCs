#include <Arduino.h>
#include <SimpleFOC.h>

float target = 500.0;

void serialLoop()
{
  static String received_chars;

  if (Serial.available() > 0)
  {
    String command = Serial.readString();
    target = command.toFloat();
    Serial.print("Target = ");
    Serial.println(target);
    command = "";
  }
}

// BLDCMotor motor(7, 0.0361, 340); //7 is correct. not specifying phase resistance or KV yet. Also, got info that KV should be typed in as 100% to 200% of its true value. Find here: 
BLDCMotor motor(7); // 7 is correct.
BLDCDriver6PWM driver(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL); //got this from here: 
LowsideCurrentSense currentSense(0.003, -64.0 / 7.0, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT); //got this from here:

HallSensor sensor(A_HALL1, A_HALL2, A_HALL3, 7); // hallpin1, hallpin2, hallpin3, #ofPolePairs
// Interrupt routine initialization + callbacks
void doA() { sensor.handleA(); }
void doB() { sensor.handleB(); }
void doC() { sensor.handleC(); }

// Commander command(Serial);
// void doTarget(char *cmd) { command.motion(&motor, cmd); }
// void doMotor(char *cmd) { command.motion(&motor, cmd); }

void setup()
{

  sensor.enableInterrupts(doA, doB, doC); // hardware interrupt enable
  sensor.init();                          // initialise sensor hardware

  driver.voltage_power_supply = 24;
  driver.init();
  motor.linkDriver(&driver);
  currentSense.linkDriver(&driver);

  currentSense.skip_align = true; // important?
  currentSense.init();
  motor.linkCurrentSense(&currentSense);
  // motor.voltage_sensor_align = 3;  // aligning voltage [V] ???
  // motor.velocity_index_search = 3; // index search velocity [rad/s] ???

  motor.voltage_limit = 0.0361; //???
  // motor.voltage_sensor_align = 1;
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::velocity_openloop;
  motor.init();
  motor.initFOC();

  // command.add('M', doMotor, (char*)"motor");
  // command.add('T', doTarget, (char*)"target");
  motor.useMonitoring(Serial);
  // motor.monitor_downsample = 0; // disable monitor at first - optional

  Serial.begin(115200);
  delay(100);
  Serial.println("setup done");
}

void loop()
{
  // motor.monitor(); //not working. ignore for now
  // command.run(); // not working. ignore for now

  // serialLoop();
  motor.move(target); // why won't this work??
  motor.loopFOC();    // why won't this work??
  // Serial.print(sensor.getAngle());      // this works fine
  // Serial.print("\t");                   // this works fine
  // Serial.println(sensor.getVelocity()); // this works fine
}
