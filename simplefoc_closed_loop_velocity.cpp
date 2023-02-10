#include <Arduino.h>
#include <SimpleFOC.h>

// using a little, sensorless outrunner. Should be capable of at least 300W.
BLDCMotor motor(7, 0.08, 270);                                                               // hmm let's mess around with KV. they say it's usually 150%-170% of datasheet value but they also say it's 100%-200%.
BLDCDriver6PWM driver(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL); // got this from here: https://github.com/simplefoc/Arduino-FOC/blob/master/examples/hardware_specific_examples/B_G431B_ESC1/B_G431B_ESC1.ino
LowsideCurrentSense current_sense(0.003, -64.0 / 7.0, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);        // got this from here: https://github.com/simplefoc/Arduino-FOC/blob/master/examples/hardware_specific_examples/B_G431B_ESC1/B_G431B_ESC1.ino

HallSensor sensor(A_HALL1, A_HALL2, A_HALL3, 7); // hallpin1, hallpin2, hallpin3, #ofPolePairs
void doA() { sensor.handleA(); } // Interrupt routine initialization + callbacks
void doB() { sensor.handleB(); } // ^
void doC() { sensor.handleC(); } // ^

Commander commander(Serial);
void doMotor(char *cmd) { commander.motion(&motor, cmd); }
void doTarget(char *cmd) { commander.motion(&motor, cmd); }
void doLimitCurrent(char *cmd) { commander.scalar(&motor.current_limit, cmd); }

float target = 0;
float prev_val = 0;
float temp = 0;

// float start_target = 1.4f;    // rpm. this needs to be low to start. i'm almost certain your motor won't be able to make big jumps in rpm. to go higher, ramp it up. maybe write a ramp function that sets motor.target += 20 every few seconds. can definitely just start with commander "T??"
// float end_target = 2.5f;     // rpm
// float target = start_target; // rpm
unsigned long time_stamp_a = 0;
unsigned long time_stamp_b = 0;
// int level = 0;

// void ramp()
// {
//   if (level != 99) {
//     if (millis() - time_stamp > 200) {
//       time_stamp = millis();
//       motor.target = (float)(level)/(99-0)*(end_target-start_target)+start_target; // (6.28/60)* for rpm to rev/s
//       level+=1;
//     }
//   } 
//   else {
//     motor.target = start_target;
//   }
// }

void printTemp() {
    if(millis()-time_stamp_a > 1000) {
    temp = map(analogRead(A_TEMPERATURE),0,1024,-49,125);
    Serial.print("TempC: "); Serial.println(temp);
    time_stamp_a = millis();
  }
}

void setup()
{
  Serial.begin(115200);
  delay(6000); //NEED THIS (at least 3s), at least for some using platformio, so that the IDE serial monitor has time to boot up before the MCU tries to send stuff to it. at least for me, if i don't do this, I miss some or all SimpleFOCDebug statements at the beginning, which are SO useful.
  motor.useMonitoring(Serial); //this doesn't slow anything down. need this if you want to see the SimpleFOCDebug statements at the beginning, which are so useful. 
  // motor.monitor_variables = _MON_TARGET | _MON_VEL | _MON_VOLT_Q | _MON_VOLT_D; // specifies which variables get output. there's a whole bunch of them you can monitor. check this out if interested: https://docs.simplefoc.com/monitoring
  motor.monitor_downsample = 500;     
  
  sensor.enableInterrupts(doA, doB, doC); // hardware interrupt enable
  sensor.init(); //essential
  motor.linkSensor(&sensor);                           // downsampling, or how many loops between each communication with Serial. default 10. Higher number here means less frequent samples, so less latency, and less crazy serial output.

  driver.voltage_power_supply = 24; //
  // driver.voltage_limit = 5;   //makes no visible changes. motor.current_limit is the limiting factor.
  driver.pwm_frequency = 30000; // 20kHz is the standard for many boards, but the B-G431B-ESC1 can do 30kHz. i don't see why one would lower this if hardware can handle it. the higher, the smoother.
  driver.init(); //essential
  motor.linkDriver(&driver); //essential

  // current_sense.skip_align; //don't do this unless...idk, you'd need some good reason.
  current_sense.linkDriver(&driver); //essential
  current_sense.init(); //essential
  motor.linkCurrentSense(&current_sense); //essential

  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 5;
  motor.LPF_velocity.Tf = 0.01; //smooths out noisy sensor signal

  //board get SUPER HOT at 1000rpm and 20A.
  motor.current_limit = 10; // measured in Amps. IF THIS IS TOO LOW, YOUR MOTOR WON'T SPIN. REGARDLESS, START LOW (5% of rated current and move up to 10%, 20%, 30%, etc.) REQUIRES motor.phase_resistance to be set, which it is. 
  motor.torque_controller = TorqueControlType::voltage; //dc_current and foc_current modes requires tuning! better use voltage for quick results.
  motor.controller = MotionControlType::velocity; //torque measured in Volts, lol. velocity in rad/s
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM; // default is SinePWM but SpaceVectorPWM gives 15% more power. there's also trapezoidal120 and trap150.
  // motor.voltage_sensor_align = 1; //i think this is if you want to limit voltage or current during the alignment process (to make it quieter). can also do this...
  motor.initFOC(); //essential
  motor.init(); //essential

  // motor.target = target * 6.28 / 60;              // converting rpm to rad/s if target is velocity
  commander.add('M', doMotor, (char *)"motor");   // this is the connection control command SimpleFOCStudio (GUI) needs to send to connect and work...haven't gotten it to work yet :) 
  commander.add('T', doTarget, (char *)"target"); // type in "T20" to set the target to 20. Type in "T50" to set the target to 50, etc.
  commander.add('C', doLimitCurrent, (char *)"current"); //type in "C1" to set the current limit to 1A. Type in "C10" to set the current limit to 10A, etc.

  Serial.println("setup done");
}

void loop()
{

  // if(Serial.available()>0) {
  //   String command = Serial.readString();
  //   motor.PID_velocity.I= command.toFloat();
  //   Serial.print("I = "); Serial.println(motor.PID_velocity.I); 
  // }
  // if(millis()-time_stamp > 150) {
  //   time_stamp = millis();
  //   Serial.print("Target Speed = "); Serial.print(motor.shaft_velocity_sp); Serial.print(" ; Measured speed = "); Serial.println(motor.shaft_velocity);
  // }

  motor.loopFOC(); //this needs to run. always.
  motor.move(); //this needs to run. always.
  // motor.monitor(); //enable if you wish. with downsampling of 500, shouldn't be too epileptic and shouldn't affect performance. lower downsampling if you want more data (and more latency)
  commander.run(); //this never hurts. much neater than writing one's own "if(Serial.available()>0) { String command = Serial.readString(); ...}"
  // ramp();
  // if(motor.shaft_velocity != prev_val) {
  //   Serial.print("measured speed: "); Serial.println(motor.shaft_velocity);
  // }
}