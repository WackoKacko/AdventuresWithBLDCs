#include <Arduino.h>
#include <SimpleFOC.h>

volatile int speed_target;
volatile long pwm_on_micros; // from 1000 to 2000
volatile unsigned long time_stamp;

// void setTargetFromPWM(BLDCMotor the_motor) {
//   if(millis() - time_stamp > 80) {
//     if (pwm_on_micros < 12) {speed_target = 0;}
//     else if (pwm_on_micros <= 1000) {speed_target = map(pwm_on_micros, 15, 965, -2850, -145); }
//     else if (pwm_on_micros > 1000) { speed_target = map(pwm_on_micros,1068,2040,145,2850); }
//     // Serial.println(speed_target);
//     the_motor.target = speed_target*6.28/60;
//     // Serial.println(pwm_on_micros);
//     time_stamp = millis();
//   }
// }

void pwm1() {
  static bool last_state;
  static long last_start_time;
  unsigned long now = micros();
  bool state_now = digitalRead(A_PWM);
  if (state_now != last_state) {
      if (state_now) {
          last_start_time = now;
      } else {
          pwm_on_micros = now - last_start_time;
      }
      last_state = state_now;
  }
}

// using a little, sensorless outrunner. Should be capable of at least 300W.
BLDCMotor motor(7, 0.08, 300);                                                               // hmm let's mess around with KV. they say it's usually 150%-170% of datasheet value but they also say it's 100%-200%.
BLDCDriver6PWM driver(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL); // got this from here: https://github.com/simplefoc/Arduino-FOC/blob/master/examples/hardware_specific_examples/B_G431B_ESC1/B_G431B_ESC1.ino
LowsideCurrentSense current_sense(0.003, -64.0 / 7.0, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);        // got this from here: https://github.com/simplefoc/Arduino-FOC/blob/master/examples/hardware_specific_examples/B_G431B_ESC1/B_G431B_ESC1.ino

HallSensor sensor(A_HALL1, A_HALL2, A_HALL3, 7); // hallpin1, hallpin2, hallpin3, #ofPolePairs
void doA() { sensor.handleA(); } // Interrupt routine initialization + callbacks
void doB() { sensor.handleB(); } // ^
void doC() { sensor.handleC(); } // ^

Commander commander(Serial);
void onMotor(char* cmd){ commander.motor(&motor,cmd); }
// void doMotor(char *cmd) { commander.motion(&motor, cmd); }
void doTarget(char *cmd) { commander.motion(&motor, cmd); }
void doLimitCurrent(char *cmd) { commander.scalar(&motor.current_limit, cmd); }

void ButtonIrqHandler() {
motor.disable();
HAL_NVIC_SystemReset();
}

void setup()
{
  Serial.begin(115200);
  delay(3500); //NEED THIS (at least 3s), at least for some using platformio, so that the IDE serial monitor has time to boot up before the MCU tries to send stuff to it. at least for me, if i don't do this, I miss some or all SimpleFOCDebug statements at the beginning, which are SO useful.
  motor.useMonitoring(Serial); //this doesn't slow anything down. need this if you want to see the SimpleFOCDebug statements at the beginning, which are so useful. 
  motor.monitor_variables = _MON_TARGET;// | _MON_VEL | _MON_CURR_D | _MON_CURR_Q; // specifies which variables get output. there's a whole bunch of them you can monitor. check this out if interested: https://docs.simplefoc.com/monitoring
  motor.monitor_downsample = 4000;     

  attachInterrupt(digitalPinToInterrupt(A_BUTTON), ButtonIrqHandler, FALLING);
  pinMode(A_BUTTON, INPUT);

  attachInterrupt(digitalPinToInterrupt(A_PWM), pwm1, CHANGE);
  
  sensor.enableInterrupts(doA, doB, doC); // hardware interrupt enable
  sensor.init(); //essential
  motor.linkSensor(&sensor);                           // downsampling, or how many loops between each communication with Serial. default 10. Higher number here means less frequent samples, so less latency, and less crazy serial output.

  driver.voltage_power_supply = 24; //
  driver.voltage_limit = 16; //see this page to find out more about this: https://docs.simplefoc.com/bldcdriver6pwm
  // driver.dead_zone = 0.05; // dead_zone [0,1] - default 0.02 - 2% see this page to find out more: https://docs.simplefoc.com/bldcdriver6pwm
  driver.pwm_frequency = 30000; // 20kHz is the standard for many boards, but the B-G431B-ESC1 can do 30kHz. i don't see why one would lower this if hardware can handle it. the higher, the smoother.
  driver.init(); //essential
  motor.linkDriver(&driver); //essential

  // current_sense.skip_align; //don't do this unless...idk, you'd need some good reason.
  current_sense.linkDriver(&driver); //essential
  current_sense.init(); //essential
  motor.linkCurrentSense(&current_sense); //essential

  motor.PID_velocity.P = 0.2; //this is as close as I've come
  motor.PID_velocity.I = 1; //this is as close as I've come
  // motor.PID_velocity.D = 0.0001;
  motor.PID_velocity.output_ramp = 300; // jerk control using voltage voltage ramp (default value is 300 volts per sec  ~ 0.3V per millisecond)
  motor.LPF_velocity.Tf = 0.01; //smooths out noisy sensor signal

  //board get SUPER HOT at 1000rpm and 20A.
  motor.current_limit = 3; // measured in Amps. IF THIS IS TOO LOW, YOUR MOTOR WON'T SPIN. REGARDLESS, START LOW (5% of rated current and move up to 10%, 20%, 30%, etc.) REQUIRES motor.phase_resistance to be set, which it is. 
  // motor.voltage_sensor_align = 0.1*driver.voltage_limit;
  motor.voltage_sensor_align = 1; // don't want lots of power for aligning sensors.
  motor.torque_controller = TorqueControlType::foc_current; //dc_current and foc_current modes requires tuning! better use voltage for quick results.
  motor.controller = MotionControlType::velocity; //torque measured in Volts, lol. velocity in rad/s
  motor.foc_modulation = FOCModulationType::SinePWM; // default is SinePWM but SpaceVectorPWM gives 15% more power. there's also trapezoidal120 and trap150.
  motor.initFOC(); //essential
  motor.init(); //essential

  // motor.target = target * 6.28 / 60;              // converting rpm to rad/s if target is velocity
  // commander.add('M', doMotor, (char *)"motor");   // this is the connection control command SimpleFOCStudio (GUI) needs to send to connect and work...haven't gotten it to work yet :) 
  commander.add('M', onMotor, (char *)"my motor");
  commander.add('T', doTarget, (char *)"target"); // type in "T20" to set the target to 20. Type in "T50" to set the target to 50, etc.
  commander.add('C', doLimitCurrent, (char *)"current"); //type in "C1" to set the current limit to 1A. Type in "C10" to set the current limit to 10A, etc.
  // commander.add('I', doIadjust)

  Serial.println("setup done");
}

void loop() 
{
  motor.loopFOC(); //this needs to run. always.
  motor.move(); //this needs to run. always.
  motor.monitor(); //enable if you wish. with downsampling of 500, shouldn't be too epileptic and shouldn't affect performance. lower downsampling if you want more data (and more latency)
  commander.run(); //this never hurts. much neater than writing one's own "if(Serial.available()>0) { String command = Serial.readString(); ...}"
  // setTargetFromPWM(motor);
  if(millis() - time_stamp > 80) {
    if (pwm_on_micros < 12) {speed_target = 0;}
    else if (pwm_on_micros <= 1000) {speed_target = map(pwm_on_micros, 15, 965, -2850, -145); }
    else if (pwm_on_micros > 1000) { speed_target = map(pwm_on_micros,1068,2040,145,2850); }
    // Serial.println(speed_target);
    motor.target = speed_target*6.28/60;
    // Serial.println(pwm_on_micros);
    time_stamp = millis();
  }
}