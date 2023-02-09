#include <Arduino.h>
#include <SimpleFOC.h>

// using a little, sensorless outrunner. Should be capable of at least 300W.
BLDCMotor motor(7, 0.025, 1400);                                                               // hmm let's mess around with KV. they say it's usually 150%-170% of datasheet value but they also say it's 100%-200%.
BLDCDriver6PWM driver(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL); // got this from here: https://github.com/simplefoc/Arduino-FOC/blob/master/examples/hardware_specific_examples/B_G431B_ESC1/B_G431B_ESC1.ino
LowsideCurrentSense current_sense(0.003, -64.0 / 7.0, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);        // got this from here: https://github.com/simplefoc/Arduino-FOC/blob/master/examples/hardware_specific_examples/B_G431B_ESC1/B_G431B_ESC1.ino

Commander commander(Serial);
void doMotor(char *cmd) { commander.motion(&motor, cmd); }
void doTarget(char *cmd) { commander.motion(&motor, cmd); }
void doLimitCurrent(char *cmd) { commander.scalar(&motor.current_limit, cmd); }

float target = 200; //rpm. this needs to be low to start. i'm almost certain your motor won't be able to make big jumps in rpm. to go higher, ramp it up. maybe write a ramp function that sets motor.target += 20 every few seconds. can definitely just start with commander "T??"

void setup()
{
  Serial.begin(115200);
  delay(3000); //NEED THIS, at least for some using platformio, so that the IDE serial monitor has time to boot up before the MCU tries to send stuff to it. at least for me, if i don't do this, I miss some or all SimpleFOCDebug statements at the beginning, which are SO useful.
  motor.useMonitoring(Serial); //this doesn't slow anything down. need this if you want to see the SimpleFOCDebug statements at the beginning, which are so useful. 
  motor.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE; // specifies which variables get output. there's a whole bunch of them you can monitor. check this out if interested: https://docs.simplefoc.com/monitoring
  motor.monitor_downsample = 500;                                // downsampling, or how many loops between each communication with Serial. default 10. Higher number here means less frequent samples, so less latency, and less crazy serial output.

  driver.voltage_power_supply = 12; //
  // driver.voltage_limit = 5;   //makes no visible changes. motor.current_limit is the limiting factor.
  driver.pwm_frequency = 30000; // 20kHz is the standard for many boards, but the B-G431B-ESC1 can do 30kHz. i don't see why one would lower this if hardware can handle it. the higher, the smoother.
  driver.init(); //essential
  motor.linkDriver(&driver); //essential

  // current_sense.skip_align; //don't do this unless...idk, you'd need some good reason.
  current_sense.linkDriver(&driver); //essential
  current_sense.init(); //essential
  motor.linkCurrentSense(&current_sense); //essential

  motor.current_limit = 15; // measured in Amps. IF THIS IS TOO LOW, YOUR MOTOR WON'T SPIN. REGARDLESS, START LOW (5% of rated current and move up to 10%, 20%, 30%, etc.) REQUIRES motor.phase_resistance to be set, which it is. 
  // motor.voltage_limit = 0.05f; // do this OR motor.current_limit. current limit superior, in my humble opinion.
  motor.torque_controller = TorqueControlType::foc_current; //voltage mode should work. dc_current better. foc_current best. Thankfully, the STM32G4 on the B-G431B-ESC1 can do 170MHz so it can do foc_current, which needs lots of computation power.
  motor.controller = MotionControlType::velocity_openloop; //it's the name of this example and the identifying part of this setup/algorithm.
  // motor.foc_modulation = FOCModulationType::SinePWM; // default or...
  // motor.foc_modulation = FOCModulationType::SpaceVectorPWM; // not default. 15% more power, but only matters once everything else is tuned, i think.
  motor.initFOC(); //essential
  motor.init(); //essential

  motor.target = target * 6.28 / 60;              // converting rpm to rev/s
  commander.add('M', doMotor, (char *)"motor");   // this is the connection control command SimpleFOCStudio (GUI) needs to send to connect and work...haven't gotten it to work yet :) 
  commander.add('T', doTarget, (char *)"target"); // type in "T20" to set the target to 20. Type in "T50" to set the target to 50, etc.
  commander.add('C', doLimitCurrent, (char *)"current"); //type in "C1" to set the current limit to 1A. Type in "C10" to set the current limit to 10A, etc.

  Serial.println("setup done");
}

void loop()
{
  motor.loopFOC(); //this needs to run. always.
  motor.move(); //this needs to run. always.
  motor.monitor(); //enable if you wish. with downsampling of 500, shouldn't be too epileptic and shouldn't affect performance. lower downsampling if you want more data (and more latency)
  commander.run(); //this never hurts. much neater than writing one's own "if(Serial.available()>0) { String command = Serial.readString(); ...}"
}

// Bonus: if you want to kill the program from the Serial monitor for some reason, define this before setup: 
// void(* resetFunc) (void) = 0; //declare reset function at address 0
// Then you need your own "if(Serial.available()>0) { String command = Serial.readString(); ...}" kind of thing going on in loop();
// Then, in loop, send something via Serial Monitor to make resetFunc() execute.
// That being said, "T0" works fine, and the reset approach will not actually reset the board. you'll have to upload again or something to restart the program.