# AdventuresWithBLDCs
Here, you'll find my documented efforts at BLDC control using the SimpleFOC repository.  

I've used a VESC SIX EDU to profile my (hall-sensored) brushless DC motor, and I am currently trying to get perfect FOC control of the motor working on a B-G431B-ESC1. This combination is very cost efficient, as the motor is under $100 and the ESC is under $20, and the hardware can handle 40A at 24V, giving ~1kW of power with a near-flat torque curve across its whole RPM range (100-3000RPM, roughly). My power supply only does 500W at the moment, but I digress.  

The motor moves marvelously with the VESC, but I've had quite a bit of trouble with SimpleFOC on PlatformIO, so I'm documenting my process here. I've been trying to follow [**this excellent tutorial**](https://www.youtube.com/watch?v=ufMs83Y3sXg) for a _SimpleFOC & PlatformIO & B-G431B-ESC1_ setup.  

Here's a [**link**](https://trampaboards.com/vesc-six-education--with-imu-p-34332.html) to the VESC SIX EDU.  
Here's a [**link**](https://www.st.com/en/evaluation-tools/b-g431b-esc1.html) to the B-G431B-ESC1.  
Here's a [**link**](https://maytech.cn/products/mto5065-170-ha-c) to the Maytech MTO 5065-HA-C (BLDC) that I'm using.  
Here's a [**link**](https://github.com/simplefoc/Arduino-FOC) to the SimpleFOC library for PlatformIO and ArduinoIDE.  
I'll try to get SimpleFOCStudio (SimpleFOC's GUI) running soon, too, so here's a [**link**](https://github.com/JorgeMaker/SimpleFOCStudio#readme) to that, too.  

### Some notes: ###  
- The repo is a collection of my platformio projects regarding this motor. That, and the C header file generated in the VESC Tool that gives all the motor and VESC control algorithm parameters. 
- I didn't know if there was a .gitignore to include to stop any of my binaries etc from uploading, so just clean each project before you build it, and you should be good. Do let me know if there is a `.gitignore` I should include. 
- _The only file that differs between projects is the main.cpp file_. No need to check every 
- A word of warning: this file gives the _line_ resistance and _line_ inductance, not the _phase_ resistance and _phase_ inductance; and the Lq-Ld _difference_, not the Lq/Ld _ratio_. Haven't examined much of the file yet, as I don't know which are relevant to SimpleFOC or where to insert them in my project.  

### My progress so far: ###
- I've set up all my hardware and PlatformIO successfully, confirming that code can be compiled and uploaded to the board (LED blinks) and there is a serial monitor connection. This required modifying the platform.ini file a bit to what I saw in [**this tutorial**](https://youtu.be/ufMs83Y3sXg?t=79). I will point out that since the time of the tutorial's release, the B-G431B-ESC1 has become supported by PlatformIO, so maybe no build flags are needed. The working `platform.ini` files are found in this repo and are identical across all its project folders.  
- I've gotten the hall sensor example from [**here**](https://youtu.be/ufMs83Y3sXg?t=320) to work.    
- I haven't been able to get the same working for motor.monitor. **Trying to get this to work now**.  
- I haven't been able to get the motor to spin with `MotionControlType::velocity_open_loop`, as done here. My guess is that the example uses a Gimbal motor, while I'm using a BLDC, and that's a key difference. **Trying to get this to work now**.
- I haven't been able to get SimpleFOCStudio working, or motor.monitor, for that matter. I was hoping I could write 

### My questions so far: ###
- Does Serial Monitor reading and writing successfully mean that ST Link works properly? ~~What if it's just the ESC's daughterboard with its USB connector, LED, and potentiometer that are working, but not the mainboard? (Ridiculous, I know, since the STM32G4 chip is on the mainboard.)~~ Hall sensing works correctly, and that's done on the mainboard, so ST Link must be working correctly.  
- Does it matter if I set `debug_tool = stlink` and `upload_protocol = stlink` in my `platform.ini`? Will SimpleFOCStudio work with or without this declaration? ST-Link and UART seem to be used interchangeably, here. I also know the board is capable of ST-Link V2.1; I believe there is no need to specify to PlatformIO whether it's supposed to be using ST-Link V.20 or ST-Link V2.1.  
- Do I need any `build_flags` in my `platform.ini` if the B-G431B-ESC1 is now officially supported by PlatformIO?  
- Can I forgo setting a `motor.voltage_limit` and instead set a `motor.current_limit`? My board is capable of `LowSideCurrentSensing`, after all, as is said [**here**](https://docs.simplefoc.com/current_sense). I'd much prefer to do it this way because current is intuitive--I'd want to limit it to 1A, to start, or even a little less. Here's why this matters to me: the [**"Supported driver boards"**](https://docs.simplefoc.com/drivers) page of the SimpleFOC documentation says that "The simplest way to [make sure your hardware can handle the currents your motor requires] is by checking the motor phase resistance `R`. Either check the datasheet of your motor and search for the resistance value or measure it yourself using a multimeter. Then check the value of your power supply voltage `V_dc` and once when you have the values you can find the maximum current I_max value by calculating `I_max = V_dc/R`." ... This gives me a max current of 24V/0.0361Ω=**665A** :) Following this, if I want `I_max = 1A`, I'd have to limit motor and/or supply voltage to 0.0361V? That's like PWM-ing 24V at a 0.15% duty cycle, which...well, I won't try to make this sound more ridiculous. You get my point.  
- Is there some extension for PlatformIO's SerialMonitor? It doesn't let you see what you're typing, it automatically enters it for you after a second of not typing. Half the time, I don't know if I have a connection at all. This is my biggest complaint about PlatformIO. It would be useful to know for getting `motor.monitor` to work.
- Why don't I see anything in my Serial Monitor when I have `Serial.println("xyz whatever");`in `void setup()` ? Actually, I'd expect to see something like _"hall sensors initialized ... current sense initialized ... FOC initialized ..."_ etc.
