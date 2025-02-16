<p align="center">
  <strong><font size="+3">AUTO-TUNER</font></strong>
</p>
<p align="center">
  <font size="+1">An STM32-Based Automatic Guitar Tuner</font>
</p>



## About
The Auto-Tuner (AT) is a handheld device designed to automatically tune guitar strings one at a time. It operates by placing the tuning motor fork over one of the guitar’s tuning pegs and then plucking the corresponding string. The AT detects the string’s pitch through vibration sensing and adjusts the tuning peg until the string reaches the correct pitch.  Once the string is in tune, the device alerts the user by emitting two short vibration pulses. Click the thumbnail below to see it in action!

<p align="center">
  <a href="https://www.youtube.com/watch?v=4Ss6xfbAHeE">
    <img src="https://img.youtube.com/vi/4Ss6xfbAHeE/0.jpg" alt="Watch on YouTube">
  </a>
</p>


## Auto-Tuner Design
A high-level block diagram of the Auto-Tuner’s design is shown below. Key features include an OLED screen and navigation buttons for menu control, along with a tone generator, metronome, and a gyroscope that ensures the screen remains correctly oriented in all positions during operation. However, the core tuning functionality of the AT lies in its vibration sensing and motor control, which operate within a feedback loop.

![Block Design](assets/block_design.png)

First, a piezo transducer detects string vibrations through the contact the Auto-Tuner makes with the guitar’s tuning peg. The signal is then amplified, offset, and low-pass filtered before being sampled by the microcontroller’s ADC. A pitch detection algorithm determines the pitch of the plucked string, and a state machine calculates the necessary motor rotation and direction based on the error between the detected and desired pitch. The motor then adjusts the tuning peg, after which the pitch is resampled, and the process repeats until the error falls below a defined threshold.


### Pitch Detection


### State Machine