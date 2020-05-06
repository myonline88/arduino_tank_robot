/*
Created by MyOnline88, May 4, 2020

You may use this program  "as is" without warranty 
The circuit diagram provided is tested according to my personal 
experience, but may or may not work well immediately with your 
designed circuit. The components used are standard, 
however, it may vary depending on what model, brand and design
your available components may be.

*/

this Arduino Tank robot used T300 tank chassis,
NRF24L01 wireless module, 12V TO 5V 3A buck converter to power up
the 4 servos for the Robotic Arm. It is very important to supply the
robotic arms' servos to achieve good performance. A 1N4001 diode is
also used and must be sufficient enough to prevent "short circuit" due to
reversed polarity. However, the L293D motor driver's power supplied isn't 
protected with reverse polarity and maybe damaged when the polarity of the 
battery is reversed but the Arduino Nano and the buck converter maybe 
spared. So be careful

The Autonomous mode maybe achieved the "K" button of the Thumb Joystick is
connected to Digital pin 2. This action will deprive the robot with 
the oscilating servo for the HC-SR04 scanning servo. You may use the 
ping sensor on a fixed position.  

if you wish to use the Autonomous feature, you must connect the signal pin 
of the thumb joystick to digital pin 2. 

D2 is also designed for Headlights, laser module or any device that is 
intended for ON or OFF mode. 

Nano A6 can also be used to attach additional potentiometer to be used
as "motor speed" selector. you must add additional potentiometer to the 
Transmitter circuit.

enjoy!!! please subscribe to my channel