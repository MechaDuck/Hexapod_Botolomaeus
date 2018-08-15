# HexapodHDA
v0.8.0
State machine for continues rotations and continues steps in X, Y direction are implemented and tested. Bluetooth communication integrated and tested. (28.06.2018)
Missing:
  -Automated shutdown, when battery low via ISR  
  
v0.2.0
Basic state machines for a simple movement are implemented. Missing Bluetooth communication and a lot of testing with the real 
robot (16.06.2018)

v0.0.1
Implemented inverse kinematics algorithm and interpolation. Interpolation need to be checked, 
because we got offset and the time intervals could be problematic   

v0.0.0
Implemented rough class structure for further implementations and tests on the Arduino Mega 2560
_____
Hardware that is being used:

-Dynamixel AX-18A (http://emanual.robotis.com/docs/en/dxl/ax/ax-18a/)  
-Arduino 2560

_____
Additionally Files:  
Servo_EEPROM_INIT/Servo_EEPROM_INIT.ino  -->File is used for the very first initialization of the servos e.g. setting ID's.  
_____
Main Files:  
Rough class structure from 16.06.2018

![alt text](Klassendiagramm.PNG)
