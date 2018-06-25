/*
 * Hexapod_TeamProject.ino
 *
 * Created: 4/22/2018 9:01:43 PM
 * Author: Tomislav Romic
 */ 
#include "RobotControl.h"
RobotControl myRobot;
void setup(){
	pinMode(3, INPUT); 
	digitalWrite(3, LOW);
	Serial.begin(9600);
}

void loop(){
		myRobot.test_blueoothCommunication();
}