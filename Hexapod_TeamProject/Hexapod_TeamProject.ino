/*
 * Hexapod_TeamProject.ino
 *
 * Created: 4/22/2018 9:01:43 PM
 * Author: Tomislav Romic
 */ 
/**
*@file Hexapod_TeamProject.ino
*/
/**
*@brief Contains endless loop that operates the robot and will be called by the arduino controller.
*/
#include "RobotControl.h"
RobotControl myRobot;
void setup(){

	//Serial.begin(9600);
	myRobot.setup();
}

void loop(){
		myRobot.run();
}