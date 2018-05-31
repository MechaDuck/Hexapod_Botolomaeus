/*
 * Hexapod_TeamProject.ino
 *
 * Created: 4/22/2018 9:01:43 PM
 * Author: Tomislav Romic
 */ 
#include "RobotControl.h"
RobotControl myRobot;
void setup()
{
	/* add setup code here, setup code runs once when the processor starts */
	
	myRobot.test_interpolationAngleForSyncLinMovement();

}

void loop(){


//testFunctions();


}



void testAndroid2ServoCommunicationViaBluetooth(){
	
}