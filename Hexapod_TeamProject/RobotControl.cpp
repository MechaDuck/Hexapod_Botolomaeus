/* 
* RobotControl.cpp
*
* Created: 22.04.2018 21:02:48
* Author: Tomislav Romic
*/

#include "RobotControl.h"
#include "BluetoothInterface.h"


#define BluetoothPollingFreq 1000 /*ms*/

// default constructor
RobotControl::RobotControl(){
	pxOld=0;
	pyOld=0;
	pzOld=0;
	rotZOld=0;
	
} //RobotControl

 RobotControl::setup(){
	 //Pin for interrupt of the battery guard
	pinMode(3, INPUT);
	digitalWrite(3, LOW);
	
	myMovementController.setRobotState(st_initStep);
	myMovementController.doContinuesRotationOrStep(0,0,0,0);
	myMovementController.setRobotState(st_lift236AndGoHomeAndLower236AndPush145);

}

 RobotControl::run(){	
	int cycles =0;
	float px=0,py=0,pz=0,rotZ=0;
	 	
	myBluetoothInterface.readInput();	
	
	px=myBluetoothInterface.getDirectionY();
	py=(-1.0)*myBluetoothInterface.getDirectionX();
	rotZ=myBluetoothInterface.getRotation();
	//Dunno y, but it has to be delayed here!
	delay(15);
	if(px != 0 || py != 0 || rotZ != 0){
			
		myMovementController.doContinuesRotationOrStep(px,py,0,rotZ);
			
		if(myMovementController.getRobotState() == st_lift236AndGoHomeAndLower236AndPush145){
			myMovementController.setRobotState(st_lift145AndGoHomeAndLower145AndPush236);
		}else{
			myMovementController.setRobotState(st_lift236AndGoHomeAndLower236AndPush145);
		}
	}
}

// default destructor
RobotControl::~RobotControl()
{
} //~RobotControl