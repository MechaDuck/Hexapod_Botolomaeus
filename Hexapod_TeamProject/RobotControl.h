/* 
* RobotControl.h
*
* Created: 22.04.2018 21:02:49
* Author: Tomislav Romic
*/


#ifndef __ROBOTCONTROL_H__
#define __ROBOTCONTROL_H__

#include "MovementController.h"
#include "BluetoothInterface.h"


class RobotControl
{
//variables
public:
protected:
private:
	MovementController myMovementController;
	BluetoothInterface myBluetoothInterface;
	
	
//functions
public:
	RobotControl();
	run();
	testFunctions();
	testServoAdjustment();
	testAndroidBluetooth();
	testSimpleBluetooth();
	testInverseKinematic();
	test_interpolationAngleForSyncLinMovement();
	test_timeConsumptionOfOneStepCalculations();
	test_ICS();
	test_stepMachine();
	test_allLegsTogether();
	test_leg1CorrectMovement();
	~RobotControl();

}; //RobotControl

#endif //__ROBOTCONTROL_H__
