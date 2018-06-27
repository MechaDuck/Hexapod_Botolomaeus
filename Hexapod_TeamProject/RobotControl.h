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
#include "MotionSequence.h"
#include "PtPMotion.h"

class RobotControl
{
//variables
public:
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
	test_raiseLeg();
	test_checkMegaState();
	test_blueoothCommunication();
	test_rotation();
	test_megaStep();
	
	~RobotControl();

}; //RobotControl

#endif //__ROBOTCONTROL_H__
