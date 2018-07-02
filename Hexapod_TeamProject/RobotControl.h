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
    /**
     * @link aggregationByValue 
     */
	MovementController myMovementController;
    /**
     * @link aggregationByValue 
     */
	BluetoothInterface myBluetoothInterface;
private:
	float pxOld;
	float pyOld;
	float pzOld;
	float rotZOld;
//functions
public:
	RobotControl();
	setup();
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
