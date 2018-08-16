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

/**
*@file RobotControl.h
*/
/*!
*@brief Holds the run and setup method, that enables the control of the robot.
*/
class RobotControl
{
//variables
public:
	MovementController myMovementController;
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
	
	~RobotControl();

}; //RobotControl

#endif //__ROBOTCONTROL_H__
