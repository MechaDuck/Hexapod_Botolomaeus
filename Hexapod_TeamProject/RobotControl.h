/* 
* RobotControl.h
*
* Created: 22.04.2018 21:02:49
* Author: Tomislav Romic
*/


#ifndef __ROBOTCONTROL_H__
#define __ROBOTCONTROL_H__

#include "MovementController.h"


class RobotControl
{
//variables
public:
protected:
private:
	MovementController myMovementController;
	
//functions
public:
	RobotControl();
	testFunctions();
	testServoAdjustment();
	~RobotControl();

}; //RobotControl

#endif //__ROBOTCONTROL_H__
