/* 
* RobotControl.h
*
* Created: 22.04.2018 21:02:49
* Author: Dr. Tomo
*/


#ifndef __ROBOTCONTROL_H__
#define __ROBOTCONTROL_H__

#include "Body.h"
#include "UARTInterface.h"
#include "MovementController.h"

class RobotControl
{
//variables
public:
protected:
private:
	UARTInterface m_ServoCommunication;
	MovementController m_UserCommunication;
	Body m_HexapodBody;

//functions
public:
	RobotControl();
	~RobotControl();

}; //RobotControl

#endif //__ROBOTCONTROL_H__
