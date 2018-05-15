/* 
* RobotControl.cpp
*
* Created: 22.04.2018 21:02:48
* Author: Dr. Tomo
*/

//TODO: Change author name
#include "RobotControl.h"

// default constructor
RobotControl::RobotControl()
{
	m_HexapodBody.connectServosToBodyParts(&m_ServoCommunication);
} //RobotControl

// default destructor
RobotControl::~RobotControl()
{
} //~RobotControl
