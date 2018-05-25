/* 
* Leg.cpp
*
* Created: 22.04.2018 21:07:09
* Author: Tomislav Romic
*/


#include "Leg.h"

// default constructor
Leg::Leg(AX12A& m_pConnectedBus, unsigned char ID_bodyServo, unsigned char ID_middleLegServo, unsigned char ID_lowerLegServo)
	:m_lowerLegServo(m_pConnectedBus,ID_lowerLegServo),m_bodyServo(m_pConnectedBus,ID_bodyServo),m_middleLegServo(m_pConnectedBus,ID_middleLegServo){
	
} //Leg

unsigned char Leg::setBodyServoAngle(int angle){
	m_bodyServo.setServoAngle(angle);

}

unsigned char Leg::setMiddleLegServoAngle(int angle){
	m_middleLegServo.setServoAngle(angle);
}

unsigned char Leg::setLowerLegServoAngle(int angle){
	
	m_lowerLegServo.setServoAngle(angle);
}

// default destructor
Leg::~Leg()
{
} //~Leg
