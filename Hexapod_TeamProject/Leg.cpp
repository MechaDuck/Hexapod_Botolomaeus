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

unsigned char Leg::move2HomePosition(){
	m_bodyServo.setServoAngle(150);
	m_middleLegServo.setServoAngle(84.8);
	m_lowerLegServo.setServoAngle(53.6);

}

unsigned char Leg::setBodyServoAngle(double angle){
	m_bodyServo.setServoAngle(angle);

}

unsigned char Leg::setMiddleLegServoAngle(double angle){
	m_middleLegServo.setServoAngle(angle);
}

unsigned char Leg::setLowerLegServoAngle(double angle){
	
	m_lowerLegServo.setServoAngle(angle);
}

// default destructor
Leg::~Leg()
{
} //~Leg
