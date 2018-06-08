/* 
* Leg.cpp
*
* Created: 22.04.2018 21:07:09
* Author: Tomislav Romic
*/


#include "Leg.h"

// default constructor
Leg::Leg(AX12A& m_pConnectedBus, unsigned char ID_bodyServo, unsigned char ID_middleLegServo, unsigned char ID_lowerLegServo)
	:m_lowerServo(m_pConnectedBus,ID_lowerLegServo),m_bodyServo(m_pConnectedBus,ID_bodyServo),m_middleServo(m_pConnectedBus,ID_middleLegServo){
	m_statusOfPosition=positionUnknown;
} //Leg

unsigned char Leg::move2HomePosition(){
	m_bodyServo.setServoAngleAndSpeed(def_homePositionBodyServo,100);
	m_middleServo.setServoAngleAndSpeed(def_homePositionMiddleServo,100);
	m_lowerServo.setServoAngleAndSpeed(def_homePositionLowerServo,100);
	
	setpkX(0);
	setpkY(0);
	setpkZ(0);
	m_statusOfPosition=positionKnown;
}


unsigned char Leg::moveLegToKnownPosition(float angleBody, float angleMiddle, float angleLower, float pkXnew, float pkYnew, float pkZnew){
	if(m_statusOfPosition==positionKnown){
		if(angleBody != -1){
			m_bodyServo.setServoAngle(angleBody);
		}
		if(angleMiddle != -1){
			m_middleServo.setServoAngle(angleMiddle);
		}
		if(angleBody != -1){
			m_lowerServo.setServoAngle(angleLower);
		}
		m_pkX=pkXnew;
		m_pkY=pkYnew;
		m_pkX=pkZnew;
		return 1;
	}
	return 0;
}

unsigned char Leg::registerDesiredPosition(float angleBody, float angleMiddle, float angleLower, float pkXnew, float pkYnew, float pkZnew){
	if(m_statusOfPosition==positionKnown){
		if(angleBody != -1){
			m_bodyServo.setServoAngle(angleBody);
		}
		if(angleMiddle != -1){
			m_middleServo.setServoAngle(angleMiddle);
		}
		if(angleBody != -1){
			m_lowerServo.setServoAngle(angleLower);
		}
		m_pkXreg=pkXnew;
		m_pkYreg=pkYnew;
		m_pkZreg=pkZnew;
		return 1;
	}
	return 0;
}

unsigned char Leg::moveLegToRegisteredPosition(){
	Servo::broadcastAction(m_pConnectedBus);
	m_pkX=m_pkXreg;
	m_pkY=m_pkYreg;
	m_pkZ=m_pkZreg;

	m_pkXreg=0;
	m_pkYreg=0;
	m_pkZreg=0;
}

unsigned char Leg::setBodyServoAngle(float angle){
	m_bodyServo.setServoAngle(angle);
	//Sets the position to unknown
	m_statusOfPosition=positionUnknown;
}

unsigned char Leg::setMiddleServoAngle(float angle){
	m_middleServo.setServoAngle(angle);
	//Sets the position to unknown
	m_statusOfPosition=positionUnknown;
}

unsigned char Leg::setLowerServoAngle(float angle){
	m_lowerServo.setServoAngle(angle);
	//Sets the position to unknown
	m_statusOfPosition=positionUnknown;
}

float Leg::getCurrentPosX(){
	return m_pkX;
}

float Leg::getCurrentPosY(){
	return m_pkY;
}

float Leg::getCurrentPosZ(){
	return m_pkZ;
}

unsigned char Leg::setpkX(float val){
	m_pkX=val;
}

unsigned char Leg::setpkY(float val){
	m_pkY=val;
}

unsigned char Leg::setpkZ(float val){
	m_pkZ=val;
}

// default destructor
Leg::~Leg()
{
} //~Leg
