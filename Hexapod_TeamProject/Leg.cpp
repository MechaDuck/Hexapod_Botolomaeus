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
	digitalWrite(12, LOW);
	m_bodyServo.setServoAngleAndSpeed(def_homePositionBodyServo,def_speed);
	m_middleServo.setServoAngleAndSpeed(def_homePositionMiddleServo,def_speed);
	m_lowerServo.setServoAngleAndSpeed(def_homePositionLowerServo,def_speed);
	
	m_pkX=0;
	m_pkY=0;
	m_pkZ=0;
	
	m_statusOfPosition=positionKnown;
}



unsigned char Leg::moveBodyServoToHomePos(){
	m_bodyServo.setServoAngleAndSpeed(def_homePositionBodyServo,def_speed);
	m_pkX=0;
	m_pkY=0;
}

unsigned char Leg::liftLeg(){
	if(m_lowerServo.setServoAngleAndSpeed(m_lowerServo.getCurrentAngle()+20.0,def_speed)){
		digitalWrite(12, HIGH); 
		}
		m_statusOfPosition=positionUnknown;
}

unsigned char Leg::lowerLeg(){
	m_middleServo.setServoAngleAndSpeed(def_homePositionMiddleServo,def_speed);
	m_lowerServo.setServoAngleAndSpeed(def_homePositionLowerServo,def_speed);
	m_pkZ=0;
}

unsigned char Leg::moveLegToKnownPosition(float angleBody, float angleMiddle, float angleLower, float pkXnew, float pkYnew, float pkZnew){
	if(m_statusOfPosition==positionKnown){
		if(angleBody != -1){
			if(m_bodyServo.setServoAngle(angleBody)){
			}else{
				digitalWrite(12, HIGH);
			}
		}
		if(angleMiddle != -1){
			if(m_middleServo.setServoAngle(angleMiddle)){
			}else{
				digitalWrite(12, HIGH);
			}
		}
		if(angleLower != -1){
			if(m_lowerServo.setServoAngle(angleLower)){
			}else{
				digitalWrite(12, HIGH);
			}
		}
		m_pkX=pkXnew;
		m_pkY=pkYnew;
		m_pkX=pkZnew;

		return 1;
	}
	return 0;
}

unsigned char Leg::registerDesiredPositionAndSpeed(float angleBody, float angleMiddle, float angleLower, float pkXnew, float pkYnew, float pkZnew, float speed){
	if(m_statusOfPosition==positionKnown){
		if(angleBody != -1){
			if(m_bodyServo.setServoAngleAndSpeedReg(angleBody,speed)){
				}else{
				digitalWrite(12, HIGH);
			}
		}
		if(angleMiddle != -1){
			if(m_middleServo.setServoAngleAndSpeedReg(angleMiddle,speed)){
				}else{
				digitalWrite(12, HIGH);
			}
		}
		if(angleBody != -1){
			if(m_lowerServo.setServoAngleAndSpeedReg(angleLower,speed)){
				}else{
				digitalWrite(12, HIGH);
			}
		}
		m_pkXreg=pkXnew;
		m_pkYreg=pkYnew;
		m_pkZreg=pkZnew;
		return 1;
	}
	return 0;
}

unsigned char Leg::registerDesiredPosition(float angleBody, float angleMiddle, float angleLower, float pkXnew, float pkYnew, float pkZnew){
	if(m_statusOfPosition==positionKnown){
		if(angleBody != -1){
			if(m_bodyServo.setServoAngleAndSpeedReg(angleBody,def_speed)){
			}else{
				//digitalWrite(12, HIGH); 
			}
		}
		if(angleMiddle != -1){
			if(m_middleServo.setServoAngleAndSpeedReg(angleMiddle,def_speed)){
			}else{
				//digitalWrite(12, HIGH);
			}
		}
		if(angleBody != -1){
			if(m_lowerServo.setServoAngleAndSpeedReg(angleLower,def_speed)){
			}else{
				//digitalWrite(12, HIGH);
			}
		}
		m_pkXreg=pkXnew;
		m_pkYreg=pkYnew;
		m_pkZreg=pkZnew;
		return 1;
	}
	return 0;
}

unsigned char Leg::moveLegToRegisteredPosition(){
	m_bodyServo.sendAction();
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

unsigned char Leg::setMaxAngles(float maxAngleBody,float maxAngleMiddle,float maxAngleLower){
	m_bodyServo.setMaxAngle(maxAngleBody);
	m_middleServo.setMaxAngle(maxAngleMiddle);
	m_lowerServo.setMaxAngle(maxAngleLower);
}

unsigned char Leg::setMinAngles(float minAngleBody,float minAngleMiddle,float minAngleLower){
	m_bodyServo.setMinAngle(minAngleBody);
	m_middleServo.setMinAngle(minAngleMiddle);
	m_lowerServo.setMinAngle(minAngleLower);
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

float Leg::getCurrentAngles(float& bodyAngle, float& middleAngle, float& lowerAngle){
	bodyAngle=m_bodyServo.getCurrentAngle();
	middleAngle=m_middleServo.getCurrentAngle();
	lowerAngle=m_lowerServo.getCurrentAngle();
}

bool Leg::getMovingStatus(){
	return(m_bodyServo.getServoMovingStatus() || m_middleServo.getServoMovingStatus() || m_lowerServo.getServoMovingStatus());
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
