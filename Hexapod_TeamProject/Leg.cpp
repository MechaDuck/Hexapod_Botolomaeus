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
} //Leg

unsigned char Leg::move2HomePosition(){
	digitalWrite(12, LOW);
	m_bodyServo.setServoAngleAndSpeed(def_homePositionBodyServo,def_speed);
	m_middleServo.setServoAngleAndSpeed(def_homePositionMiddleServo,def_speed);
	m_lowerServo.setServoAngleAndSpeed(def_homePositionLowerServo,def_speed);
}



unsigned char Leg::moveBodyServoToHomePos(){
	m_bodyServo.setServoAngleAndSpeed(def_homePositionBodyServo,def_speed);
}

unsigned char Leg::liftLeg(){
	if(m_lowerServo.setServoAngleAndSpeed(m_lowerServo.getCurrentAngle()+40.0,def_speed)){
		digitalWrite(12, HIGH); 
	}
}

unsigned char Leg::lowerLeg(){
	m_middleServo.setServoAngleAndSpeed(def_homePositionMiddleServo,def_speed);
	m_lowerServo.setServoAngleAndSpeed(def_homePositionLowerServo,def_speed);
}

unsigned char Leg::moveLegToKnownPosition(float angleBody, float angleMiddle, float angleLower){
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
		return 1;
}

unsigned char Leg::registerDesiredPositionAndSpeed(float angleBody, float angleMiddle, float angleLower, float speed){
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
}

unsigned char Leg::registerDesiredPosition(float angleBody, float angleMiddle, float angleLower){
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
}

unsigned char Leg::moveLegToRegisteredPosition(){
	m_bodyServo.sendAction();
}

unsigned char Leg::setBodyServoAngle(float angle){
	m_bodyServo.setServoAngle(angle);
}

unsigned char Leg::setMiddleServoAngle(float angle){
	m_middleServo.setServoAngle(angle);
}

unsigned char Leg::setLowerServoAngle(float angle){
	m_lowerServo.setServoAngle(angle);
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

float Leg::getCurrentAngles(float& bodyAngle, float& middleAngle, float& lowerAngle){
	bodyAngle=m_bodyServo.getCurrentAngle();
	middleAngle=m_middleServo.getCurrentAngle();
	lowerAngle=m_lowerServo.getCurrentAngle();
}

bool Leg::getMovingStatus(){
	return(m_bodyServo.getServoMovingStatus() || m_middleServo.getServoMovingStatus() || m_lowerServo.getServoMovingStatus());
}


// default destructor
Leg::~Leg()
{
} //~Leg
