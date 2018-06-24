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
		
	m_pkX=0;
	m_pkY=0;
	m_pkZ=0;
	
	m_pkXreg=0;
	m_pkYreg=0;
	m_pkZreg=0;
	
	m_positionStatus=positionUnknown;
} //Leg

unsigned char Leg::move2HomePosition(){
	digitalWrite(12, LOW);
	m_bodyServo.setServoAngleAndSpeed(def_homePositionBodyServo,def_speed);
	m_middleServo.setServoAngleAndSpeed(def_homePositionMiddleServo,def_speed);
	m_lowerServo.setServoAngleAndSpeed(def_homePositionLowerServo,def_speed);
	
	m_positionStatus=positionKnown;
}



unsigned char Leg::moveBodyServoToHomePos(){
	m_bodyServo.setServoAngleAndSpeed(def_homePositionBodyServo,def_speed);
	
	m_pkX=0;
	m_pkY=0;

}

unsigned char Leg::liftLeg(){
	if(m_lowerServo.setServoAngleAndSpeed(m_lowerServo.getCurrentAngle()+40.0,def_speed)){
		digitalWrite(12, HIGH); 
	}
	m_positionStatus=positionUnknown;
}

unsigned char Leg::lowerLeg(){
	m_middleServo.setServoAngleAndSpeed(def_homePositionMiddleServo,def_speed);
	m_lowerServo.setServoAngleAndSpeed(def_homePositionLowerServo,def_speed);
	m_pkZ=0;
}

unsigned char Leg::moveLegToKnownPosition(float angleBody, float angleMiddle, float angleLower,float pkX, float pkY, float pkZ){
	int retVal=0;
		if(angleBody != -1){
			if(m_bodyServo.setServoAngle(angleBody)){
			}else{
				digitalWrite(12, HIGH);
				retVal=retVal+2;
			}
		}
		if(angleMiddle != -1){
			if(m_middleServo.setServoAngle(angleMiddle)){
			}else{
				digitalWrite(12, HIGH);
				retVal=retVal+4;
			}
		}
		if(angleLower != -1){
			if(m_lowerServo.setServoAngle(angleLower)){
			}else{
				digitalWrite(12, HIGH);
				retVal=retVal+8;
			}
		}
		if(retVal==0){
				m_pkX=pkX;
				m_pkY=pkY;
				m_pkZ=pkZ;
				m_positionStatus=positionKnown;
			return 1;
		}else{
			m_positionStatus=positionUnknown;
			return retVal;
		}
}

unsigned char Leg::registerDesiredPositionAndSpeed(float angleBody, float angleMiddle, float angleLower, float speedBody, float speedMiddle, float speedLower,float pkX, float pkY, float pkZ){
	int retVal=0;
		if(angleBody != -1){
			if(m_bodyServo.setServoAngleAndSpeedReg(angleBody,speedBody)){
			}else{
				digitalWrite(12, HIGH);
				retVal=retVal+2;
			}
		}
		if(angleMiddle != -1){
			if(m_middleServo.setServoAngleAndSpeedReg(angleMiddle,speedMiddle)){
			}else{
				digitalWrite(12, HIGH);
				retVal=retVal+4;
			}
		}
		if(angleBody != -1){
			if(m_lowerServo.setServoAngleAndSpeedReg(angleLower,speedLower)){
			}else{
				digitalWrite(12, HIGH);
				retVal=retVal+8;
			}
		}
		if(retVal==0){
			m_pkXreg=pkX;
			m_pkYreg=pkY;
			m_pkZreg=pkZ;
			return 1;
		}else{
			m_positionStatus=positionUnknown;
			return retVal;
		}
}

unsigned char Leg::registerDesiredPosition(float angleBody, float angleMiddle, float angleLower,float pkX, float pkY, float pkZ){
		int retVal=0;
		if(angleBody != -1){
			if(m_bodyServo.setServoAngleAndSpeedReg(angleBody,def_speed)){
			}else{
				digitalWrite(12, HIGH); 
				retVal=retVal+2;
			}
		}
		if(angleMiddle != -1){
			if(m_middleServo.setServoAngleAndSpeedReg(angleMiddle,def_speed)){
			}else{
				digitalWrite(12, HIGH);
				retVal=retVal+4;
			}
		}
		if(angleBody != -1){
			if(m_lowerServo.setServoAngleAndSpeedReg(angleLower,def_speed)){
			}else{
				digitalWrite(12, HIGH);
				retVal=retVal+8;
			}
		}
		if(retVal==0){
			m_pkXreg=pkX;
			m_pkYreg=pkY;
			m_pkZreg=pkZ;
			return 1;
			}else{
			m_positionStatus=positionUnknown;
			return retVal;
		}
}

unsigned char Leg::registerDesiredPosition(float angleBody, float angleMiddle, float angleLower){
		int retVal=0;
		if(angleBody != -1){
			if(m_bodyServo.setServoAngleAndSpeedReg(angleBody,def_speed)){
				}else{
				digitalWrite(12, HIGH);
				retVal=retVal+2;
			}
		}
		if(angleMiddle != -1){
			if(m_middleServo.setServoAngleAndSpeedReg(angleMiddle,def_speed)){
				}else{
				digitalWrite(12, HIGH);
				retVal=retVal+4;
			}
		}
		if(angleBody != -1){
			if(m_lowerServo.setServoAngleAndSpeedReg(angleLower,def_speed)){
				}else{
				digitalWrite(12, HIGH);
				retVal=retVal+8;
			}
		}
		
		m_positionStatus=positionUnknown;
		if(retVal==0){
			return 1;
			}else{
			return retVal;
		}
}

unsigned char Leg::moveLegToRegisteredPosition(){
	m_bodyServo.sendAction();
	if(m_positionStatus != positionUnknown){
		m_pkX=m_pkXreg;
		m_pkY=m_pkYreg;
		m_pkZ=m_pkZreg;
	}

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

unsigned char Leg::getCurrentPos(float& pkX, float& pkY, float& pkZ){
	if(m_positionStatus==positionKnown){
		pkX=m_pkX;
		pkY=m_pkY;
		pkZ=m_pkZ;
		return 1;
	}
	return 0;

}

bool Leg::getMovingStatus(){
	return(m_bodyServo.getServoMovingStatus() || m_middleServo.getServoMovingStatus() || m_lowerServo.getServoMovingStatus());
}


unsigned char Leg::setComplianceMargin(unsigned char cw, unsigned char ccw){
	m_bodyServo.setComplianceMargin(cw,ccw);
	m_middleServo.setComplianceMargin(cw,ccw);
	m_lowerServo.setComplianceMargin(cw,ccw);
}

unsigned char Leg::setComplianceSlope(unsigned char cw, unsigned char ccw){
	m_bodyServo.setComplianceSlope(cw,ccw);
	m_middleServo.setComplianceSlope(cw,ccw);
	m_lowerServo.setComplianceSlope(cw,ccw);

}

// default destructor
Leg::~Leg()
{
} //~Leg
