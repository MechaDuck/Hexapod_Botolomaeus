/* 
* Servo.cpp
*
* Created: 22.04.2018 21:03:15
* Author: Tomislav Romic
*/


#include "Servo.h"



void Servo::sendAction(){
	m_pConnectedBus->action();
}

// default constructor
Servo::Servo(AX12A& m_pConnectedBus, unsigned char ID){
	this->m_pConnectedBus=&m_pConnectedBus;
	this->ID=ID;
	
		//Initialize borders with default values
		m_maxAngle=default_maxAngle;
		m_minAngle=default_minAngle;
		m_maxSpeed=default_maxSpeed;
} //Servo



unsigned char Servo::setMaxAngle(float val){
	if(val >= 0 && val < 300){
		m_maxAngle=val;
		return 1;
	}
	return 0;
}

unsigned char Servo::setMinAngle(float val){
	if(val >= 0 && val < 300){
		m_minAngle=val;
		return 1;
	}
	return 0;
}

unsigned char Servo::setServoAngle(float angleValue){	
	if(angleValue >= m_minAngle && angleValue <= m_maxAngle){
		//Conversion from deg to a 10 bit integer value (0 to 1023)
		float con_angleValue = angleValue/0.29;
		m_pConnectedBus->moveSpeed(ID,con_angleValue,100);
		return 1;
	}
	return 0;
}


unsigned char Servo::setServoAngleAndSpeed(float angleValue, float speed){
	if(angleValue >= m_minAngle && angleValue <= m_maxAngle && speed <= m_maxSpeed){
		//Conversion from deg to a 10 bit integer value (0 to 1023)
		float con_angleValue = angleValue/0.29;
		//Conversion from deg/sec in rpm and then to a 10 bit integer value (0 to 1023)
		float con_speed = speed * 0.166666/0.111;
		m_pConnectedBus->moveSpeed(ID,con_angleValue,con_speed);
		return 1;
	}
	return 0;	

}

unsigned char Servo::setServoAngleAndSpeedReg(float angleValue, float speed){
	if(angleValue >= m_minAngle && angleValue <= m_maxAngle && speed <= m_maxSpeed){
		//Conversion from deg to a 10 bit integer value (0 to 1023)
		float con_angleValue = angleValue/0.29;
		//Conversion from deg/sec in rpm and then to a 10 bit integer value (0 to 1023)
		float con_speed = speed * 0.166666/0.111;
		m_pConnectedBus->moveSpeedRW(ID,con_angleValue,con_speed);
		return 1;
	}
	return 0;
}

unsigned char Servo::setComplianceMargin(unsigned char cw, unsigned char ccw){
	m_pConnectedBus->setCMargin(ID,cw,ccw);

}

unsigned char Servo::setComplianceSlope(unsigned char cw, unsigned char ccw){
	m_pConnectedBus->setCSlope(ID,cw,ccw);

}

float Servo::getCurrentAngle(){
	return(m_pConnectedBus->readPosition(ID)*0.29);
}

float Servo::getCurrentSpeed(){
	return(m_pConnectedBus->readSpeed(ID)*0.111/0.166666);
}

float Servo::getVoltage(){
	return(m_pConnectedBus->readVoltage(ID)/10);
	
}

bool Servo::getServoMovingStatus()
{
	return(m_pConnectedBus->moving(ID));
}

// default destructor
Servo::~Servo()
{
} //~Servo 