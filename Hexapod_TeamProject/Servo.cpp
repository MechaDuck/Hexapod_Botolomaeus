/* 
* Servo.cpp
*
* Created: 22.04.2018 21:03:15
* Author: Tomislav Romic
*/


#include "Servo.h"
// default constructor
Servo::Servo(AX12A& m_pConnectedBus, unsigned char ID){
	this->m_pConnectedBus=&m_pConnectedBus;
	this->ID=ID;
} //Servo



unsigned char Servo::setServoAngle(double angleValue){
	
	double angleServo = angleValue/0.29;
	m_pConnectedBus->moveSpeed(ID,angleServo,100);

}


unsigned char Servo::setServoAngleAndSpeed(float angleValue, float speed){
	double angleServo = angleValue/0.29;
	//TODO:
		m_pConnectedBus->moveSpeed(ID,angleServo,speed);
	

}

// default destructor
Servo::~Servo()
{
} //~Servo 