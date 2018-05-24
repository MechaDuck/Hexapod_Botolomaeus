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



unsigned char Servo::setServoAngle(int angleValue){

	m_pConnectedBus->move(ID,angleValue);

}


// default destructor
Servo::~Servo()
{
} //~Servo