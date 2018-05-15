/* 
* Servo.cpp
*
* Created: 22.04.2018 21:03:15
* Author: Dr. Tomo
*/


#include "Servo.h"

// default constructor
Servo::Servo()
{
} //Servo

// default destructor
Servo::~Servo()
{
} //~Servo

int Servo::setServoAngle(int angleValue)
{
	if(angleValue<m_maxAngle && angleValue>m_minAngle){
		m_Angle=angleValue;
	}

}
