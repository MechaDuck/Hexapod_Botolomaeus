/* 
* Servo.h
*
* Created: 22.04.2018 21:03:15
* Author: Tomislav Romic
*/


#ifndef __SERVO_H__
#define __SERVO_H__

#include "AX12A.h"
class Servo
{
//variables
public:
    AX12A* m_pConnectedBus;
    unsigned char ID;
    int m_maxAngle;
    int m_minAngle;
    int m_desiredAngle;
    int m_adjustedAngle;
protected:
private:


//functions
public:
	Servo(AX12A& m_pConnectedBus, unsigned char ID);
	unsigned char setServoAngle(int angleValue);
	
	
	
	~Servo();

}; //Servo

#endif //__SERVO_H__
