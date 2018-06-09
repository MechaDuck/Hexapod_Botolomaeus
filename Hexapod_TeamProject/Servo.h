/* 
* Servo.h
*
* Created: 22.04.2018 21:03:15
* Author: Tomislav Romic
*/


#ifndef __SERVO_H__
#define __SERVO_H__

#include "AX12A.h"

#define default_maxAngle 300
#define default_minAngle 0
#define default_maxSpeed  684.0 /* deg/sec^2 */

/**
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	Make sure Safety Mode is turned on when testing critical functions. Roboter safety reasons :)
	TODO:
*/
#define safetyMode_On true
#define safety_maxAngle 200
#define safety_minAngle 100
#define safety_maxSpeed 150
class Servo
{
//variables
public:
    AX12A* m_pConnectedBus;
    unsigned char ID;
//Borders defined by the software. No hardware check.
    float m_maxAngle;
    float m_minAngle;
	
	float m_maxSpeed;
protected:
private:


//functions
public:
	static void broadcastAction(AX12A* broadcastBus);
	Servo(AX12A& m_pConnectedBus, unsigned char ID);
	unsigned char setMaxAngle(float val);
	unsigned char setMinAngle(float val);
	unsigned char setServoAngle(float angleValue);
	unsigned char setServoAngleAndSpeed(float angleValue, float speed);
	unsigned char setServoAngleAndSpeedReg(float angleValue, float speed);
	int getCurrentAngle();
	int getCurrentSpeed();
	
	~Servo();

}; //Servo

#endif //__SERVO_H__
