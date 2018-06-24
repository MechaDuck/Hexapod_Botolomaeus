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
	void sendAction();
	Servo(AX12A& m_pConnectedBus, unsigned char ID);
	unsigned char setMaxAngle(float val);
	unsigned char setMinAngle(float val);
	unsigned char setServoAngle(float angleValue);
	unsigned char setServoAngleAndSpeed(float angleValue, float speed);
	unsigned char setServoAngleAndSpeedReg(float angleValue, float speed);
	unsigned char setComplianceMargin(unsigned char cw, unsigned char ccw);
	unsigned char setComplianceSlope(unsigned char cw, unsigned char ccw);
	float getCurrentAngle();
	float getCurrentSpeed();
	float getVoltage();
	bool getServoMovingStatus();
	
	~Servo();

}; //Servo

#endif //__SERVO_H__
