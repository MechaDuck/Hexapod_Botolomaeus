/* 
* Servo.h
*
* Created: 22.04.2018 21:03:15
* Author: Tomislav Romic
*/


#ifndef __SERVO_H__
#define __SERVO_H__


class Servo
{
//variables
public:
protected:
private:
	int m_maxAngle;
	int m_minAngle;
	int m_Angle;

//functions
public:
	Servo();
	~Servo();
	int setServoAngle(int angleValue);

}; //Servo

#endif //__SERVO_H__
