/* 
* Servo.h
*
* Created: 22.04.2018 21:03:15
* Author: Tomislav Romic
*/

/**
@file Servo.h
*/

#ifndef __SERVO_H__
#define __SERVO_H__

#include "AX12A.h"

/*************************************************
Default parameters for Botolomäus. Change if needed!
**********************************************/
#define default_maxAngle 300
#define default_minAngle 0
#define default_maxSpeed  684.0 /* deg/sec^2 */

/*!
@brief Enables the communication with the servos
*/
class Servo
{
//variables
public:
    AX12A* m_pCommunicationObject; /*!< The communication object the leg is connected to */
    unsigned char ID;
//Borders defined by the software. No hardware check.
    float m_maxAngle; /*!< Maximal angle that can be adjusted. Describes only a software border. */
    float m_minAngle; /*!< Minimal angle that can be adjusted. Describes only a software border. */
	
	float m_maxSpeed; /*!< Maximal velocity that can be adjusted. */
//functions
public:
	/**
	*@function sendAction
	*@brief Sends a broadcast over the connected bus. Every registered setting will be executed.
				!Attention. EVERY servo that is connected to this bus, will execute his registered commands.
	*/
	void sendAction();
	/**
	*@function Servo
	*@brief Constructor. Sets desired communication object and the corresponding ID
	*@param m_pCommunicationObject
				Communication object that is need to communicate with servos
	*@param ID
				Corresponding ID that is connected
	*/
	Servo(AX12A& m_pCommunicationObject, unsigned char ID);
	/**
	*@function setMaxAngle
	*@brief Sets maximal angle that can be adjusted. Only software checks are implemented. No real hardware borders!
	*@param val
	*			maximal angle value
	*@result returns 1 if some rational value is given (here: 0>=0 <300)
	*/
	unsigned char setMaxAngle(float val);
	/**
	*@function setMinAngle
	*@brief Sets minimal angle that can be adjusted. Only software checks are implemented. No real hardware borders!
	*@param val
	*			minimal angle value
	*@result returns 1 if some rational value is given (here: 0>=0 <300)
	*/
	unsigned char setMinAngle(float val);
	/**
	*@function setServoAngle
	*@brief Servo proceeds to given angle
	*@param angleValue
				Angle to which the servo should proceed
	*@result returns 1 if angle can be proceeded. This means given value is between software borders (minAngle >= val < maxAngle) 
	*/
	unsigned char setServoAngle(float angleValue);
	/**
	*@function setServoAngleAndSpeed
	*@brief Servo proceeds to given angle with given speed.
	*@param angleValue [deg]
				Angle to which the servo should proceed
	*@param speed [deg/sec]
	*@result returns 1 if angle can be proceeded. This means given value is between software borders (minAngle >= val < maxAngle) and
				maxSpeed is not exceeded
	*/
	unsigned char setServoAngleAndSpeed(float angleValue, float speed);
	
	/**
	*@function setServoAngleAndSpeedReg
	*@brief Servo registers to given angle with given speed. With an action broadcast. Thoses registered positions are proceeded.
	*@param angleValue [deg]
				Angle to which the servo should be registered
	*@param speed [deg/sec]
	*@result returns 1 if angle can be proceeded. This means given value is between software borders (minAngle >= val < maxAngle) and
				maxSpeed is not exceeded
	*/	
	unsigned char setServoAngleAndSpeedReg(float angleValue, float speed);
	
	/**
	*@function setComplianceMargin
	*@brief Sets compliance margins for each servo. See AX12A description for more details
	*@param cw
				Clockwise compliance margin
	*@param ccw
				Counter-clockwise compliance margin
	*/		
	unsigned char setComplianceMargin(unsigned char cw, unsigned char ccw);
	/**
	*@function setComplianceSlope
	*@brief Sets compliance slopes for each servo. See AX12A description for more details
	*@param cw
				Clockwise compliance slope
	*@param ccw
				Counter-clockwise compliance slope
	*/			
	unsigned char setComplianceSlope(unsigned char cw, unsigned char ccw);
	
	/**
	*@function getCurrentAngle
	*@brief Reads out the register of the servo to get the current angle that is adjusted. 
	*@result float value in degrees
	*/
	float getCurrentAngle();
	
	/**
	*@function getCurrentSpeed
	*@brief Reads out the register of the servo to get the current velocity that is adjusted. 
	*@result float value in degrees/sec
	*/
	float getCurrentSpeed();
	
	/**
	*@function getVoltage
	*@brief Reads out the register of the servo to get the current voltage that is driving the servo. 
	*@result float value in Volt
	*/	
	float getVoltage();
	
	/**
	*@function getServoMovingStatus
	*@brief Reads out the register of the servo to get the current moving status of the servo. Is it moving?
	
	*@result returns 1 if servo is still moving. Returns 0 if the servo has reached his end position.
	*/		
	bool getServoMovingStatus();
	
	~Servo();

}; //Servo

#endif //__SERVO_H__
