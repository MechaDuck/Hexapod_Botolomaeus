/* 
* Leg.h
*
* Created: 22.04.2018 21:07:09
* Author: Tomislav Romic
*/

/**
@file Leg.h
*/

#ifndef __LEG_H__
#define __LEG_H__

#include "Servo.h"
#include "AX12A.h"

/*************************************************
Default parameters for Botolomäus. Change if needed!
**********************************************/

#define def_homePositionBodyServo 150
#define def_homePositionMiddleServo 119.8806
#define def_homePositionLowerServo 87.273
#define def_speed 350 
#define def_posOfStatusLed 12
#define positionKnown 1
#define positionUnknown 2


/*   

Rough structure of the hexapod to understand the naming of the three servos on one leg. 

         (LeftLeg)                      <Hexapod>                        (RightLeg)	

							/////######################\\\\\		
	   _____________	 BodyServo				     BodyServo          _____________
	   LowerLegServo	  /////#########################\\\\\		    LowerLegServo
	 /////      \\\\\    /////							 \\\\\		  /////      \\\\\
	/////		 \\\\\  /////							  \\\\\		 /////		  \\\\\
   /////		MiddleLegServo		     				MiddleLegServo		       \\\\\
____\/______________________________________Ground___________________________________\/______
    TCP(Tool Center Point)                                                           TCP
															 
*/

/*!
@brief Provides functions to actuate with a leg and to read out sensor data

*/
class Leg
{
//variables
public:
	
	AX12A* m_pCommunicationObject; /*!< The communication object the leg is connected to */
	
	/** @name Servo objects of the leg
	*Every leg consists of three servos. The naming is unique, see description 
	*/
	/**@{*/	
	Servo m_bodyServo;
	Servo m_middleServo;
	Servo m_lowerServo;
	/**@}*/ 
	
	/** @name Coordinates of the leg TCP
	*Used to keep track of the current position of the leg TCP. 
	*/
	/**@{*/	
	float m_pkX;
	float m_pkY;
	float m_pkZ;
	/**@}*/ 
	
	/** @name Registered coordinates of the leg TCP
	*Used to keep track of the registered positions of the leg TCP. Registered positions are written in the servo 
	*	registers but are not yet executed
	*/
	/**@{*/	
	float m_pkXreg;
	float m_pkYreg;
	float m_pkZreg;
	/**@}*/ 
		
	
	int m_positionStatus; /*!< Used for the position tracking function. Position Status can be Known or Unknown */
	
	int m_PinForStatusLED; /*!< Pin where the status LED is connected to. The status LED will light up if a position is not reachable */
//functions
public:


	/**
	*@function Leg 
	*@brief Constructor of the Leg class. Every servo has a unique ID that needs to be connected.
	*@param &m_pCommunicationObject
				Communication settings for the three servos
	*@param ID_bodyServo, ID_middleLegServo, ID_lowerLegServo
	*			 ID's of the specific servos
	*/
	Leg(AX12A& m_pCommunicationObject, unsigned char ID_bodyServo, unsigned char ID_middleLegServo, unsigned char ID_lowerLegServo);
	/**
	*@function move2HomePosition
	*@brief moves leg to the home position (PtP-Motion)
	*/
	unsigned char move2HomePosition();
	
	/**
	*@function moveBodyServoToHomePos
	*@brief moves only the body servo to the home position
	*/
	unsigned char moveBodyServoToHomePos();
	
	/**
	*@function liftLeg
	*@brief lifts leg while adding to the current angle of the lower servo 40 degrees
	*/
	unsigned char liftLeg();
	/**
	*@function lowerLeg
	*@brief lowers leg while moving middle and lower servo to the home position
	*/
	unsigned char lowerLeg();
	
	/**
	*@function moveLegToKnownPosition
	*@brief Moves leg with max velocity to given angles. To keep track of the leg position it is required to provide the coordinates of the end position
	*			of the leg. There is no check if the final position is valid considering the given angles.
	*@param angleBody, angleMiddle, angleLower
				Angles to which the servos should be moved to.
	*@param pkX, pkY, pkZ
				Coordinates of the end position, when the leg is moved to the given angles.
	*@result returns 0, if no error occurs. Returns error code that specifies which servo couldn't reach desired end position.
	*/
	unsigned char moveLegToKnownPosition(float angleBody, float angleMiddle, float angleLower,float pkX, float pkY, float pkZ);
	
	/**
	*@function registerDesiredPositionAndSpeed
	*@brief Registers desired angles and the velocities in the servo registers. To keep track of the leg position it is required to provide the coordinates of the end position
	*			of the leg. There is no check if the final position is valid considering the given angles.
	*@param angleBody, angleMiddle, angleLower
				Angles to which the servos should be moved to.
	*@param pkX, pkY, pkZ
				Coordinates of the end position, when the leg is moved to the given angles.
	*@param speedBody, speedMiddle, speedLower
				Velocities the given angles are actuated.
	*@result returns 0, if no error occurs. Returns error code that specifies which servo couldn't reach desired end position.

	*/
	unsigned char registerDesiredPositionAndSpeed(float angleBody, float angleMiddle, float angleLower, float speedBody, float speedMiddle, float speedLower,float pkX, float pkY, float pkZ);
	
	/**
	*@function registerDesiredPosition
	*@brief Registers desired angles and the velocities in the servo registers. As velocity the "def_speed" is used. To keep track of the leg position it is required to provide the coordinates of the end position
	*			of the leg. There is no check if the final position is valid considering the given angles.
	*@param angleBody, angleMiddle, angleLower
				Angles to which the servos should be moved to.
	*@param pkX, pkY, pkZ
				Coordinates of the end position, when the leg is moved to the given angles.
	*@result returns 0, if no error occurs. Returns error code that specifies which servo couldn't reach desired end position.
	*/
	unsigned char registerDesiredPosition(float angleBody, float angleMiddle, float angleLower,float pkX, float pkY, float pkZ);
	
	/**
	*@function registerDesiredPosition
	*@brief Registers desired angles and the velocities in the servo registers. As velocity the "def_speed" is used. This function keeps no track of the
				end-position coordinates.
	*@param angleBody, angleMiddle, angleLower
				Angles to which the servos should be moved to.
	*@param pkX, pkY, pkZ
				Coordinates of the end position, when the leg is moved to the given angles.
	*@result returns 0, if no error occurs. Returns error code that specifies which servo couldn't reach desired end position.
	*/
	unsigned char registerDesiredPosition(float angleBody, float angleMiddle, float angleLower);
	
	/**
	*@function moveLegToRegisteredPosition
	*@brief Sends out a broadcast on the connected bus. So every servo that is connected to the bus moves to the registered position.
	*/
	unsigned char moveLegToRegisteredPosition();
	
	/**
	*@function setBodyServoAngle
	*@brief Moves body servo with max. speed to the desired angle.
	*@param angle
	*/
	unsigned char setBodyServoAngle(float angle);
	/**
	*@function setMiddleServoAngle
	*@brief Moves middle servo with max. speed to the desired angle.
	*@param angle
	*/
	unsigned char setMiddleServoAngle(float angle);
	/**
	*@function setLowerServoAngle
	*@brief Moves lower servo with max. speed to the desired angle.
	*@param angle
	*/
	unsigned char setLowerServoAngle(float angle);
	/**
	*@function setMaxAngles
	*@brief Sets software borders. For max. angles.
	*@param maxAngleBody, maxAngleMiddle, maxAngleLower
				Maximum angle that the servo is allowed to reach
	*/	
	unsigned char setMaxAngles(float maxAngleBody,float maxAngleMiddle,float maxAngleLower);
	/**
	*@function setMinAngles
	*@brief Sets software borders. For min. angles.
	*@param minAngleBody, minAngleMiddle, minAngleLower
				Minimum angle that the servo is allowed to reach
	*/	
	unsigned char setMinAngles(float minAngleBody,float minAngleMiddle,float minAngleLower);
	/**
	*@function setPinForStatusLED
	*@brief Sets pin where the status LED is connected to.
	*@param val
				Pin where the status LED is connected to.
	*/	
	unsigned char setPinForStatusLED(int val);
		
	/**
	*@function setComplianceMargin
	*@brief Sets compliance margins for each servo. This function sets all three servos to the same margins.
				See AX12A description for more details
	*@param cw
				Clockwise compliance margin
	*@param ccw
				Counter-clockwise compliance margin
	*/			
	unsigned char setComplianceMargin(unsigned char cw, unsigned char ccw);
	/**
	*@function setComplianceSlope
	*@brief Sets compliance slopes for each servo. This function sets all three servos to the same slope values.
				See AX12A description for more details
	*@param cw
				Clockwise compliance slope
	*@param ccw
				Counter-clockwise compliance slope
	*/			
	unsigned char setComplianceSlope(unsigned char cw, unsigned char ccw);
	/**
	*@function getCurrentAngles
	*@brief Reads out the registers of the servos. Returns the current angles the servos are set to.
	*@param bodyAngle, middleAngle, lowerAngle [return-by-reference]
				Holds the values that are read out of the servo registers. Current angular position. 
	*/	
	
	float getCurrentAngles(float& bodyAngle, float& middleAngle, float& lowerAngle);
	
	/**
	*@function getCurrentPos
	*@brief Returns the position of the leg. There is no guarantee that those values are valid. The user is required to provide right
				coordinates when setting new angles.
	*@param pkX, pkY, pkZ [return-by-reference]
				Current coordinates of the legs.
	*@result returns 1, when position is "known". 
	*/
	unsigned char getCurrentPos(float& pkX, float& pkY, float& pkZ);
	
	/**
	*@function getMovingStatus
	*@brief Reads out the registers of the servos.
	*@result returns 1, if a servo is still moving.
	*/
	bool getMovingStatus();
	
	~Leg();
	


}; //Leg

#endif //__LEG_H__
