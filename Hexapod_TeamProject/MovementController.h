/* 
* MovementController.h
*
* Created: 22.04.2018 21:39:35
* Author: Tomislav Romic
*/

/**
@file MovementController.h
*/

#ifndef __MOVEMENTCONTROLLER_H__
#define __MOVEMENTCONTROLLER_H__

#include "Leg.h"
#include "Arduino.h"
#include "AX12A.h"

///@{
/**
*@name UART settings
*@brief Settings for the UART communication with the Servos. Servos can be connected 
*		to three UART-Ports (Serial1, Serial2, Serial3)
*/

#define DirectionPinForBus1 	(10u)
#define BaudRate  				(1000000ul)
#define ServoSerialForBus1		(Serial1)

#define DirectionPinForBus2 	(10u)
#define BaudRate  				(1000000ul)
#define ServoSerialForBus2		(Serial2)

#define DirectionPinForBus3 	(10u)
#define BaudRate  				(1000000ul)
#define ServoSerialForBus3		(Serial3)

///@}

///@{
/**
*@name Servo ID's
*@brief Predefined ID's, that the servos are set to.
*/
#define ID_leg1_lowerLegServo 3
#define ID_leg1_middleLegServo 2
#define ID_leg1_bodyServo 1

#define ID_leg2_lowerLegServo 4
#define ID_leg2_middleLegServo 5
#define ID_leg2_bodyServo 6

#define ID_leg3_lowerLegServo 7
#define ID_leg3_middleLegServo 8
#define ID_leg3_bodyServo 9

#define ID_leg4_lowerLegServo 10
#define ID_leg4_middleLegServo 11
#define ID_leg4_bodyServo 12

#define ID_leg5_lowerLegServo 13
#define ID_leg5_middleLegServo 14
#define ID_leg5_bodyServo 15

#define ID_leg6_lowerLegServo 16
#define ID_leg6_middleLegServo 17
#define ID_leg6_bodyServo 18

///@}

/**

Rough structure of the hexapod to understand the naming of the six legs. To find the "UP" position please look on the backside of the robot.
<pre>
\verbatim
leg1	    	      leg2
<===\     /"UP"\     /===>
	 \==|--------|==/
		|--------|
leg3	|--------|	leg4
<=======|--------|=======>
		|--------|
		|--------|
leg5 /==|--------|==\leg6
<===/   |--------|	 \===>
\endverbatim
</pre>

*/
/**   
Rough structure of the hexapod to understand the naming of the three servos on one leg. 
\verbatim
         (LeftLeg)                      Hexapod	                         (RightLeg)	
							/////######################\\\\\		
	   _____________	 BodyServo				     BodyServo          _____________
	   LowerLegServo	  /////#########################\\\\\		    LowerLegServo
	 /////      \\\\\    /////							 \\\\\		  /////      \\\\\
	/////		 \\\\\  /////							  \\\\\		 /////		  \\\\\
   /////		MiddleLegServo		     				   MiddleLegServo	       \\\\\
____\/__________________________________ Ground______________________________________\/______															 
\endverbatim
*/

struct interpolatedMovement
{
	float AngleMovementQ1[12];
	float AngleMovementQ2[12];
	float AngleMovementQ3[12];
	float PositionMovementPx[12];
	float PositionMovementPy[12];
	float PositionMovementPz[12];
};


class MovementController
{
//variables
public:
	///@{
	/**
	*@brief Three objects of the class AX12A are created. To enable parallel communication with three serial ports on the Arduino. 
	*/
	AX12A m_ServoComObject1; /**< Communication object */
	AX12A m_ServoComObject2; /**< Communication object */
	AX12A m_ServoComObject3; /**< Communication object */
	///@}	

	///@{
	/**
	*@brief Every leg on the hexapod can be individually controlled by following objects 
	*/
	Leg m_Leg1;
	Leg m_Leg2;
	Leg m_Leg3;
	Leg m_Leg4;
	Leg m_Leg5;
	Leg m_Leg6;
	///@}
protected:
private:

	
	
//functions
public:
	MovementController();
	
	unsigned char initRobot();
	unsigned char move2HomePosition();
	
	///@{
	/**  
	* @name worldCoordinatesToLegCoordinates                                                        
	* @abstract Converts world coordinates to the specific leg coordinate system.
	*/
	/**
	* @param legNumber
	*			The leg to that the coordinate system is transformed
	* @param pkXold, pkYold, pkZold [mm]
	*			Current position of the leg in the leg coordinate system
	* @param pwX, pwY, pwZ [mm]
	*			New desired position. Given in the world coordinate system.
	* @param pkX, pkY, pkZ [mm](return-by-reference)
	*			New calculated position in specific leg coordinate system.
	* @retval Returns an error code, if problems occur. Else it returns 1
	*/
	unsigned char getLegCoordinatesFromWorldCoordinates(unsigned char legNumber,float pkXold, float pkYold, float pkZold, float pwX, float pwY, float pwZ, float& pkX, float& pkY, float& pkZ);
	unsigned char worldPositionToLegOnePosition(float pkXold, float pkYold, float pkZold, float pwX, float pwY, float pwZ, float& pkX, float& pkY, float& pkZ);
	unsigned char worldPositionToLegTwoPosition(float pkXold, float pkYold, float pkZold, float pwX, float pwY, float pwZ, float& pkX, float& pkY, float& pkZ);
	unsigned char worldPositionToLegThreePosition(float pkXold, float pkYold, float pkZold, float pwX, float pwY, float pwZ, float& pkX, float& pkY, float& pkZ);
	unsigned char worldPositionToLegFourPosition(float pkXold, float pkYold, float pkZold, float pwX, float pwY, float pwZ, float& pkX, float& pkY, float& pkZ);
	unsigned char worldPositionToLegFivePosition(float pkXold, float pkYold, float pkZold, float pwX, float pwY, float pwZ, float& pkX, float& pkY, float& pkZ);
	unsigned char worldPositionToLegSixPosition(float pkXold, float pkYold, float pkZold, float pwX, float pwY, float pwZ, float& pkX, float& pkY, float& pkZ);
	///@}
	
	///@{
	/**
	*@name getWorldCoordinatesFromFK 
	*@param legNumber
	*			 The leg to that the coordinate system is transformed
	*@param q1, q2, q3 [deg] 
	*			Angles that are currently set on the servos. 
	*@param px, py, pz [mm] (return-by-reference)
	*			Calculated position where the chosen leg currently is. Coordinates are given in respect to the world coordinate system.
	*/
	unsigned char getWorldCoordinatesFromFK(unsigned char legNumber, float q1, float q2, float q3, float& px, float& py, float& pz);
	unsigned char legOneFKCalculation(float q1, float q2, float q3, float& px, float& py, float& pz);
	unsigned char legTwoFKCalculation(float q1, float q2, float q3, float& px, float& py, float& pz);
	unsigned char legThreeFKCalculation(float q1, float q2, float q3, float& px, float& py, float& pz);
	unsigned char legFourFKCalculation(float q1, float q2, float q3, float& px, float& py, float& pz);
	unsigned char legFiveFKCalculation(float q1, float q2, float q3, float& px, float& py, float& pz);
	unsigned char legSixFKCalculation(float q1, float q2, float q3, float& px, float& py, float& pz);
	///@}

	/**
	* @function    getAngleWithIK_tanFormula
	* @abstract    calculates the angles q1 [rad/sec], q2 [rad/sec] and q3 [rad/sec] for the desired movement in px [mm], py [mm] and pz [mm]
					A error from this formula can be expected, due to the use of the tan-function. For better results, use the 'getAngleWithIK(...)' function.
	* @param       px, py, py [mm]
	*			   desired coordinates
	* @param       q1, q2, q3 [deg](return-by-reference)
	*			   calculated angles for the desired movement. Angle values are adapted to the servo settings
	* @result      returns an error code, if problems occur. Else it returns 1
	*/
	unsigned char getAngleWithIK_tanFormula(float px, float py, float pz, float& q1, float& q2, float& q3);
	
	/**
	* @function
	* @abstract
	* @param
	* @result
	*/
	unsigned char interpolationAngleEndposition(float qend, float qhome, float (&interpolatedAngleMovement)[10], float& movementSpeed);
	unsigned char interpolationAngleForSyncLinMovement(float deltaQ, float tb, float tv, float *interpolatedAngleMovement, float *interpolatedVelocity, int size);
	unsigned char moveLegOneWithInterpolatedPosition(float q1old,float q2old,float q3old, float q1, float q2, float q3);
	
	/**
	* @function    getAngleWithIK
	* @abstract    calculates the angles q1 [rad/sec], q2 [rad/sec] and q3 [rad/sec] for the desired movement in px [mm], py [mm] and pz [mm]
	* @param       px, py, py [mm]
	*			   desired coordinates
	* @param       q1, q2, q3 [deg](return-by-reference)
	*			   calculated angles for the desired movement. Angle values are adapted to the servo settings
	* @result      returns an error code, if problems occur. Else it returns 1
	*/
	unsigned char getAngleWithIK(float px, float py, float pz, float& q1, float& q2, float& q3);
	
	unsigned char moveLegs(float pxold, float pyold, float pzold, float px, float py, float pz);

	unsigned char doOneStep(float px, float py, float pz);
	unsigned char calculatePath(unsigned char legNumber, float pxold, float pyold, float pzold, float px, float py, float pz, interpolatedMovement &var);
	unsigned char moveAllLegsToHomePos();
	~MovementController();

}; //MovementController

#endif //__MOVEMENTCONTROLLER_H__
