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
#include "MotionSequence.h"
#include "PtPMotion.h"
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
enum move_direction {dir_positive=1, dir_negative=-1};
enum move_mode {pushWith236=1, pushWith145=2};
///@{
/**
*@name Servo ID's
*@brief Predefined ID's, that the servos are set to.
*/
#define ID_leg1_lowerLegServo 6
#define ID_leg1_middleLegServo 5
#define ID_leg1_bodyServo 4

#define ID_leg2_lowerLegServo 3
#define ID_leg2_middleLegServo 2
#define ID_leg2_bodyServo 1

#define ID_leg3_lowerLegServo 12
#define ID_leg3_middleLegServo 11
#define ID_leg3_bodyServo 10

#define ID_leg4_lowerLegServo 15
#define ID_leg4_middleLegServo 14
#define ID_leg4_bodyServo 13

#define ID_leg5_lowerLegServo 18
#define ID_leg5_middleLegServo 17
#define ID_leg5_bodyServo 16

#define ID_leg6_lowerLegServo 9
#define ID_leg6_middleLegServo 8
#define ID_leg6_bodyServo 7


#define interpolation_size 10
///@}

/**

Rough structure of the hexapod to understand the naming of the six legs. To find the "UP" position please look on the backside of the robot.
<pre>
\verbatim
leg2	    	      leg1
<===\     /"UP"\     /===>
	 \==|--------|==/
		|--------|
leg4	|--------|	leg3
<=======|--------|=======>
		|--------|
		|--------|
leg6 /==|--------|==\leg5
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
	Leg* pLegs[6];
	///@}
private:
	unsigned char TimerCounter;
	
	
//functions
public:
	MovementController();
	
	unsigned char initRobot();
	
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
	* @function    getAngleWithIK
	* @abstract    calculates the angles q1 [rad/sec], q2 [rad/sec] and q3 [rad/sec] for the desired movement in px [mm], py [mm] and pz [mm]
	* @param       px, py, py [mm]
	*			   desired coordinates
	* @param       q1, q2, q3 [deg](return-by-reference)
	*			   calculated angles for the desired movement. Angle values are adapted to the servo settings
	* @result      returns an error code, if problems occur. Else it returns 1
	*/
	unsigned char getAngleWithIK(float px, float py, float pz, float& q1, float& q2, float& q3);
	

	unsigned char moveAllLegsToHomePosWithLiftingLegs();
	unsigned char doOneStep(float px, float py, float pz,move_mode mode);
	unsigned char doOneStepWith145(float px, float py, float pz);
	unsigned char doOneStepWith236(float px, float py, float pz);
	
	unsigned char calculateLinearMotion(unsigned char legNumber, float pxold, float pyold, float pzold, float px, float py, float pz, MotionSequence &var, move_direction dir);
	unsigned char calculatePtpMotion(unsigned char legNumber, float pxold, float pyold, float pzold, float px, float py, float pz, PtPMotion &var);
	unsigned char moveAllLegsToHomePos();
	
	unsigned char moveLegsSimultanouslyInterpolated(struct MotionSequence (&datalegs)[6]);
	unsigned char moveLegsSimultanouslyInterpolatedWithSpeed(struct MotionSequence (&datalegs)[6], float speed);
												
	unsigned char moveLegs145_SimultanouslyInterpolated(struct MotionSequence dataLeg1, struct MotionSequence dataLeg4, struct MotionSequence dataLeg5);
	unsigned char moveLegs236_SimultanouslyInterpolated(struct MotionSequence dataLeg2, struct MotionSequence dataLeg3, struct MotionSequence dataLeg6);

	unsigned char moveLegsSimultanouslyPtp(struct PtPMotion dataLeg1, struct PtPMotion dataLeg2, struct PtPMotion dataLeg3, struct PtPMotion dataLeg4, struct PtPMotion dataLeg5,struct PtPMotion dataLeg6);
	
	unsigned char lowerLegs(bool lowerLeg1, bool lowerLeg2, bool lowerLeg3, bool lowerLeg4, bool lowerLeg5, bool lowerLeg6);
	unsigned char liftLegs(bool liftLeg1, bool liftLeg2, bool liftLeg3, bool liftLeg4, bool liftLeg5, bool liftLeg6);
	unsigned char moveBodyServosToHome(bool XYHomeLeg1, bool XYHomeLeg2, bool XYHomeLeg3, bool XYHomeLeg4, bool XYHomeLeg5, bool XYHomeLeg6);

	
	//TODO: More or less test functions that were used or were replaced
	unsigned char moveLegs(float pxold, float pyold, float pzold, float px, float py, float pz);
	unsigned char interpolationAngleEndposition(float qend, float qhome, float (&interpolatedAngleMovement)[10], float& movementSpeed);
	unsigned char interpolationAngleForSyncLinMovement(float deltaQ, float tb, float tv, float *interpolatedAngleMovement, float *interpolatedVelocity, int size);
	unsigned char moveLegOneWithInterpolatedPosition(float q1old,float q2old,float q3old, float q1, float q2, float q3);
	unsigned char calculateLinearMotionInverse(unsigned char legNumber, float pxold, float pyold, float pzold, float px, float py, float pz, MotionSequence &var);
	unsigned char calculateInverseLinearMotionWithRaisingLeg(unsigned char legNumber, float pxold, float pyold, float pzold, float px, float py, float pz, MotionSequence &var, float raiseDis);

	~MovementController();

}; //MovementController

#endif //__MOVEMENTCONTROLLER_H__
