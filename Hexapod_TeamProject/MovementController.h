/* 
* MovementController.h
*
* Created: 22.04.2018 21:39:35
* Author: Tomislav Romic
*/


#ifndef __MOVEMENTCONTROLLER_H__
#define __MOVEMENTCONTROLLER_H__

#include "Leg.h"
#include "Arduino.h"
#include "AX12A.h"

//Settings for the UART communication with the Servos
//TODO: Test settings, will be changed later
#define DirectionPinForBus1 	(10u)
#define BaudRate  				(1000000ul)
#define ServoSerialForBus1		(Serial1)

#define DirectionPinForBus2 	(10u)
#define BaudRate  				(1000000ul)
#define ServoSerialForBus2		(Serial2)

#define DirectionPinForBus3 	(10u)
#define BaudRate  				(1000000ul)
#define ServoSerialForBus3		(Serial3)

#define ID_leg1_lowerLegServo 1
#define ID_leg1_middleLegServo 2
#define ID_leg1_bodyServo 3

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
/*

Rough structure of the hexapod to understand the naming of the six legs. To find the "UP" position please look on the backside of the robot.

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

//TODO: In this example leg1 and leg2 are connected to Serial1. Leg3 and leg4 are connected to Serial2. Leg5 and leg6 are connected to Serial3.
/*   

Rough structure of the hexapod to understand the naming of the three servos on one leg. 

         (LeftLeg)                      <Hexapod>                        (RightLeg)	

							/////######################\\\\\		
	   _____________	 BodyServo				     BodyServo          _____________
	   LowerLegServo	  /////#########################\\\\\		    LowerLegServo
	 /////      \\\\\    /////							 \\\\\		  /////      \\\\\
	/////		 \\\\\  /////							  \\\\\		 /////		  \\\\\
   /////		MiddleLegServo		     				MiddleLegServo		       \\\\\
____\/__________________________________ Ground______________________________________\/______
															 
*/
class MovementController
{
//variables
public:
	AX12A m_ServoBus12;
	AX12A m_ServoBus34;
	AX12A m_ServoBus56;
	
	Leg m_Leg1;
	Leg m_Leg2;
	Leg m_Leg3;
	Leg m_Leg4;
	Leg m_Leg5;
	Leg m_Leg6;
protected:
private:

//functions
public:
	MovementController();
	
	unsigned char world2LegCoordinateSystemWithFK(unsigned char legNumber, double q1, double q2, double q3, double& px, double& py, double& pz);
	unsigned char legOneFKCalculation(double q1, double q2, double q3, double& px, double& py, double& pz);
	unsigned char legTwoFKCalculation(double q1, double q2, double q3, double& px, double& py, double& pz);
	unsigned char legThreeFKCalculation(double q1, double q2, double q3, double& px, double& py, double& pz);
	unsigned char legFourFKCalculation(double q1, double q2, double q3, double& px, double& py, double& pz);
	unsigned char legFiveFKCalculation(double q1, double q2, double q3, double& px, double& py, double& pz);
	unsigned char legSixFKCalculation(double q1, double q2, double q3, double& px, double& py, double& pz);
	/*@function    getAngleWithIK
	* @abstract    calculates the angles q1 [rad/sec], q2 [rad/sec] and q3 [rad/sec] for the desired movement in px [mm], py [mm] and pz [mm]
	* @param       px, py, py
	*			   desired coordinates
	* @param       q1, q2, q3 (return-by-reference)
	*			   calculated angles for the desired movement

	* @result      returns an error code

	*/
	unsigned char getAngleWithIK_tanFormula(double px, double py, double pz, double& q1, double& q2, double& q3);
	/*@function
	* @abstract
	* @param
	* @result
	*
	*
	
	*/
	unsigned char interpolationAngleEndposition(double qend, double qhome, double (&interpolatedAngleMovement)[10], double& movementSpeed);
	
	unsigned char getAngleWithIK(double px, double py, double pz, double& q1, double& q2, double& q3);
	
	~MovementController();

}; //MovementController

#endif //__MOVEMENTCONTROLLER_H__
