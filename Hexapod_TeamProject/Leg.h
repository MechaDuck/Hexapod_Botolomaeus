/* 
* Leg.h
*
* Created: 22.04.2018 21:07:09
* Author: Tomislav Romic
*/


#ifndef __LEG_H__
#define __LEG_H__

#include "Servo.h"
#include "AX12A.h"
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
															 
*/

class Leg
{
//variables
public:
	AX12A* m_pConnectedBus;
	Servo m_bodyServo;
	Servo m_middleLegServo;
	Servo m_lowerLegServo;
protected:
private:

//functions
public:
	Leg(AX12A& m_pConnectedBus, unsigned char ID_bodyServo, unsigned char ID_middleLegServo, unsigned char ID_lowerLegServo);
	unsigned char setBodyServoAngle(int angle);
	unsigned char setMiddleLegServoAngle(int angle);
	unsigned char setLowerLegServoAngle(int angle);
	~Leg();

}; //Leg

#endif //__LEG_H__
