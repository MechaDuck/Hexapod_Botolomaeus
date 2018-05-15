/* 
* Leg.h
*
* Created: 22.04.2018 21:07:09
* Author: Dr. Tomo
*/


#ifndef __LEG_H__
#define __LEG_H__

#include "Servo.h"
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
protected:
private:
	Servo m_bodyServo;
	Servo m_middleLegServo;
	Servo LowerLegServo;

//functions
public:
	Leg();
	~Leg();

}; //Leg

#endif //__LEG_H__
