/* 
* ServoCommunication.cpp
*
* Created: 15.05.2018 18:13:54
* Author: Tomislav Romic
*/


#include "ServoCommunication.h"
//TODO: Test settings, will be changed later
#define DirectionPin 	(10u)
#define BaudRate  		(1000000ul)
#define ID				(1u)



// default constructor
ServoCommunication::ServoCommunication()
{
} //ServoCommunication

unsigned char ServoCommunication::hardResetServos(){
	
	//DANGER//
	/*Changing this function may cause serious damages to the hexapod*/
	//TODO: Set IDs !DIFFICULT! Servo ID are set to 1 by default, so every servo needs to be connected separately. Create new file for setting IDs
	//TODO: Set BAUD rate
	
	
}

// default destructor
ServoCommunication::~ServoCommunication()
{
} //~ServoCommunication
