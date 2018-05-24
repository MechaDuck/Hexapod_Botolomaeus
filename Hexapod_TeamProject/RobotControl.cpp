/* 
* RobotControl.cpp
*
* Created: 22.04.2018 21:02:48
* Author: Tomislav Romic
*/

//TODO: Change author name
#include "RobotControl.h"

// default constructor
RobotControl::RobotControl(){
	
} //RobotControl

 RobotControl::testFunctions(){
	 double q1,q2,q3,px,py,pz;
	 double beginTime;
	 double endTime;
	 double tmpTime;
	 px=0;
	 py=0;
	 pz=0;

	 /* Test inverse kinematics and interpolation*/
	 double interpolatedArrayOne[10];
	 double interpolatedArrayTwo[10];
	 double interpolatedArrayThree[10];
	 double servoSpeed[3];
	 double _2deg;
	 

	 _2deg=180/M_PI;

	 beginTime=millis();
	 for (int i =0;i<6;i++)
	 {
		 Serial.begin(9600);      // open the serial port at 9600 bps:
		 myMovementController.getAngleWithIK(0,162.9,0,q1,q2,q3);
		 myMovementController.interpolationAngleEndposition(q1,0,interpolatedArrayOne, servoSpeed[0]);
		 myMovementController.interpolationAngleEndposition(q2,0,interpolatedArrayTwo,servoSpeed[1]);
		 myMovementController.interpolationAngleEndposition(q3,0,interpolatedArrayThree,servoSpeed[2]);
		 
		 tmpTime=millis();
		 
		 Serial.print("Done\n");
		 Serial.print("Angles: \n");
		 Serial.print("q1: ");
		 Serial.print(q1*_2deg);
		 Serial.print("\n");
		 Serial.print("q2: ");
		 Serial.print(q2*_2deg);
		 Serial.print("\n");
		 Serial.print("q3: ");
		 Serial.print(q3*_2deg);
		 Serial.print("\n");
		 
		 Serial.print("Speed of Servo 1 (q1): \n");
		 Serial.print(servoSpeed[0]);
		 Serial.print("\n");
		 Serial.print("Interpolated q1: \n");
		 for(unsigned char i=0;i<10;i++){
			 Serial.print(interpolatedArrayOne[i]*_2deg);
			 Serial.print("\n");
		 }
		 
		 Serial.print("Speed of Servo 2 (q2): \n");
		 Serial.print(servoSpeed[1]);
		 Serial.print("\n");
		 Serial.print("Interpolated q2: \n");
		 for(unsigned char i=0;i<10;i++){
			 Serial.print(interpolatedArrayTwo[i]*_2deg);
			 Serial.print("\n");
		 }
		 
		 Serial.print("Speed of Servo 3 (q3): \n");
		 Serial.print(servoSpeed[2]);
		 Serial.print("\n");
		 Serial.print("Interpolated q3: \n");
		 for(unsigned char i=0;i<10;i++){
			 Serial.print(interpolatedArrayThree[i]*_2deg);
			 Serial.print("\n");
		 }
		 beginTime=(millis()-tmpTime)+beginTime;

	 }
	 endTime=millis();
	 Serial.print("Time consumption for calculating all neccessary values: \n");
	 Serial.print((endTime-beginTime)/1000);

	 if(true){
		 while(true){/*Do nothing*/}
	 }
 }

 RobotControl::testServoAdjustment(){
	 while(true){
	 myMovementController.m_Leg1.setBodyServoAngle(210/0.29);
	 delay(3000);
	 myMovementController.m_Leg1.setBodyServoAngle(90/0.29);
	 delay(3000);
	 }

}

// default destructor
RobotControl::~RobotControl()
{
} //~RobotControl
