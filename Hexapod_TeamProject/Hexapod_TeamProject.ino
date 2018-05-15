/*
 * Hexapod_TeamProject.ino
 *
 * Created: 4/22/2018 9:01:43 PM
 * Author: Tomislav Romic
 */ 
#include "MovementController.h"
MovementController myMovementController;
double q1,q2,q3,px,py,pz;
double beginTime;
double endTime;
double tmpTime;

void setup()
{
	/* add setup code here, setup code runs once when the processor starts */
Serial.begin(9600);      // open the serial port at 9600 bps:
px=0;
py=0;
pz=0;
}

void loop()
{

	  /* add main program code here, this code starts again each time it ends */


/*Test reaction time of ínverse kinemati calculations*/
	  
// 	  myMovementController.getAngleWithIK(px,py,pz,q1,q2,q3);
// 	  Serial.print("Done\n");
// 	  Serial.print(q1);
// 	  Serial.print(q2);
// 	  Serial.print(q3);
// 	  px=px+0.01;
// 	  py=px+0.01;
// 	  pz=px+0.01;
// 	  if(px>1){
// 		  while(true){/*Do nothing*/}
// 	  }

/* Test inverse kinematics and interpolation*/
	double interpolatedArrayOne[10];
	double interpolatedArrayTwo[10];
	double interpolatedArrayThree[10];
	
	beginTime=millis();
	for (int i =0;i<6;i++)
	{
		myMovementController.getAngleWithIK(1,1,0,q1,q2,q3);
		myMovementController.interpolationAngleEndposition(q1,0,interpolatedArrayOne);
		myMovementController.interpolationAngleEndposition(q2,0,interpolatedArrayTwo);
		myMovementController.interpolationAngleEndposition(q3,0,interpolatedArrayThree);
		
		tmpTime=millis();
		Serial.print("Done\n");
		Serial.print(q1);
		Serial.print("\n");
		Serial.print(q2);
		Serial.print("\n");
		Serial.print(q3);
		Serial.print("\n");
		
		for(unsigned char i=0;i<10;i++){
			Serial.print(interpolatedArrayOne[i]);
			Serial.print("\n");
		}
		for(unsigned char i=0;i<10;i++){
			Serial.print(interpolatedArrayTwo[i]);
			Serial.print("\n");
		}
		for(unsigned char i=0;i<10;i++){
			Serial.print(interpolatedArrayThree[i]);
			Serial.print("\n");
		}
		beginTime=(millis()-tmpTime)+beginTime;

	}
	endTime=millis();
	Serial.print((endTime-beginTime)/1000);
	if(true){
		while(true){/*Do nothing*/}
	}

}

void testFunctions(){

}