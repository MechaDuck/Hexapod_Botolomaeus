/* 
* RobotControl.cpp
*
* Created: 22.04.2018 21:02:48
* Author: Tomislav Romic
*/

//TODO: Change author name
#include "RobotControl.h"
#include "BluetoothInterface.h"


#define BluetoothPollingFreq 1000 /*ms*/

// default constructor
RobotControl::RobotControl(){
	
} //RobotControl

 RobotControl::run(){	
	myMovementController.robotCur_state=st_initStep;
	myMovementController.doContinuesSteps(50,0,0);
	
	while(true){
		myMovementController.robotCur_state=st_lift236AndGoHomeAndLower236AndPush145;
		myMovementController.doContinuesSteps(50,0,0);
		myMovementController.robotCur_state=st_lift145AndGoHomeAndLower145AndPush236;
		myMovementController.doContinuesSteps(50,0,0);
	}
}

 RobotControl::testFunctions(){
	 float q1,q2,q3,px,py,pz;
	 float beginTime;
	 float endTime;
	 float tmpTime;
	 px=0;
	 py=0;
	 pz=0;

	 /* Test inverse kinematics and interpolation*/
	 float interpolatedArrayOne[10];
	 float interpolatedArrayTwo[10];
	 float interpolatedArrayThree[10];
	 float servoSpeed[3];
	 float _2deg;
	 

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


 RobotControl::testSimpleBluetooth(){
	Serial.begin(9600);      // open the serial port at 9600 bps:
	while(true){
		myBluetoothInterface.readInput();
		
		delay(200);
	}
}

 RobotControl::testInverseKinematic(){
	 float pkx,pky,pkz,q1,q2,q3;
	 delay(2000);
	 Serial.begin(9600);

	 delay(5000);

	 myMovementController.getLegCoordinatesFromWorldCoordinates('3',0,157.8,0,200,0,0,pkx,pky,pkz);
	 Serial.println("pkx, pky, pkz");
	Serial.println(pkx);
	Serial.println(pky);
	Serial.println(pkz);
	 myMovementController.getAngleWithIK(pkx,pky,pkz,q1,q2,q3);
	 Serial.println("q1, q2, q3 from inverted kinematics");
	 Serial.println(q1*180/M_PI);
	 Serial.println(q2*180/M_PI);
	 Serial.println(q3*180/M_PI);
	 Serial.println("q1, q2, q3 servo values (with offset)");
	 // convert q1, q2, q3 which we get from inverted kinematics to the angles which we send to the servos
	 float q1Servo = 60.0+(q1*180.0/M_PI); //offset = 60: 90 in kinematic -> 150 in Servo
	 float q2Servo = 150.0-(q2*180.0/M_PI);//offset = 150: 0 in kinematic -> 150 in Servo
	 float q3Servo = 150.0+(q3*180.0/M_PI)+36.3;// 36.3 constant angle. //offset = 150: 0 in kinematic -> 150 + 36.3 in Servo (because of construction)
	 
	Serial.println(q1Servo);
	Serial.println(q2Servo);
	Serial.println(q3Servo);
	
	/***********************************************/
	/*************next step interpoltion************/
	/* 
	* as defines we have: Vm_tmp, bm_tmp, T_IPO
	* 1. get Se = max(q1-q1_home, q2-q2_home, q3-q3_home)
	* 2. check Vm_tmp > sqrt(Se*bm_tmp) : YES -> go to 3 , NO -> got to 4
	* 3. set Vm_tmp = sqrt(Se*bm_tmp)
	* 4. calculate tb = Ceil(Vm_tmp/(bm_tmp*T_IPO)) * T_IPO
	* 5. calculate tv = Ceil(Se/(Vm_tmp*T_IPO)) * T_IPO
	* 6. calculate te = tb + tv (the same for all servos!!)
	* 7. calculate Vm1 = Se1/tv; Vm2 = Se2/tv; Vm3 = Se3/tv;
	* 8. calculate bm1 = Vm1/tb; bm2 = Vm2/tb; bm3 = Vm3/tb;
	* now we have all the values to calculate the interpolations: Vm1, Vm2, Vm3 bm1, bm2, bm3, te, tb, tv
	* 1. define the time step: t = a * T_IPO -> a = ... 0.001, 0.01, 0.1, 1, 10 , 100 ....
	* 2.open a loop: while(t <= te){
		if(t >= 0 && t < tb)
		{
			q1_Ipo = 0.5 * bm1 * t * t;
			q2_Ipo = 0.5 * bm2 * t * t;
			q3_Ipo = 0.5 * bm3 * t * t;
			
			V_q1_Ipo = bm1 * t;
			V_q2_Ipo = bm2 * t;
			V_q3_Ipo = bm3 * t;
			
			//send values to all servos
		}
		
		if(t >= tb && t < tv)
		{
			q1_Ipo = (Vm1 * t) - (0.5 * Vm1 * Vm1 / bm1);
			q2_Ipo = (Vm2 * t) - (0.5 * Vm2 * Vm2 / bm2);
			q3_Ipo = (Vm3 * t) - (0.5 * Vm3 * Vm3 / bm3);
			
			V_q1_Ipo = Vm1;
			V_q2_Ipo = Vm2;
			V_q3_Ipo = Vm3;
			
			//send values to all servos
		}
		
		TODO
	
	*/
	/***********************************************/
	
	 myMovementController.m_Leg1.setBodyServoAngle(q1Servo);
	 myMovementController.m_Leg1.setMiddleServoAngle(q2Servo);
	 myMovementController.m_Leg1.setLowerServoAngle(q3Servo);	 

}

 RobotControl::test_interpolationAngleForSyncLinMovement(){
	Serial.begin(9600);
	Serial.println("Start:");
	delay(5000);
	float q1, q2,q3,oldq1,oldq2,oldq3,pkx,pky,pkz;
	myMovementController.getLegCoordinatesFromWorldCoordinates('3',0,157.8,0,120,0,0,pkx,pky,pkz);
	myMovementController.getAngleWithIK(pkx,pky,pkz,q1,q2,q3);
	Serial.println("Desired position: ");
	float q1Servo = 60.0+(q1); //offset = 60: 90 in kinematic -> 150 in Servo
	float q2Servo = 150.0-(q2);//offset = 150: 0 in kinematic -> 150 in Servo
	float q3Servo = 150.0+(q3)+36.3;// 36.3 constant angle. //offset = 150: 0 in kinematic -> 150 + 36.3 in Servo (because of construction)
	Serial.print(q1Servo);
	Serial.print(q2Servo);
	Serial.print(q3Servo);
	Serial.println();
	
 	myMovementController.moveLegs(0,157.8,0,120,0,0);
// 	delay(5000);
// 	myMovementController.move2HomePosition();
// 	delay(2000);
// 	myMovementController.calculateWorldMovement2LegMovement('3',0,157.8,0,120,0,0,pkx,pky,pkz);
// 	myMovementController.getAngleWithIK(pkx,pky,pkz,q1,q2,q3);
// 	
// 	q1Servo = 60.0+(q1);
// 	q2Servo = 150.0-(q2);
// 	q3Servo = 150.0+(q3)+36.3;
// 	myMovementController.m_Leg1.setBodyServoAngle(q1Servo);
// 	myMovementController.m_Leg1.setMiddleLegServoAngle(q2Servo);
// 	myMovementController.m_Leg1.setLowerLegServoAngle(q3Servo);
}

 RobotControl::test_timeConsumptionOfOneStepCalculations(){
	float endTime, beginTime;
	Serial.begin(9600);
	beginTime=millis();
	myMovementController.doOneStep(0,160.0,0,pushWith145); 
	 
	endTime=millis();
	Serial.print("Time consumption for calculating all necessary values: \n");
	Serial.print((endTime-beginTime)/1000);

}
RobotControl::test_ICS(){
	Serial.begin(9600);
	int angle;
	while(true){
		Serial.print("Voltage");
		angle=myMovementController.m_Leg4.m_bodyServo.getVoltage();
		Serial.println(angle );
		delay(1000);
		myMovementController.m_Leg4.m_bodyServo.setServoAngleAndSpeed(120,100);
		delay(1000);
		myMovementController.m_Leg4.m_bodyServo.setServoAngleAndSpeed(150,100);
	}
}

 RobotControl::test_stepMachine()
{
	//myMovementController.moveAllLegsToHomePosWithLiftingLegs();
	//delay(2000);
	
	for(int i=0;i < 11;i++){
			//myMovementController.doOneCompleteStep(30,0,0);
	}

	delay(150);
}

 RobotControl::test_allLegsTogether()
{
	PtPMotion ptpMove[6]={PtPMotion(SingleSpeed,EnablePositionTracking),
						  PtPMotion(SingleSpeed,EnablePositionTracking),
						  PtPMotion(SingleSpeed,EnablePositionTracking),
						  PtPMotion(SingleSpeed,EnablePositionTracking),
						  PtPMotion(SingleSpeed,EnablePositionTracking),
						  PtPMotion(SingleSpeed,EnablePositionTracking)};

	for(int i=0;i<6;i++){
		ptpMove[i].setAngleSequenceAt(1,150,150,150);
		ptpMove[i].setMotionSequenceAt(1,20,20,20);
	}
	myMovementController.moveAllLegsToHomePos();
 	myMovementController.moveLegsSimultanouslyPtp(ptpMove[0],ptpMove[1],ptpMove[2],ptpMove[3],ptpMove[4],ptpMove[5]);

}

 RobotControl::test_leg1CorrectMovement(){
	 Leg *pleg1, *pleg2, *pleg3, *pleg4, *pleg5, *pleg6;
	 pleg1 = &myMovementController.m_Leg1;
	 pleg2 = &myMovementController.m_Leg2;
	 pleg3 = &myMovementController.m_Leg3;
	 pleg4 = &myMovementController.m_Leg4;
	 pleg5 = &myMovementController.m_Leg5;
	 pleg6 = &myMovementController.m_Leg6;
#if 0
	 Serial.begin(9600);
	 
 //myMovementController.moveAllLegsToHomePosWithLiftingLegs();
// 	MotionSequence movLeg1neg(15,SingleSpeed,EnablePositionTracking);
// 	MotionSequence movLeg3neg(15,SingleSpeed,EnablePositionTracking);
// 	MotionSequence movLeg4neg(15,SingleSpeed,EnablePositionTracking);
// 		delay(2000);
// 	myMovementController.m_Leg3.move2HomePosition();
// 	myMovementController.m_Leg4.move2HomePosition();
// 		delay(2000);
// 	myMovementController.calculateLinearMotion('1',0,0,0,20,0,0,movLeg1neg,dir_positive);
// 	myMovementController.calculateLinearMotion('1',0,0,0,20,0,0,movLeg4neg,dir_positive);
// 	myMovementController.calculateLinearMotion('1',0,0,0,20,0,0,movLeg3neg,dir_positive);
// 	for(int i=0; i< movLeg1neg.getSize();i++){
// 		Serial.print("i:");
// 		Serial.print(i);
// 		Serial.print(";;");
// 		Serial.print("Q1: ");
// 		Serial.print(movLeg1neg.getQ1AngleSequenceAt(i));
// 		Serial.print(";;");
// 		Serial.print("Q2: ");
// 		Serial.print(movLeg1neg.getQ2AngleSequenceAt(i));
// 		Serial.print(";;");
// 		Serial.print("Q3: ");
// 		Serial.print(movLeg1neg.getQ3AngleSequenceAt(i));
// 		Serial.print(";;");
// 		Serial.print("X: ");
// 		Serial.print(movLeg1neg.getXMotionSequenceAt(i));
// 		Serial.print(";;");
// 		Serial.print("Y: ");
// 		Serial.print(movLeg1neg.getYMotionSequenceAt(i));
// 		Serial.print(";;");
// 		Serial.print("Z: ");
// 		Serial.print(movLeg1neg.getZMotionSequenceAt(i));
// 		Serial.print(";;");
// 		Serial.println();	
// 	}
// 	for(int i=0;i < movLeg1neg.getSize();i++){
// 			myMovementController.m_Leg1.moveLegToKnownPosition(movLeg4neg.getQ1AngleSequenceAt(i),movLeg4neg.getQ2AngleSequenceAt(i),movLeg4neg.getQ3AngleSequenceAt(i));
// 			//myMovementController.m_Leg3.moveLegToKnownPosition(movLeg3neg.getQ1AngleSequenceAt(i),movLeg3neg.getQ2AngleSequenceAt(i),movLeg3neg.getQ3AngleSequenceAt(i));
// 			delay(50);
// 	}
#else
//////////////////////////
pleg1->move2HomePosition();
pleg2->move2HomePosition();
pleg3->move2HomePosition();
pleg4->move2HomePosition();
pleg5->move2HomePosition();
pleg6->move2HomePosition();
	delay(2000);
	float q1,q2,q3;
	float x, y,z, pkx,pky,pkz;
	x=-50;
	y=0;
	z=0;
	//Leg 1
	myMovementController.getLegCoordinatesFromWorldCoordinates('1',0,0,0,x,y,z,pkx,pky,pkz);
	myMovementController.getAngleWithIK(pkx,pky,pkz,q1,q2,q3);
	//Conversion from mathematical model to real physical model. Angles have an offset that are compensated with following calculations.
	myMovementController.conversionFromMathematicalModelToMechanicalModel('1',q1,q2,q3);
	pleg1->moveLegToKnownPosition(q1,q2,q3,0,0,0);
	
// 	//Leg 2
// 	myMovementController.getAngleWithIK(-50,0,0,q1,q2,q3);
// 	//Conversion from mathematical model to real physical model. Angles have an offset that are compensated with following calculations.
// 	myMovementController.conversionFromMathematicalModelToMechanicalModel('2',q1,q2,q3);
// 	pleg2->moveLegToKnownPosition(q1,q2,q3,0,0,0);
	
	//Leg 3
	myMovementController.getLegCoordinatesFromWorldCoordinates('3',0,0,0,x,y,z,pkx,pky,pkz);
	myMovementController.getAngleWithIK(pkx,pky,pkz,q1,q2,q3);
	//Conversion from mathematical model to real physical model. Angles have an offset that are compensated with following calculations.
	myMovementController.conversionFromMathematicalModelToMechanicalModel('3',q1,q2,q3);
	pleg3->moveLegToKnownPosition(q1,q2,q3,0,0,0);
	
	//Leg 5
	myMovementController.getLegCoordinatesFromWorldCoordinates('5',0,0,0,x,y,z,pkx,pky,pkz);
	myMovementController.getAngleWithIK(pkx,pky,pkz,q1,q2,q3);
	//Conversion from mathematical model to real physical model. Angles have an offset that are compensated with following calculations.
	myMovementController.conversionFromMathematicalModelToMechanicalModel('5',q1,q2,q3);
	pleg5->moveLegToKnownPosition(q1,q2,q3,0,0,0);	
	
	
	////Leg 3
		//myMovementController.getAngleWithIK(50,157.8,0,q1,q2,q3);
		////Conversion from mathematical model to real physical model. Angles have an offset that are compensated with following calculations.
		//q1 = 300-(60.0+(q1)); //offset = 60: 90 in kinematic -> 150 in Servo
		//q2 = 150.0-(q2);//offset = 150: 0 in kinematic -> 150 in Servo
		//q3 = 186.3+(q3);// 36.3 constant angle. //offset = 150: 0 in kinematic -> 150 + 36.3 in Servo (because of construction)
		//pleg3->moveLegToKnownPosition(q1,q2,q3);
		//
	////Leg 4
		//myMovementController.getAngleWithIK(50,157.8,0,q1,q2,q3);
		////Conversion from mathematical model to real physical model. Angles have an offset that are compensated with following calculations.
		//q1 = 60.0+(q1); //offset = 60: 90 in kinematic -> 150 in Servo
		//q2 = 150.0-(q2);//offset = 150: 0 in kinematic -> 150 in Servo
		//q3 = 186.3+(q3);// 36.3 constant angle. //offset = 150: 0 in kinematic -> 150 + 36.3 in Servo (because of construction)
		//pleg4->moveLegToKnownPosition(q1,q2,q3);
		
		delay(2000);
		
#endif	
	


}

 RobotControl::test_raiseLeg(){
	 MotionSequence movLeg1neg(10,SpeedSequence,EnablePositionTracking);
	myMovementController.calculateLinearMotionWithRaisingLeg2('2',-30,-30,0,0,0,0,movLeg1neg,30,50);
	for(int i=0; i< movLeg1neg.getSize();i++){
		Serial.print("i:");
		Serial.print(i);
		Serial.print(";;");
		Serial.print("Q1: ");
		Serial.print(movLeg1neg.getQ1AngleSequenceAt(i));
		Serial.print(";;");
		Serial.print("Q2: ");
		Serial.print(movLeg1neg.getQ2AngleSequenceAt(i));
		Serial.print(";;");
		Serial.print("Q3: ");
		Serial.print(movLeg1neg.getQ3AngleSequenceAt(i));
		Serial.print(";;");
		Serial.print("X: ");
		Serial.print(movLeg1neg.getXMotionSequenceAt(i));
		Serial.print(";;");
		Serial.print("Y: ");
		Serial.print(movLeg1neg.getYMotionSequenceAt(i));
		Serial.print(";;");
		Serial.print("Z: ");
		Serial.print(movLeg1neg.getZMotionSequenceAt(i));
		Serial.print(";;");
		Serial.println();
	}
}

 RobotControl::test_checkMegaState(){
	myMovementController.robotCur_state=st_initStep;
	myMovementController.doContinuesSteps(-50,-50,0);
	
	while(true){
// 		for(int i=0; i< 4;i++){
// 				myMovementController.robotCur_state=st_lift236AndGoHomeAndLower236AndPush145;
// 				myMovementController.doContinuesSteps(50,0,0);
// 				myMovementController.robotCur_state=st_lift145AndGoHomeAndLower145AndPush236;
// 				myMovementController.doContinuesSteps(50,0,0);
// 		}
// 		
		for(int i=0; i<4;i++){
				myMovementController.robotCur_state=st_lift236AndGoHomeAndLower236AndPush145;
				myMovementController.doContinuesSteps(-50,-50,0);
				myMovementController.robotCur_state=st_lift145AndGoHomeAndLower145AndPush236;
				myMovementController.doContinuesSteps(-50,-50,0);
		}
	}
	
	
// 	MotionSequence motionLeg1(interpolation_size,SpeedSequence,EnablePositionTracking);
// 	MotionSequence motionLeg4(interpolation_size,SpeedSequence,EnablePositionTracking);
// 	MotionSequence motionLeg5(interpolation_size,SpeedSequence,EnablePositionTracking);
// 	MotionSequence motionLeg2(interpolation_size,SpeedSequence,EnablePositionTracking);
// 	MotionSequence motionLeg3(interpolation_size,SpeedSequence,EnablePositionTracking);
// 	MotionSequence motionLeg6(interpolation_size,SpeedSequence,EnablePositionTracking);
// 	
// 	MotionSequence* motionAllLegs[6];
// 	motionAllLegs[0]=&motionLeg1;
// 	motionAllLegs[1]=&motionLeg2;
// 	motionAllLegs[2]=&motionLeg3;
// 	motionAllLegs[3]=&motionLeg4;
// 	motionAllLegs[4]=&motionLeg5;
// 	motionAllLegs[5]=&motionLeg6;
// 		
// 	float px=50,py=0,pz=0;
// 	
// 	float pkXold,pkYold,pkZold;
// 	
// 	myMovementController.calculateLinearMotionWithRaisingLeg2('1',0,0,0,0,0,0,motionLeg1,50);
// 	myMovementController.moveLegsSimultanouslyInterpolatedWithSpeed(motionAllLegs);
// 	delay(2000);
// 	
// 	myMovementController.m_Leg1.getCurrentPos(pkXold,pkYold,pkZold);
// 	myMovementController.calculateLinearMotion('1',pkXold,pkYold,pkZold,px,py,pz,motionLeg1,dir_Xnegative,dir_Ypositive,dir_Znegative);
// 	myMovementController.moveLegsSimultanouslyInterpolatedWithSpeed(motionAllLegs);
// 	delay(2000);
// 	
// 	myMovementController.m_Leg1.getCurrentPos(pkXold,pkYold,pkZold);
// 	myMovementController.calculateLinearMotionWithRaisingLeg2('1',pkXold,pkYold,pkZold,0,0,0,motionLeg1,50);
// 	myMovementController.moveLegsSimultanouslyInterpolatedWithSpeed(motionAllLegs);
// 	delay(2000);
	

}

 RobotControl::test_blueoothCommunication(){
	myBluetoothInterface.readInput();
	myMovementController.robotCur_state=st_initStep;
	myMovementController.doContinuesSteps(myBluetoothInterface.getDirectionX(),myBluetoothInterface.getDirectionY(),0);
	
	while(true){
			myBluetoothInterface.readInput();
			myMovementController.robotCur_state=st_lift236AndGoHomeAndLower236AndPush145;
			myMovementController.doContinuesSteps(myBluetoothInterface.getDirectionX(),myBluetoothInterface.getDirectionY(),0);
			Serial.println(myBluetoothInterface.getDirectionX());
			
			myBluetoothInterface.readInput();
			myMovementController.robotCur_state=st_lift145AndGoHomeAndLower145AndPush236;
			myMovementController.doContinuesSteps(myBluetoothInterface.getDirectionX(),myBluetoothInterface.getDirectionY(),0);
}
}

 RobotControl::test_rotation(){
	 	myMovementController.robotCur_state=st_initStep;
	 	myMovementController.doContinuesRotation(-40.0/180.0*M_PI);
	 	
	 	while(true){
		 	myMovementController.robotCur_state=st_lift236AndGoHomeAndLower236AndPush145;
		 	myMovementController.doContinuesRotation(-40.0/180.0*M_PI);
		 	
		 	myMovementController.robotCur_state=st_lift145AndGoHomeAndLower145AndPush236;
		 	myMovementController.doContinuesRotation(-40.0/180.0*M_PI);
		 }

}

 RobotControl::test_megaStep(){
	myMovementController.robotCur_state=st_initStep;
	myMovementController.doContinuesSteps(0,0,0);
	
	while(true){
		for(int i=0; i<4 ; i++){
				myMovementController.robotCur_state=st_lift236AndGoHomeAndLower236AndPush145;
				myMovementController.doContinuesSteps(50,0,0);
				
				myMovementController.robotCur_state=st_lift145AndGoHomeAndLower145AndPush236;
				myMovementController.doContinuesSteps(50,0,0);
		}
		for(int i=0; i<4 ; i++){
			myMovementController.robotCur_state=st_lift236AndGoHomeAndLower236AndPush145;
			myMovementController.doContinuesSteps(0,50,0);
			
			myMovementController.robotCur_state=st_lift145AndGoHomeAndLower145AndPush236;
			myMovementController.doContinuesSteps(0,50,0);
		}
		for(int i=0; i<4 ; i++){
			myMovementController.robotCur_state=st_lift236AndGoHomeAndLower236AndPush145;
			myMovementController.doContinuesSteps(0,-50,0);
					
			myMovementController.robotCur_state=st_lift145AndGoHomeAndLower145AndPush236;
			myMovementController.doContinuesSteps(0,-50,0);
		}
		for(int i=0; i<4 ; i++){
			myMovementController.robotCur_state=st_lift236AndGoHomeAndLower236AndPush145;
			myMovementController.doContinuesSteps(-50,0,0);
			
			myMovementController.robotCur_state=st_lift145AndGoHomeAndLower145AndPush236;
			myMovementController.doContinuesSteps(-50,0,0);
		}
		for(int i=0;i<4;i++){
			myMovementController.robotCur_state=st_lift236AndGoHomeAndLower236AndPush145;
			myMovementController.doContinuesRotation(-40.0/180.0*M_PI);
			
			myMovementController.robotCur_state=st_lift145AndGoHomeAndLower145AndPush236;
			myMovementController.doContinuesRotation(-40.0/180.0*M_PI);
		}
		for(int i=0;i<4;i++){
			myMovementController.robotCur_state=st_lift236AndGoHomeAndLower236AndPush145;
			myMovementController.doContinuesRotation(40.0/180.0*M_PI);
			
			myMovementController.robotCur_state=st_lift145AndGoHomeAndLower145AndPush236;
			myMovementController.doContinuesRotation(40.0/180.0*M_PI);
		}
		
	}
}

// default destructor
RobotControl::~RobotControl()
{
} //~RobotControl