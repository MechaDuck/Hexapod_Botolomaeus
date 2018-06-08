/* 
* MovementController.cpp
*
* Created: 22.04.2018 21:39:35
* Author: Tomislav Romic
*/

/**
@file MovementController.cpp
*/

#include "MovementController.h"
#include "AX12A.h"
#include <math.h>


/*--------------------------------------------------------------------------------------------*/
/*Constant values for the calculation of inverse kinematics*/
//Properties of the body (see documentation for definition)

///@{
/** 
*@name constant leg parameters
*@brief vector parts that starts from the origin of the world coordinate system to an origin of a leg coordinate system.
*	Can be used for every leg, because the robot is symmetrical with respect to the x-axes AND y-axes. Only the sign of those variables varies.
*	(see documentation for further definition)
*/

#define pwkx 116.41 
#define pwky 79.379 
#define pwkz 666 /*TODO: Unclear which parameter goes here!!!*/
#define pwkAngle (30.0/180.0)*M_PI
///@}



//Properties of the legs (see documentation for definition)

#define l01a		119.845		/*mm*/
#define l01b		47.75	 	/*mm*/
#define l12			76.395      /*mm*/
#define l23			204.33	    /*mm*/
#define l12quad		pow(l12,2)
#define l23quad		pow(l23,2)
//Properties of the servos (see documentation for definition)
#define bmax		4956.084 /*deg/sec^2*/ //TODO: Need to be calculated again!! In degrees/sec^2
#define vmax		684.0/2 /*deg/sec^2*/ // TODO: Need to be calculated again!! In degress/sec

//Constant values for kinematic calculations
#define c_k			sqrt(pow(l01a,2)+pow(l01b,2))
#define c_phi		M_PI / 6.0 //TODO: Need to be evaluated
#define c_kquad		pow(c_k,2)
#define c_gamma		atan(l01b/l01a)
#define c_delta		atan(l01a/l01b)

//Variables for interpolation calculations
#define T_IPO		0.01f
/*--------------------------------------------------------------------------------------------*/

 MovementController::MovementController():
	m_Leg1(m_ServoComObject1, ID_leg1_bodyServo, ID_leg1_middleLegServo, ID_leg1_lowerLegServo),
	m_Leg2(m_ServoComObject1, ID_leg2_bodyServo, ID_leg2_middleLegServo, ID_leg2_lowerLegServo),
	m_Leg3(m_ServoComObject1, ID_leg3_bodyServo, ID_leg3_middleLegServo, ID_leg3_lowerLegServo),
	m_Leg4(m_ServoComObject1, ID_leg4_bodyServo, ID_leg4_middleLegServo, ID_leg4_lowerLegServo),
	m_Leg5(m_ServoComObject1, ID_leg5_bodyServo, ID_leg5_middleLegServo, ID_leg5_lowerLegServo),
	m_Leg6(m_ServoComObject1, ID_leg6_bodyServo, ID_leg6_middleLegServo, ID_leg6_lowerLegServo){
		
	initRobot();

	
}



unsigned char MovementController::initRobot(){
	//Enable and initialize UART communication with Leg-Servos
	m_ServoComObject1.begin(BaudRate,DirectionPinForBus1,&ServoSerialForBus1);
	//TODO: Set max and min angles. set max speed.
}

unsigned char MovementController::move2HomePosition(){
	m_Leg1.move2HomePosition();
}

unsigned char MovementController::getLegCoordinatesFromWorldCoordinates(unsigned char legNumber,float pkXold, float pkYold, float pkZold, float pwX, float pwY, float pwZ, float& pkX, float& pkY, float& pkZ){
		switch (legNumber){
			case '1':
				worldPositionToLegOnePosition(pkXold,pkYold,pkZold,pwX,pwY,pwZ,pkX,pkY,pkZ);
			break;
			case '2':
				worldPositionToLegTwoPosition(pkXold,pkYold,pkZold,pwX,pwY,pwZ,pkX,pkY,pkZ);
			break;
			case '3':
			    worldPositionToLegThreePosition(pkXold,pkYold,pkZold,pwX,pwY,pwZ,pkX,pkY,pkZ);
			break;
			case '4':
				worldPositionToLegFourPosition(pkXold,pkYold,pkZold,pwX,pwY,pwZ,pkX,pkY,pkZ);
			break;
			case '5':
				worldPositionToLegFivePosition(pkXold,pkYold,pkZold,pwX,pwY,pwZ,pkX,pkY,pkZ);
			break;
			case '6':
				worldPositionToLegSixPosition(pkXold,pkYold,pkZold,pwX,pwY,pwZ,pkX,pkY,pkZ);
			break;
			default:
				return 0;
			break;
		}
		return 1;
}

unsigned char MovementController::worldPositionToLegOnePosition(float pkXold, float pkYold, float pkZold, float pwX, float pwY, float pwZ, float& pkX, float& pkY, float& pkZ){

	pkX=(pwX*cos(c_phi))+(pwY*sin(c_phi))+pkXold;
	pkY=pwX*sin(c_phi)-pwY*cos(c_phi)+pkYold;
	pkZ=(-1)*pwZ+pkZold;
	
	
	//TODO:define sin(phi) etc:....

}

unsigned char MovementController::worldPositionToLegThreePosition(float pkXold, float pkYold, float pkZold, float pwX, float pwY, float pwZ, float& pkX, float& pkY, float& pkZ){
	pkX=pwX+pkXold;
	pkY=pkYold-pwY;
	pkZ=pkZold-pwZ;
}

unsigned char MovementController::getWorldCoordinatesFromFK(unsigned char legNumber, float q1, float q2, float q3, float& px, float& py, float& pz){
	switch (legNumber){
		case '0': 
			legOneFKCalculation(q1, q2, q3, px, py, pz);
			break;
		case '2':
			legTwoFKCalculation(q1, q2, q3, px, py, pz);
			break;
		case '3':
			legThreeFKCalculation(q1,q2,q3,px,py,pz);
			break;
		case '4':
			legFourFKCalculation(q1, q2, q3, px, py, pz);
			break;
		case '5':
			legFiveFKCalculation(q1, q2, q3, px, py, pz);
			break;
		case '6':
			legSixFKCalculation(q1, q2, q3, px, py, pz);
			break;
		default:
			return 0;
			break;		
	}
	return 1;
}

unsigned char MovementController::legOneFKCalculation(float q1, float q2, float q3, float& px, float& py, float& pz){
	//TODO: Implement rotation matrix when robot design is final
	
	return 0;
}

unsigned char MovementController::legTwoFKCalculation(float q1, float q2, float q3, float& px, float& py, float& pz){
		//TODO: Implement rotation matrix when robot design is final
		return 0;

}

unsigned char MovementController::legThreeFKCalculation(float q1, float q2, float q3, float& px, float& py, float& pz){
		//TODO: Implement rotation matrix when robot design is final
		return 0;
}

unsigned char MovementController::legFourFKCalculation(float q1, float q2, float q3, float& px, float& py, float& pz){
		//TODO: Implement rotation matrix when robot design is final
		return 0;
}

unsigned char MovementController::legFiveFKCalculation(float q1, float q2, float q3, float& px, float& py, float& pz){
		//TODO: Implement rotation matrix when robot design is final
		return 0;
}

unsigned char MovementController::legSixFKCalculation(float q1, float q2, float q3, float& px, float& py, float& pz){
		//TODO: Implement rotation matrix when robot design is final
		return 0;
}

unsigned char MovementController::getAngleWithIK_tanFormula(float px, float py, float pz, float& q1, float& q2, float& q3){
	float tmp; //used for intermediate steps
	float r;
	float m;
	float s;
	float mquad;
	float theta;
	float alpha;
	float squad;
	float beta;
	float omega;
	float phi;
	
	r=sqrt(pow(px,2)+pow(py,2));
	m=sqrt(pow(px,2)+pow(py,2)+pow(pz,2));
	mquad=pow(m,2);
	theta=atan(fabs(pz)/r);
	beta=(M_PI/2)-theta-c_gamma;
	squad=c_kquad+mquad-2*c_k*m*cos(beta); 
	s=sqrt(squad);
	tmp=((mquad-pow(c_k,2)-squad)*(-1))/(2*c_k*s);
	phi=acos(tmp);
	omega=M_PI-c_delta-phi;
	
	tmp=(squad-l12quad-l23quad)/(2*l12*l23);
	q3=-acos(tmp);
	
	tmp=((l23quad-l12quad-squad)*(-1))/(2*l12*s);
	q2=acos(tmp)-omega;
	
	q1=atan2(py,px);
	
	//Conversion from rad to deg.
	q1=q1/M_PI*180;
	q2=q2/M_PI*180;
	q3=q3/M_PI*180;
	//Conversion from mathematical model to real physical model. Angles have an offset that are compensated with following calculations.
	q1 = 60.0+(q1); //offset = 60: 90 in kinematic -> 150 in Servo
	q2 = 150.0-(q2);//offset = 150: 0 in kinematic -> 150 in Servo
	q3 = 150.0+(q3)+36.3;// 36.3 constant angle. //offset = 150: 0 in kinematic -> 150 + 36.3 in Servo (because of construction)
	return 1;
}



unsigned char MovementController::interpolationAngleEndposition(float qend, float qhome, float (&interpolatedAngleMovement)[10], float& movementSpeed){

	float se;
	float vm;
	float tmp;
	float te;
	float tv;
	float tb;
	float t;
	float timeStep;
	float deltaq;
	signed short signOfDirection;
	
	deltaq=qend-qhome;
	if(deltaq<0){
		signOfDirection=-1;
		}else{
		signOfDirection=1;
	}
	se=fabs(deltaq);
	
	tmp=sqrt(se*bmax);
	if(vmax>tmp){
		vm=tmp;
	}else{
		vm=vmax;		
	}
	tb=vm/bmax;
	te=(se/vm)+tb;
	tv=te-tb;
	
	
	timeStep=te/9;
	
	for(unsigned char i=0;i<10;i++){
		t=timeStep*i;
		if(t<=tb){
			interpolatedAngleMovement[i]=(0.5*bmax*pow(t,2))*signOfDirection;
		}else if(t<=tv){
			interpolatedAngleMovement[i]=((vm*t)-(0.5*(pow(vm,2)/bmax)))*signOfDirection;
		}else{
			interpolatedAngleMovement[i]=((vm*tv)-((bmax/2)*pow((te-timeStep*i),2)))*signOfDirection;	
		}
	}	
	movementSpeed=vm;
}

unsigned char MovementController::interpolationAngleForSyncLinMovement(float deltaQ, float tb, float tv, float *interpolatedAngleMovement, float *interpolatedVelocity, int size){
	float t;
	float deltaq;
	float te;
	float se;
	float vm;
	float bm;
	signed short signOfDirection;
	
	te=tv+tb;
		
	if(deltaQ<0){
		signOfDirection=-1;
	}else{
		signOfDirection=1;
	}
	se=fabs(deltaQ);
	vm=se/tv;
	bm=vm/tb;
	
	for(unsigned char i=0;i<size;i++){
		t=T_IPO*i;
		if(t >= 0 && t < tb){
			interpolatedAngleMovement[i]=(0.5*bm*pow(t,2))*signOfDirection;
			interpolatedVelocity[i]=bm*t;
		}else if(t >=tb && t < tv){
			interpolatedAngleMovement[i]=((vm*t)-(0.5*(pow(vm,2)/bm)))*signOfDirection;
			interpolatedVelocity[i]=vm;
		}else{
			interpolatedAngleMovement[i]=((vm*tv)-((bm/2)*pow((te-t),2)))*signOfDirection;
			interpolatedVelocity[i]=bm*(te-t);
		}
	}
}

unsigned char MovementController::moveLegOneWithInterpolatedPosition(float q1old,float q2old,float q3old, float q1, float q2, float q3){

	float tmp;
	float se;
	float deltaQ1, deltaQ2, deltaQ3;
	signed short signOfDirection;
	float vm;
	float tb,tv,te;
	int size;
	
	deltaQ1=q1-q1old;
	deltaQ2=q2-q2old;
	deltaQ3=q3-q3old;
	
	
	se = fmax(abs(deltaQ1), abs(deltaQ2));
	se = fmax(se, abs(deltaQ3));

	tmp=sqrt(se*bmax);
	if(vmax > tmp){
		vm=tmp;
		tb=int(vm/(bmax*T_IPO))*T_IPO;
		tv=int(se/(vm*T_IPO))*T_IPO;
	}else{
		vm=vmax;
		tb=round(vm/(bmax*T_IPO))*T_IPO;
		tv=round(se/(vm*T_IPO))*T_IPO;
	}

	
	te=tb+tv;
	size=te/T_IPO+1;
	
	float* posVecQ1 = new float[size];
	float* veloVecQ1= new float[size];
	float* posVecQ2 = new float[size];
	float* veloVecQ2= new float[size];
	float* posVecQ3 = new float[size];
	float* veloVecQ3= new float[size];
	
	interpolationAngleForSyncLinMovement(deltaQ1,tb,tv,posVecQ1,veloVecQ1,size);
	interpolationAngleForSyncLinMovement(deltaQ2,tb,tv,posVecQ2,veloVecQ2,size);
	interpolationAngleForSyncLinMovement(deltaQ3,tb,tv,posVecQ3,veloVecQ3,size);
	
	float finalServoPosQ1, finalServoPosQ2, finalServoPosQ3;
	
	for(int i=0;i<size;i++){
		finalServoPosQ1 = q1old+(posVecQ1[i]);
		m_Leg1.m_bodyServo.setServoAngleAndSpeed(finalServoPosQ1, veloVecQ1[i]);
	
		finalServoPosQ2 = q2old+(posVecQ2[i]);
		m_Leg1.m_middleServo.setServoAngleAndSpeed(finalServoPosQ2,veloVecQ2[i]);
		
		finalServoPosQ3 = q3old+(posVecQ3[i]);
		m_Leg1.m_lowerServo.setServoAngleAndSpeed(finalServoPosQ3,veloVecQ3[i]);
	}			
	delete[] posVecQ1;
	delete[] veloVecQ1;
	delete[] posVecQ2;
	delete[] veloVecQ2;
	delete[] posVecQ3;
	delete[] veloVecQ3;	
}

unsigned char MovementController::getAngleWithIK(float px, float py, float pz, float& q1, float& q2, float& q3){
	float r;
	float s;
	float squad;
	float tmp;
	float omega;
	
	r=sqrt(pow(px,2)+pow(py,2));
	tmp=pow(l01a+pz,2)+pow(r-l01b,2);
	s=sqrt(tmp);
	squad=pow(s,2);
	omega=asin(((l01a+pz)/s));
	q1=atan2(py,px);
	tmp=(l12quad+squad-l23quad)/(2*l12*s);
	q2=acos(tmp)-omega;
	tmp=(squad-l12quad-l23quad)/(2*l12*l23);
	q3=(-1)*acos(tmp);
	
	//Conversion from rad to deg.
	q1=q1/M_PI*180;
	q2=q2/M_PI*180;
	q3=q3/M_PI*180;
	//Conversion from mathematical model to real physical model. Angles have an offset that are compensated with following calculations.
	q1 = 60.0+(q1); //offset = 60: 90 in kinematic -> 150 in Servo
	q2 = 150.0-(q2);//offset = 150: 0 in kinematic -> 150 in Servo
	q3 = 186.3+(q3);// 36.3 constant angle. //offset = 150: 0 in kinematic -> 150 + 36.3 in Servo (because of construction)
	
	if(q1<50 || q1>130){
		return 0;
	}
	if(q2<20 || q2>100){
		return 0;
	}
	if(q3<(-16) || q3>(-80)){
		return 0;
	}
	return 1;

}


unsigned char MovementController::moveLegs(float pxold, float pyold, float pzold, float px, float py, float pz){
	float q1, q2,q3,oldq1,oldq2,oldq3,pkx,pky,pkz;
	int steps=12;
	getLegCoordinatesFromWorldCoordinates('3',pxold,pyold,pzold,px,py,pz,pkx,pky,pkz);
	float linDis = sqrt(pow(pkx-pxold,2)+pow(pky-pyold,2)+pow(pkz-pzold,2));
	float m=linDis/steps;
	
	float posVecQ1[steps+1];
	float posVecQ2[steps+1];
	float posVecQ3[steps+1];

	float posVecPx[steps+1];
	float posVecPy[steps+1];
	float posVecPz[steps+1];
	
	float px_cur,py_cur,pz_cur;
	float px_fac,py_fac,pz_fac;

	px_fac=(pkx-pxold)/linDis;
	py_fac=(pky-pyold)/linDis;
	pz_fac=(pkz-pzold)/linDis;
	
	for(int i = 0; i <= steps; i++){
		px_cur=pxold+i*m*px_fac;
		py_cur=pyold+i*m*py_fac;
		pz_cur=pzold+i*m*pz_fac;
		getAngleWithIK(px_cur,py_cur,pz_cur,q1,q2,q3);
		
		posVecQ1[i] = q1;
		posVecQ2[i] = q2;
		posVecQ3[i] = q3;
		
		posVecPx[i] =px_cur;
		posVecPy[i] =py_cur;
		posVecPz[i] =pz_cur;
	}
	for(int i=0;i<=steps;i++){
		m_Leg1.setBodyServoAngle(posVecQ1[i]);
		// 		Serial.println("Position send to Leg 3 Q1:");
		// 		Serial.print(q1Servo);
		// 		Serial.println();
				
		m_Leg1.setMiddleServoAngle(posVecQ2[i]);
		// 		Serial.println("Position send to Leg 3 Q2:");
		// 		Serial.print(q2Servo);
		// 		Serial.println();
				
		m_Leg1.setLowerServoAngle(posVecQ3[i]);
		// 		Serial.println("Position send to Leg 3 Q3:");
		// 		Serial.print(q3Servo);
		// 		Serial.println();
	}
}

unsigned char MovementController::doOneStep(float px, float py, float pz){
	enum state{st_setHomePosition, st_lift145AndPush236, st_lower145, st_lift236AndPush145, st_lower236};
	state cur_state = st_setHomePosition;
	interpolatedMovement movLeg1;
	interpolatedMovement movLeg2;
	interpolatedMovement movLeg3;
	interpolatedMovement movLeg4;
	interpolatedMovement movLeg5;
	interpolatedMovement movLeg6;

	//Step starts from Home position, so pXold, pYold and pZold are considered 0. For a more generic approach, use the Leg::getCurrentPosition() function.
	
	calculatePath('1',0,0,0,px,py,pz,movLeg1);
	calculatePath('2',0,0,0,px,py,pz,movLeg2);
	calculatePath('3',0,0,0,px,py,pz,movLeg3);
	calculatePath('4',0,0,0,px,py,pz,movLeg4);
	calculatePath('5',0,0,0,px,py,pz,movLeg5);
	calculatePath('6',0,0,0,px,py,pz,movLeg6);
	//TODO: Big state machines goes here
	switch(cur_state){
		case st_setHomePosition:
			moveAllLegsToHomePos();
			cur_state=st_lift145AndPush236;
			break;
		case st_lift145AndPush236:
			break; 

	}

}

unsigned char MovementController::calculatePath(unsigned char legNumber, float pxold, float pyold, float pzold, float px, float py, float pz, interpolatedMovement &var){
		float q1, q2,q3,pkx,pky,pkz;
		int steps=12;
		getLegCoordinatesFromWorldCoordinates(legNumber,pxold,pyold,pzold,px,py,pz,pkx,pky,pkz);
		float linDis = sqrt(pow(pkx-pxold,2)+pow(pky-pyold,2)+pow(pkz-pzold,2));
		float m=linDis/steps;
		
		float px_cur,py_cur,pz_cur;
		float px_fac,py_fac,pz_fac;

		px_fac=(pkx-pxold)/linDis;
		py_fac=(pky-pyold)/linDis;
		pz_fac=(pkz-pzold)/linDis;
		
		for(int i = 0; i < steps; i++){
			px_cur=pxold+i*m*px_fac;
			py_cur=pyold+i*m*py_fac;
			pz_cur=pzold+i*m*pz_fac;
			getAngleWithIK(px_cur,py_cur,pz_cur,q1,q2,q3);
			
			var.AngleMovementQ1[i]= q1;
			var.AngleMovementQ2[i]= q2;
			var.AngleMovementQ3[i]= q3;
			
			var.PositionMovementPx[i]=px_cur;
			var.PositionMovementPy[i]=py_cur;
			var.PositionMovementPz[i]=pz_cur;
		}

}

unsigned char MovementController::moveAllLegsToHomePos(){
	m_Leg1.move2HomePosition();
	m_Leg2.move2HomePosition();
	m_Leg3.move2HomePosition();
	m_Leg4.move2HomePosition();
	m_Leg5.move2HomePosition();
	m_Leg6.move2HomePosition();
	//TODO: When is the home position reached? delay() needs to be eliminated!
	delay(2000);
}

// default destructor
MovementController::~MovementController()
{
} //~MovementController
