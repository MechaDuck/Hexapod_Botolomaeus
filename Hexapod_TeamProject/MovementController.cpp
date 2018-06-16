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

#define TimeOutMovement 250

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
#define c_phi		M_PI / 4.0 //TODO: Need to be evaluated
#define c_kquad		pow(c_k,2)
#define c_gamma		atan(l01b/l01a)
#define c_delta		atan(l01a/l01b)
#define c_pyOffset  157.8

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
	
	TimerCounter=0;
	
	pLegs[0]=&m_Leg1;
	pLegs[1]=&m_Leg2;
	pLegs[2]=&m_Leg3;
	pLegs[3]=&m_Leg4;
	pLegs[4]=&m_Leg5;
	pLegs[5]=&m_Leg6;
	
	initRobot();
}



unsigned char MovementController::initRobot(){
	//Enable and initialize UART communication with Leg-Servos
	m_ServoComObject1.begin(BaudRate,DirectionPinForBus1,&ServoSerialForBus1);
	//TODO: Set max and min angles. set max speed.
	for(int i=0;i < 6;i++){
			pLegs[i]->setMaxAngles(230,240,270);
			pLegs[i]->setMinAngles(80,60,30);
	}
	pinMode(12, OUTPUT); //also pin 12 as LED output
	digitalWrite(12, LOW); 

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
	pkX=pwX*cos(c_phi)+pwY*sin(c_phi)+pkXold;
	pkY=pwX*sin(c_phi)-pwY*cos(c_phi)+pkYold;
	pkZ=pkZold-pwZ;
	return 1;
}

unsigned char MovementController::worldPositionToLegTwoPosition(float pkXold, float pkYold, float pkZold, float pwX, float pwY, float pwZ, float& pkX, float& pkY, float& pkZ){
	pkX=pwX*cos(c_phi)+pwY*sin(c_phi)+pkXold;
	pkY=pwX*sin(c_phi)+pwY*cos(c_phi)+pkYold;
	pkZ=pkZold-pwZ;
	return 1;
}

unsigned char MovementController::worldPositionToLegThreePosition(float pkXold, float pkYold, float pkZold, float pwX, float pwY, float pwZ, float& pkX, float& pkY, float& pkZ){
	pkX=pwX+pkXold;
	pkY=pkYold-pwY;
	pkZ=pkZold-pwZ;
	return 1;

}

unsigned char MovementController::worldPositionToLegFourPosition(float pkXold, float pkYold, float pkZold, float pwX, float pwY, float pwZ, float& pkX, float& pkY, float& pkZ){
	pkX=(-1)*pwX+pkXold;
	pkY=pkYold+pwY;
	pkZ=pkZold-pwZ;
	return 1;
}

unsigned char MovementController::worldPositionToLegFivePosition(float pkXold, float pkYold, float pkZold, float pwX, float pwY, float pwZ, float& pkX, float& pkY, float& pkZ){
	pkX=(pwX*cos(c_phi))-(pwY*sin(c_phi))+pkXold;
	pkY=(-1.0)*pwX*sin(c_phi)-pwY*cos(c_phi)+pkYold;
	pkZ=(-1.0)*pwZ+pkZold;
	return 1;
	
	//TODO:define sin(phi) etc:....
}

unsigned char MovementController::worldPositionToLegSixPosition(float pkXold, float pkYold, float pkZold, float pwX, float pwY, float pwZ, float& pkX, float& pkY, float& pkZ){
		pkX=(-1.0)*(pwX*cos(c_phi))-(pwY*sin(c_phi))+pkXold;
		pkY=(-1.0)*pwX*sin(c_phi)+pwY*cos(c_phi)+pkYold;
		pkZ=(-1.0)*pwZ+pkZold;
		return 1;
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
	
	py=py+c_pyOffset;
	
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
	return 1;
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
	return 1;
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
	return 1;
}

unsigned char MovementController::getAngleWithIK(float px, float py, float pz, float& q1, float& q2, float& q3){
	float r;
	float s;
	float squad;
	float tmp;
	float omega;
	
	py=py+c_pyOffset;
	
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


unsigned char MovementController::doOneStep(float px, float py, float pz, move_mode mode){
	if(mode==pushWith145){
		doOneStepWith145(px, py, pz);
	}else{
		doOneStepWith236(px, py, pz);
	}
}

unsigned char MovementController::doOneStepWith145(float px, float py, float pz){
	enum state{st_setHomePosition, st_lift236AndPush145, st_lift145AndPush236,st_lower145AndLift236, st_lower236AndLift145, st_moveToFlyingHome, st_lower145,st_lower236, st_finished};
	state cur_state=st_setHomePosition;
	
	MotionSequence movLeg1neg(interpolation_size,SingleSpeed,DisablePositionTracking);
	MotionSequence movLeg4neg(interpolation_size,SingleSpeed,DisablePositionTracking);
	MotionSequence movLeg5neg(interpolation_size,SingleSpeed,DisablePositionTracking);
	
	calculateLinearMotion('1',0,0,0,px,py,pz,movLeg1neg,dir_negative);
	calculateLinearMotion('4',0,0,0,px,py,pz,movLeg4neg,dir_negative);
	calculateLinearMotion('5',0,0,0,px,py,pz,movLeg5neg,dir_negative);
	
	while(cur_state != st_finished){
		switch(cur_state){
			case st_setHomePosition:
				moveAllLegsToHomePos();
			cur_state=st_lift236AndPush145;
			break;
			case st_lift236AndPush145:
				liftLegs(false,true,true,false,false,true);
				moveLegs145_SimultanouslyInterpolated(movLeg1neg,movLeg4neg,movLeg5neg);
			cur_state=st_lower236AndLift145;
			break;
			case st_lower236AndLift145:
				lowerLegs(false,true,true,false,false,true);
				liftLegs(true,false,false,true,true,false);
				cur_state=st_moveToFlyingHome;
			break;
			case st_moveToFlyingHome:
				moveBodyServosToHome(true,false,false,true,true,false);
				cur_state=st_lower145;
			break;
			case st_lower145:
				lowerLegs(true,false,false,true,true,false);
				cur_state=st_finished;
			break;
			default:
				return 0;
			break;
		}
	}
	return 1;
}

unsigned char MovementController::doOneStepWith236(float px, float py, float pz){
	enum state{st_setHomePosition, st_lift236AndPush145, st_lift145AndPush236,st_lower145AndLift236, st_lower236AndLift145, st_moveToFlyingHome, st_lower145,st_lower236, st_finished};
	state cur_state=st_setHomePosition;
	MotionSequence movLeg2neg(interpolation_size,SingleSpeed,DisablePositionTracking);
	MotionSequence movLeg3neg(interpolation_size,SingleSpeed,DisablePositionTracking);
	MotionSequence movLeg6neg(interpolation_size,SingleSpeed,DisablePositionTracking);
	
	calculateLinearMotion('2',0,0,0,px,py,pz,movLeg2neg,dir_negative);
	calculateLinearMotion('3',0,0,0,px,py,pz,movLeg3neg,dir_negative);
	calculateLinearMotion('6',0,0,0,px,py,pz,movLeg6neg,dir_negative);
	
	while(cur_state != st_finished){
		switch(cur_state){
			case st_setHomePosition:
				moveAllLegsToHomePos();
				cur_state=st_lift145AndPush236;
			break;
			case st_lift145AndPush236:
				liftLegs(true,false,false,true,true,false);
				moveLegs236_SimultanouslyInterpolated(movLeg2neg,movLeg3neg,movLeg6neg);
				cur_state=st_lower145AndLift236;
			break;
			case st_lower145AndLift236:
				lowerLegs(true,false,false,true,true,false);
				liftLegs(false,true,true,false,false,true);
				cur_state=st_moveToFlyingHome;
			break;
			case st_moveToFlyingHome:
				moveBodyServosToHome(false,true,true,false,false,true);
				cur_state=st_lower236;
			break;
			case st_lower236:
				lowerLegs(false,true,true,false,false,true);
				cur_state=st_finished;
			break;
			default:
				return 0;
			break;
		}
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
		m_Leg1.setMiddleServoAngle(posVecQ2[i]);				
		m_Leg1.setLowerServoAngle(posVecQ3[i]);
	}
	return 1;
}

unsigned char MovementController::calculateLinearMotion(unsigned char legNumber, float pxold, float pyold, float pzold, float px, float py, float pz, MotionSequence &var,move_direction dir){
		float q1, q2,q3,pkx,pky,pkz;
		int steps=var.getSize()-1;
		getLegCoordinatesFromWorldCoordinates(legNumber,pxold,pyold,pzold,dir*px,dir*py,dir*pz,pkx,pky,pkz);
		float linDis = sqrt(pow(pkx-pxold,2)+pow(pky-pyold,2)+pow(pkz-pzold,2));
		float m=linDis/steps;
		
		float px_cur,py_cur,pz_cur;
		float px_fac,py_fac,pz_fac;

		px_fac=(pkx-pxold)/linDis;
		py_fac=(pky-pyold)/linDis;
		pz_fac=(pkz-pzold)/linDis;
		
		for(int i = 0; i < var.getSize(); i++){
			px_cur=pxold+i*m*px_fac;
			py_cur=pyold+i*m*py_fac;
			pz_cur=pzold+i*m*pz_fac;
			getAngleWithIK(px_cur,py_cur,pz_cur,q1,q2,q3);
			
			var.setAngleSequenceAt(i,q1,q2,q3);
			var.setMotionSequenceAt(i,px_cur,py_cur,pz_cur);
		}
		return 1;
}

unsigned char MovementController::calculateLinearMotionInverse(unsigned char legNumber, float pxold, float pyold, float pzold, float px, float py, float pz, MotionSequence &var){
		float q1, q2,q3,pkx,pky,pkz;
		int steps=var.getSize()-1;
		getLegCoordinatesFromWorldCoordinates(legNumber,pxold,pyold,pzold,(-1.0)*px,(-1.0)*py,(-1.0)*pz,pkx,pky,pkz);
		float linDis = sqrt(pow((pkx)-pxold,2)+pow((pky)-pyold,2)+pow((pkz)-pzold,2));
		float m=linDis/steps;
		
		float px_cur,py_cur,pz_cur;
		float px_fac,py_fac,pz_fac;

		px_fac=(pkx-pxold)/linDis;
		py_fac=(pky-pyold)/linDis;
		pz_fac=(pkz-pzold)/linDis;
		
		for(int i = 0; i < var.getSize(); i++){
			px_cur=pxold+i*m*px_fac;
			py_cur=pyold+i*m*py_fac;
			pz_cur=pzold+i*m*pz_fac;
			getAngleWithIK(px_cur,py_cur,pz_cur,q1,q2,q3);
			
			var.setAngleSequenceAt(i,q1,q2,q3);
			var.setMotionSequenceAt(i,px_cur,py_cur,pz_cur);
		}
		return 1;
}

unsigned char MovementController::calculateInverseLinearMotionWithRaisingLeg(unsigned char legNumber, float pxold, float pyold, float pzold, float px, float py, float pz, MotionSequence &var, float raiseDis){
		float q1, q2,q3,pkx,pky,pkz;
		int steps=var.getSize()-1;
		getLegCoordinatesFromWorldCoordinates(legNumber,pxold,pyold,pzold,(-1)*px,(-1)*py,(-1)*pz,pkx,pky,pkz);
		float linDis = sqrt(pow(pkx-pxold,2)+pow(pky-pyold,2)+pow((pkz-raiseDis)-pzold,2));
		float m=linDis/steps;
			
		float px_cur,py_cur,pz_cur;
		float px_fac,py_fac,pz_fac;

		px_fac=(pkx-pxold)/linDis;
		py_fac=(pky-pyold)/linDis;
		pz_fac=((pkz-raiseDis)-pzold)/linDis;

		for(int i = 0; i < var.getSize(); i++){
			px_cur=pxold+i*m*px_fac;
			py_cur=pyold+i*m*py_fac;
			pz_cur=pzold+i*m*pz_fac;
			getAngleWithIK(px_cur,py_cur,pz_cur,q1,q2,q3);
				
			var.setAngleSequenceAt(i,q1,q2,q3);
			var.setMotionSequenceAt(i,px_cur,py_cur,pz_cur);
		}
		return 1;
}

unsigned char MovementController::calculatePtpMotion(unsigned char legNumber, float pxold, float pyold, float pzold, float px, float py, float pz, PtPMotion &var){
	float q1, q2,q3,pkx,pky,pkz;
	getLegCoordinatesFromWorldCoordinates(legNumber,pxold,pyold,pzold,px,py,pz,pkx,pky,pkz);
	getAngleWithIK(pxold,pyold,pzold,q1,q2,q3);
	
	var.setAngleSequenceAt(0,q1,q2,q3);
	var.setMotionSequenceAt(0,pxold,pyold,pzold);
	
	getAngleWithIK(pkx,pky,pkz,q1,q2,q3);
	var.setAngleSequenceAt(1,q1,q2,q3);
	var.setMotionSequenceAt(1,pkx,pky,pkz);
	
	return 1;
}

unsigned char MovementController::moveAllLegsToHomePos(){
	m_Leg1.move2HomePosition();
	m_Leg2.move2HomePosition();
	m_Leg3.move2HomePosition();
	m_Leg4.move2HomePosition();
	m_Leg5.move2HomePosition();
	m_Leg6.move2HomePosition();
	//Busy wait
	TimerCounter = 0;
	while((m_Leg1.getMovingStatus() || m_Leg2.getMovingStatus() || m_Leg3.getMovingStatus() || m_Leg4.getMovingStatus() || m_Leg5.getMovingStatus() || m_Leg6.getMovingStatus()) && TimerCounter < TimeOutMovement ){
		TimerCounter++;
		delay(10);
	}
	return 1;
}

unsigned char MovementController::moveAllLegsToHomePosWithLiftingLegs(){
	enum state{st_finished, st_lift145, st_moveHome145, st_lower145, st_lift236, st_moveHome236, st_lower236};
	state cur_state=st_lift145;	
	while(cur_state != st_finished){
		switch(cur_state){
			case st_lift145:
				liftLegs(true,false,false,true,true,false);
				cur_state=st_moveHome145;
			break;
			case st_moveHome145:
				moveBodyServosToHome(true,false,false,true,true,false);
				cur_state=st_lower145;
			break;
			case st_lower145:
				lowerLegs(true,false,false,true,true,false);
				cur_state=st_lift236;
			break;
			case st_lift236:
				liftLegs(false,true,true,false,false,true);
				cur_state=st_moveHome236;
			break;
			case st_moveHome236:
				moveBodyServosToHome(false,true,true,false,false,true);
				cur_state=st_lower236;
			break;
			case st_lower236:
				lowerLegs(false,true,true,false,false,true);
				cur_state=st_finished;
			break;
			default:
				return 0;
			break;
		}
	}
	return 1;
}

unsigned char MovementController::moveLegsSimultanouslyInterpolated(struct MotionSequence (&datalegs)[6]){
	if((datalegs[0].getSize() != datalegs[1].getSize()) && (datalegs[0].getSize() != datalegs[2].getSize()) && (datalegs[0].getSize() != datalegs[3].getSize())
			&& (datalegs[0].getSize() != datalegs[4].getSize()) && (datalegs[0].getSize() != datalegs[5].getSize()) && datalegs[0].getSize() != datalegs[6].getSize()){
		return 0;
	}
	if(pLegs == 0){
		return 0;
	}
	for(int i =0 ; i< datalegs[0].getSize();i++){
		for(int leg=0; leg <6; leg++){
			pLegs[leg]->registerDesiredPosition(datalegs[leg].getQ1AngleSequenceAt(i),datalegs[leg].getQ2AngleSequenceAt(i),datalegs[leg].getQ3AngleSequenceAt(i));
			pLegs[leg]->moveLegToRegisteredPosition();
		}
		
		//Busy wait
		TimerCounter = 0;
		while((m_Leg1.getMovingStatus() || m_Leg2.getMovingStatus() || m_Leg3.getMovingStatus() || m_Leg4.getMovingStatus() || m_Leg5.getMovingStatus() || m_Leg6.getMovingStatus()) && TimerCounter < TimeOutMovement ){
			TimerCounter++;
			delay(10);
		}
	}
	return 1;
}

unsigned char MovementController::moveLegsSimultanouslyInterpolatedWithSpeed(struct MotionSequence (&datalegs)[6], float speed){
	if((datalegs[0].getSize() != datalegs[1].getSize()) && (datalegs[0].getSize() != datalegs[2].getSize()) && (datalegs[0].getSize() != datalegs[3].getSize()) 
			&& (datalegs[0].getSize() != datalegs[4].getSize()) && (datalegs[0].getSize() != datalegs[5].getSize()) && datalegs[0].getSize() != datalegs[6].getSize()){
		return 0;
	}
	if(pLegs == 0){
		return 0;
	}
	for(int i =0 ; i< datalegs[0].getSize();i++){
		for(int leg=0; leg <6; leg++){
			pLegs[leg]->registerDesiredPositionAndSpeed(datalegs[leg].getQ1AngleSequenceAt(i),datalegs[leg].getQ2AngleSequenceAt(i),datalegs[leg].getQ3AngleSequenceAt(i),speed);
			pLegs[leg]->moveLegToRegisteredPosition();
		}
		
		//Busy wait
		TimerCounter = 0;
		while((m_Leg1.getMovingStatus() || m_Leg2.getMovingStatus() || m_Leg3.getMovingStatus() || m_Leg4.getMovingStatus() || m_Leg5.getMovingStatus() || m_Leg6.getMovingStatus()) && TimerCounter < TimeOutMovement ){
			TimerCounter++;
			delay(10);
		}
	}
	return 1;
}

unsigned char MovementController::moveLegsSimultanouslyPtp(struct PtPMotion dataLeg1, struct PtPMotion dataLeg2, struct PtPMotion dataLeg3, struct PtPMotion dataLeg4, struct PtPMotion dataLeg5,struct PtPMotion dataLeg6){
		m_Leg1.registerDesiredPosition(dataLeg1.getQ1AngleSequenceAt(1), dataLeg1.getQ2AngleSequenceAt(1),dataLeg1.getQ3AngleSequenceAt(1));
		m_Leg2.registerDesiredPosition(dataLeg2.getQ1AngleSequenceAt(1), dataLeg2.getQ2AngleSequenceAt(1),dataLeg2.getQ3AngleSequenceAt(1));
		m_Leg3.registerDesiredPosition(dataLeg3.getQ1AngleSequenceAt(1), dataLeg3.getQ2AngleSequenceAt(1),dataLeg3.getQ3AngleSequenceAt(1));
		m_Leg4.registerDesiredPosition(dataLeg4.getQ1AngleSequenceAt(1), dataLeg4.getQ2AngleSequenceAt(1),dataLeg4.getQ3AngleSequenceAt(1));
		m_Leg5.registerDesiredPosition(dataLeg5.getQ1AngleSequenceAt(1), dataLeg5.getQ2AngleSequenceAt(1),dataLeg5.getQ3AngleSequenceAt(1));
		m_Leg6.registerDesiredPosition(dataLeg6.getQ1AngleSequenceAt(1), dataLeg6.getQ2AngleSequenceAt(1),dataLeg6.getQ3AngleSequenceAt(1));
	
		m_Leg1.moveLegToRegisteredPosition();
		m_Leg2.moveLegToRegisteredPosition();
		m_Leg3.moveLegToRegisteredPosition();
		m_Leg4.moveLegToRegisteredPosition();
		m_Leg5.moveLegToRegisteredPosition();
		m_Leg6.moveLegToRegisteredPosition();
	//Busy wait
	TimerCounter = 0;
	while((m_Leg1.getMovingStatus() || m_Leg2.getMovingStatus() || m_Leg3.getMovingStatus() || m_Leg4.getMovingStatus() || m_Leg5.getMovingStatus() || m_Leg6.getMovingStatus()) && TimerCounter < TimeOutMovement ){
		TimerCounter++;
		delay(10);
	}
	return 1;
}


unsigned char MovementController::lowerLegs(bool lowerLeg1, bool lowerLeg2, bool lowerLeg3, bool lowerLeg4, bool lowerLeg5, bool lowerLeg6){
	if(lowerLeg1){
		m_Leg1.lowerLeg();
	}
	if(lowerLeg2){
		m_Leg2.lowerLeg();
	}
	if(lowerLeg3){
		m_Leg3.lowerLeg();
	}
	if(lowerLeg4){
		m_Leg4.lowerLeg();
	}
	if(lowerLeg5){
		m_Leg5.lowerLeg();
	}
	if(lowerLeg6){
		m_Leg6.lowerLeg();
	}
	//Busy wait
	TimerCounter = 0;
	while((m_Leg1.getMovingStatus() || m_Leg2.getMovingStatus() || m_Leg3.getMovingStatus() || m_Leg4.getMovingStatus() || m_Leg5.getMovingStatus() || m_Leg6.getMovingStatus()) && TimerCounter < TimeOutMovement ){
		TimerCounter++;
		delay(10);
	}
	return 1;
}

unsigned char MovementController::liftLegs(bool liftLeg1, bool liftLeg2, bool liftLeg3, bool liftLeg4, bool liftLeg5, bool liftLeg6){
	if(liftLeg1){
		m_Leg1.liftLeg();
	}
	if(liftLeg2){
		m_Leg2.liftLeg();
	}
	if(liftLeg3){
		m_Leg3.liftLeg();
	}
	if(liftLeg4){
		m_Leg4.liftLeg();
	}
	if(liftLeg5){
		m_Leg5.liftLeg();
	}
	if(liftLeg6){
		m_Leg6.liftLeg();
	}
	//Busy wait
	TimerCounter = 0;
	while((m_Leg1.getMovingStatus() || m_Leg2.getMovingStatus() || m_Leg3.getMovingStatus() || m_Leg4.getMovingStatus() || m_Leg5.getMovingStatus() || m_Leg6.getMovingStatus()) && TimerCounter < TimeOutMovement ){
		TimerCounter++;
		delay(10);
	}
	return 1;
}

unsigned char MovementController::moveBodyServosToHome(bool XYHomeLeg1, bool XYHomeLeg2, bool XYHomeLeg3, bool XYHomeLeg4, bool XYHomeLeg5, bool XYHomeLeg6){
	if(XYHomeLeg1){
		m_Leg1.moveBodyServoToHomePos();
	}
	if(XYHomeLeg2){
		m_Leg2.moveBodyServoToHomePos();
	}
	if(XYHomeLeg3){
		m_Leg3.moveBodyServoToHomePos();
	}
	if(XYHomeLeg4){
		m_Leg4.moveBodyServoToHomePos();
	}
	if(XYHomeLeg5){
		m_Leg5.moveBodyServoToHomePos();
	}
	if(XYHomeLeg6){
		m_Leg6.moveBodyServoToHomePos();
	}
	//Busy wait
	TimerCounter = 0;
	while((m_Leg1.getMovingStatus() || m_Leg2.getMovingStatus() || m_Leg3.getMovingStatus() || m_Leg4.getMovingStatus() || m_Leg5.getMovingStatus() || m_Leg6.getMovingStatus()) && TimerCounter < TimeOutMovement ){
		TimerCounter++;
		delay(10);
	}
	return 1;
}

unsigned char MovementController::moveLegs145_SimultanouslyInterpolated(struct MotionSequence dataLeg1, struct MotionSequence dataLeg4, struct MotionSequence dataLeg5){
	if((dataLeg1.getSize() != dataLeg4.getSize()) && dataLeg1.getSize() != dataLeg5.getSize()){
		return 0;
	}
	for(int i=0; i< dataLeg1.getSize();i++){
		m_Leg1.registerDesiredPosition(dataLeg1.getQ1AngleSequenceAt(i), dataLeg1.getQ2AngleSequenceAt(i),dataLeg1.getQ3AngleSequenceAt(i));
		m_Leg4.registerDesiredPosition(dataLeg4.getQ1AngleSequenceAt(i), dataLeg4.getQ2AngleSequenceAt(i),dataLeg4.getQ3AngleSequenceAt(i));
		m_Leg5.registerDesiredPosition(dataLeg5.getQ1AngleSequenceAt(i), dataLeg5.getQ2AngleSequenceAt(i),dataLeg5.getQ3AngleSequenceAt(i));
		
		m_Leg1.moveLegToRegisteredPosition();
		m_Leg4.moveLegToRegisteredPosition();
		m_Leg5.moveLegToRegisteredPosition();

		//Busy wait
		TimerCounter = 0;
		while((m_Leg1.getMovingStatus() || m_Leg4.getMovingStatus() || m_Leg5.getMovingStatus()) && TimerCounter < TimeOutMovement ){
			TimerCounter++;
			delay(10);
		}	
	}
	return 1;
}

unsigned char MovementController::moveLegs236_SimultanouslyInterpolated(struct MotionSequence dataLeg2, struct MotionSequence dataLeg3, struct MotionSequence dataLeg6){
	if((dataLeg2.getSize() != dataLeg3.getSize()) && dataLeg2.getSize() != dataLeg6.getSize() ){
		return 0;
	}
	for(int i=0; i< dataLeg2.getSize();i++){
		m_Leg2.registerDesiredPosition(dataLeg2.getQ1AngleSequenceAt(i), dataLeg2.getQ2AngleSequenceAt(i),dataLeg2.getQ3AngleSequenceAt(i));
		m_Leg3.registerDesiredPosition(dataLeg3.getQ1AngleSequenceAt(i), dataLeg3.getQ2AngleSequenceAt(i),dataLeg3.getQ3AngleSequenceAt(i));
		m_Leg6.registerDesiredPosition(dataLeg6.getQ1AngleSequenceAt(i), dataLeg6.getQ2AngleSequenceAt(i),dataLeg6.getQ3AngleSequenceAt(i));
			
		m_Leg2.moveLegToRegisteredPosition();
		m_Leg3.moveLegToRegisteredPosition();
		m_Leg6.moveLegToRegisteredPosition();	
		
		//Busy wait
		TimerCounter = 0;
		while((m_Leg2.getMovingStatus() || m_Leg3.getMovingStatus() || m_Leg6.getMovingStatus()) && TimerCounter < TimeOutMovement ){
			TimerCounter++;
			delay(10);
		}
	}
	return 1;
}

// default destructor
MovementController::~MovementController()
{
} //~MovementController
