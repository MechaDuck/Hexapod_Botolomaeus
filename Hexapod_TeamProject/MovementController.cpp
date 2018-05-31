/* 
* MovementController.cpp
*
* Created: 22.04.2018 21:39:35
* Author: Tomislav Romic
*/


#include "MovementController.h"
#include "AX12A.h"
#include <math.h>


/*--------------------------------------------------------------------------------------------*/
/*Constant values for the calculation of inverse kinematics*/
//Properties of the body (see documentation for definition)

/* vector parts that starts from the origin of the world coordinate system to an origin of a leg coordinate system.
Can be used for every leg, because the robot is symmetrical with respect to the x-axes AND y-axes. Only the sign of those variables varies.
(see documentation for further definition)*/
#define pwkx 116.41 
#define pwky 79.379 
#define pwkz 666 /*TODO: Unclear which parameter goes here!!!*/
#define pwkAngle (30.0/180.0)*M_PI




//Properties of the legs (see documentation for definition)

#define l01a 119.845 /*mm*/
#define l01b 47.75  /*mm*/
#define l12 76.395       /*mm*/
#define l23 204.33 /*mm*/
#define l12quad pow(l12,2)
#define l23quad pow(l23,2)
//Properties of the servos (see documentation for definition)
#define bmax 1000 /*rad/sec^2*/ //TODO: Need to be calculated again!! In degrees/sec^2
#define vmax 684.0 /*deg/sec^2*/ // TODO: Need to be calculated again!! In degress/sec

//Constant values for kinematic calculations
#define k sqrt(pow(l01a,2)+pow(l01b,2))
#define phi M_PI/6.0 //TODO: Need to be evaluated
#define kquad pow(k,2)
#define gamma atan(l01b/l01a)
#define delta atan(l01a/l01b)

//Variables for interpolation calculations
#define T_IPO 0.025f
/*--------------------------------------------------------------------------------------------*/

 MovementController::MovementController():
	m_Leg1(m_ServoBus12, ID_leg1_bodyServo, ID_leg1_middleLegServo, ID_leg1_lowerLegServo),
	m_Leg2(m_ServoBus12, ID_leg2_bodyServo, ID_leg2_middleLegServo, ID_leg2_lowerLegServo),
	m_Leg3(m_ServoBus34, ID_leg3_bodyServo, ID_leg3_middleLegServo, ID_leg3_lowerLegServo),
	m_Leg4(m_ServoBus34, ID_leg4_bodyServo, ID_leg4_middleLegServo, ID_leg4_lowerLegServo),
	m_Leg5(m_ServoBus56, ID_leg5_bodyServo, ID_leg5_middleLegServo, ID_leg5_lowerLegServo),
	m_Leg6(m_ServoBus56, ID_leg6_bodyServo, ID_leg6_middleLegServo, ID_leg6_lowerLegServo){
	
	m_ServoBus12.begin(BaudRate,DirectionPinForBus1,&ServoSerialForBus1);
	m_ServoBus34.begin(BaudRate, DirectionPinForBus2,&ServoSerialForBus2);
	m_ServoBus56.begin(BaudRate, DirectionPinForBus3,&ServoSerialForBus3);
	
}



unsigned char MovementController::move2HomePosition(){
	m_Leg1.move2HomePosition();

}

unsigned char MovementController::calculateWorldMovement2LegMovement(unsigned char legNumber,double pkXold, double pkYold, double pkZold, double pwX, double pwY, double pwZ, double& pkX, double& pkY, double& pkZ){
		switch (legNumber){
			case '0':
				worldPosition2LegOnePosition(pkXold,pkYold,pkZold,pwX,pwY,pwZ,pkX,pkY,pkZ);
			break;
			case '1':
				worldPosition2LegOnePosition(pkXold,pkYold,pkZold,pwX,pwY,pwZ,pkX,pkY,pkZ);
			break;
			case '2':
			break;
			case '3':
			    worldPosition2LegThreePosition(pkXold,pkYold,pkZold,pwX,pwY,pwZ,pkX,pkY,pkZ);
			break;
			case '4':
			break;
			case '5':
			break;
			case '6':
			break;
			default:
			return 0;
			break;
		}
		return 1;

}

unsigned char MovementController::worldPosition2LegOnePosition(double pkXold, double pkYold, double pkZold, double pwX, double pwY, double pwZ, double& pkX, double& pkY, double& pkZ){
	//Serial.println("func. worldPosition2LegOnePosition input: pkx, pky, pkz, pkxOld, pkyOld, pkzOld, pwx, pwy, pwz");
	//Serial.println(pkX);
	//Serial.println(pkY);
	//Serial.println(pkZ);
	//
	//Serial.println(pkXold);
	//Serial.println(pkYold);
	//Serial.println(pkZold);
	//
	//Serial.println(pwX);
	//Serial.println(pwY);
	//Serial.println(pwZ);
	
	//float angle_world_leg = 30.0/180.0;
	
	//Serial.println(phi);
	
	pkX=(pwX*cos(phi))+(pwY*sin(phi))+pkXold;
	pkY=pwX*sin(phi)-pwY*cos(phi)+pkYold;
	pkZ=(-1)*pwZ+pkZold;
	
	
	//Serial.println("func. worldPosition2LegOnePosition output: pkx, pky, pkz");
	//Serial.println(pkX);
	//Serial.println(pkY);
	//Serial.println(pkZ);
	
	//TODO:define sin(phi) etc:....

}

unsigned char MovementController::worldPosition2LegThreePosition(double pkXold, double pkYold, double pkZold, double pwX, double pwY, double pwZ, double& pkX, double& pkY, double& pkZ){

	pkX=pwX+pkXold;
	pkY=pkYold-pwY;
	pkZ=pkZold-pwZ;
	
}

unsigned char MovementController::world2LegCoordinateSystemWithFK(unsigned char legNumber, double q1, double q2, double q3, double& px, double& py, double& pz){
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

unsigned char MovementController::legOneFKCalculation(double q1, double q2, double q3, double& px, double& py, double& pz){
	//TODO: Implement rotation matrix when robot design is final
	
	return 0;


}

unsigned char MovementController::legTwoFKCalculation(double q1, double q2, double q3, double& px, double& py, double& pz){
		//TODO: Implement rotation matrix when robot design is final
		return 0;

}

unsigned char MovementController::legThreeFKCalculation(double q1, double q2, double q3, double& px, double& py, double& pz){
		//TODO: Implement rotation matrix when robot design is final
		return 0;
}

unsigned char MovementController::legFourFKCalculation(double q1, double q2, double q3, double& px, double& py, double& pz){
		//TODO: Implement rotation matrix when robot design is final
		return 0;
}

unsigned char MovementController::legFiveFKCalculation(double q1, double q2, double q3, double& px, double& py, double& pz){
		//TODO: Implement rotation matrix when robot design is final
		return 0;
}

unsigned char MovementController::legSixFKCalculation(double q1, double q2, double q3, double& px, double& py, double& pz){
		//TODO: Implement rotation matrix when robot design is final
		return 0;
}

unsigned char MovementController::getAngleWithIK_tanFormula(double px, double py, double pz, double& q1, double& q2, double& q3)
{
// 	double tmp; //used for intermediate steps
// 	double r;
// 	double m;
// 	double s;
// 	double mquad;
// 	double theta;
// 	double alpha;
// 	double squad;
// 	double beta;
// 	double omega;
// 	double phi;
// 	
// 	
// 	
// 	r=sqrt(pow(px,2)+pow(py,2));
// 	m=sqrt(pow(px,2)+pow(py,2)+pow(pz,2));
// 	mquad=pow(m,2);
// 	theta=atan(fabs(pz)/r);
// 	beta=(M_PI/2)-theta-gamma;
// 	squad=kquad+mquad-2*k*m*cos(beta); 
// 	s=sqrt(squad);
// 	tmp=((mquad-pow(k,2)-squad)*(-1))/(2*k*s);
// 	phi=acos(tmp);
// 	omega=pi-delta-phi;
// 	
// 	tmp=(squad-l12quad-l23quad)/(2*l12*l23);
// 	q3=-acos(tmp);
// 	
// 	tmp=((l23quad-l12quad-squad)*(-1))/(2*l12*s);
// 	q2=acos(tmp)-omega;
// 	
// 	q1=atan2(py,px);
	
	return 1;

}



unsigned char MovementController::interpolationAngleEndposition(double qend, double qhome, double (&interpolatedAngleMovement)[10], double& movementSpeed){

	double se;
	double vm;
	double tmp;
	double te;
	double tv;
	double tb;
	double t;
	double timeStep;
	double deltaq;
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
	//TODO: Replace qstart qend parameters with deltaq
	double t;
	double deltaq;
	double te;
	double se;
	double vm;
	double bm;
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
		if(t<=tb){
			interpolatedAngleMovement[i]=(0.5*bm*pow(t,2))*signOfDirection;
			interpolatedVelocity[i]=bm*t;
		}else if(t<tv){
			interpolatedAngleMovement[i]=((vm*t)-(0.5*(pow(vm,2)/bm)))*signOfDirection;
			interpolatedVelocity[i]=vm;
		}else{
			interpolatedAngleMovement[i]=((vm*tv)-((bm/2)*pow((te-t),2)))*signOfDirection;
			interpolatedVelocity[i]=bm*(te-t);
		}
	}
}

unsigned char MovementController::moveLegOneWithInterpolatedPosition(float q1old,float q2old,float q3old, float q1, float q2, float q3){
	//TODO:assuming old position is the home position
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
	se=  fmax(se, deltaQ3);

	tmp=sqrt(se*bmax);
	if(vmax>tmp){
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
		m_Leg1.m_bodyServo.setServoAngleAndSpeed(finalServoPosQ1, veloVecQ1[i]*0.166666/0.111);
		Serial.println("Position send to Leg 1 Q1:");
		Serial.print(finalServoPosQ1);
		Serial.print("with velocity: ");
		Serial.println(veloVecQ1[i]);
		Serial.println();
		
		
		finalServoPosQ2 = q2old+(posVecQ2[i]);
		m_Leg1.m_middleLegServo.setServoAngleAndSpeed(finalServoPosQ2,veloVecQ2[i]*0.166666/0.111);
// 		Serial.println("Position send to Leg 1 Q2:");
// 		Serial.print(finalServoPosQ2);
// 		Serial.print("with velocity: ");
// 		Serial.println(veloVecQ2[i]);
// 		Serial.println();
		
		finalServoPosQ3 = q3old+(posVecQ3[i]);
		m_Leg1.m_lowerLegServo.setServoAngleAndSpeed(finalServoPosQ3,veloVecQ3[i]*0.166666/0.111);
// 		Serial.println("Position send to Leg 1 Q3:");
// 		Serial.print(finalServoPosQ3);
// 		Serial.print("with velocity: ");
// 		Serial.println(veloVecQ3[i]);
// 		Serial.println("________________________________________________");
	}			
	delete[] posVecQ1;
	delete[] veloVecQ1;
	delete[] posVecQ2;
	delete[] veloVecQ2;
	delete[] posVecQ3;
	delete[] veloVecQ3;
	
}

unsigned char MovementController::getAngleWithIK(double px, double py, double pz, double& q1, double& q2, double& q3){
	double r;
	double s;
	double squad;
	double tmp;
	double omega;
	
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

unsigned char MovementController::interpolationAngleForSyncLinMovement2(float deltaQ, float vm, float bm, float *interpolatedAngleMovement, float *interpolatedVelocity, int size)
{

}

// default destructor
MovementController::~MovementController()
{
} //~MovementController
