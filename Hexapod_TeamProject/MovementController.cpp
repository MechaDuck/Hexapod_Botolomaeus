/* 
* MovementController.cpp
*
* Created: 22.04.2018 21:39:35
* Author: Tomislav Romic
*/


#include "MovementController.h"
#include <math.h>


/*--------------------------------------------------------------------------------------------*/
/*Constant values for the calculation of inverse kinematics*/
//Properties of the body (see documentation for definition)

/* vector parts that starts from the origin of the world coordinate system to an origin of a leg coordinate system.
Can be used for every leg, because the robot is symmetrical with respect to the x-axes AND y-axes. Only the sign of those variables varies.
(see documentation for further definition)*/
#define pwkx 116.41 
#define pwky 79.379 
#define pwkz 666 /*TODO: Unclear wich parameter goes here!!!*/
#define pwkAngle (30/180)*M_PI




//Properties of the legs (see documentation for definition)

#define l01a 107.248 /*mm*/
#define l01b 47.745  /*mm*/
#define l12 77       /*mm*/
#define l23 191.563 /*mm*/
#define l12quad pow(l12,2)
#define l23quad pow(l23,2)
//Properties of the servos (see documentation for definition)
#define bmax 143 /*rad/sec^2*/
#define vmax 5  /*rad/sec^2*/

//Constant values for kinematic calculations
#define k sqrt(pow(l01a,2)+pow(l01b,2))
#define kquad pow(k,2)
#define gamma atan(l01b/l01a)
#define delta atan(l01a/l01b)
#define pi 3.141593

/*--------------------------------------------------------------------------------------------*/

// default constructor
MovementController::MovementController()
{
} //MovementController


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
	double tmp; //used for intermediate steps
	double r;
	double m;
	double s;
	double mquad;
	double theta;
	double alpha;
	double squad;
	double beta;
	double omega;
	double phi;
	
	
	
	r=sqrt(pow(px,2)+pow(py,2));
	m=sqrt(pow(px,2)+pow(py,2)+pow(pz,2));
	mquad=pow(m,2);
	theta=atan(fabs(pz)/r);
	beta=(M_PI/2)-theta-gamma;
	squad=kquad+mquad-2*k*m*cos(beta); 
	s=sqrt(squad);
	tmp=((mquad-pow(k,2)-squad)*(-1))/(2*k*s);
	phi=acos(tmp);
	omega=pi-delta-phi;
	
	tmp=(squad-l12quad-l23quad)/(2*l12*l23);
	q3=-acos(tmp);
	
	tmp=((l23quad-l12quad-squad)*(-1))/(2*l12*s);
	q2=acos(tmp)-omega;
	
	q1=atan2(py,px);
	
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
	omega=asin((l01a/s+pz/s));
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

// default destructor
MovementController::~MovementController()
{
} //~MovementController
