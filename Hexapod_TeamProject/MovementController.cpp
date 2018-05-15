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
//Properties of the legs (see documentation for definition)

#define l01a 107.248
#define l01b 47.745
#define l12 77
#define l23 191.563
#define l12quad pow(l12,2)
#define l23quad pow(l23,2)
//Properties of the legs (see documentation for definition)
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


unsigned char MovementController::getAngleWithIK(double px, double py, double pz, double& q1, double& q2, double& q3)
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
	theta=atan(fabs(pz/r));
	beta=(pi/2)-theta-gamma;
	squad=kquad+mquad; /*TODO: Ask Group. Maybe use alternative 2*k*m*cos(beta)*/
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

unsigned char MovementController::interpolationAngleEndposition(double qend, double qhome, double (&interpolatedAngleMovement)[10], double& movementSpeed)
{
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

// default destructor
MovementController::~MovementController()
{
} //~MovementController
