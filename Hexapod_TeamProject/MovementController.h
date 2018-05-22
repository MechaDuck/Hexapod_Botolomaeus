/* 
* MovementController.h
*
* Created: 22.04.2018 21:39:35
* Author: Tomislav Romic
*/


#ifndef __MOVEMENTCONTROLLER_H__
#define __MOVEMENTCONTROLLER_H__



class MovementController
{
//variables
public:
protected:
private:

//functions
public:
	MovementController();
	
	unsigned char world2LegCoordinateSystemWithFK(unsigned char legNumber, double q1, double q2, double q3, double& px, double& py, double& pz);
	unsigned char legOneFKCalculation(double q1, double q2, double q3, double& px, double& py, double& pz);
	unsigned char legTwoFKCalculation(double q1, double q2, double q3, double& px, double& py, double& pz);
	unsigned char legThreeFKCalculation(double q1, double q2, double q3, double& px, double& py, double& pz);
	unsigned char legFourFKCalculation(double q1, double q2, double q3, double& px, double& py, double& pz);
	unsigned char legFiveFKCalculation(double q1, double q2, double q3, double& px, double& py, double& pz);
	unsigned char legSixFKCalculation(double q1, double q2, double q3, double& px, double& py, double& pz);
	/*@function    getAngleWithIK
	* @abstract    calculates the angles q1 [rad/sec], q2 [rad/sec] and q3 [rad/sec] for the desired movement in px [mm], py [mm] and pz [mm]
	* @param       px, py, py
	*			   desired coordinates
	* @param       q1, q2, q3 (return-by-reference)
	*			   calculated angles for the desired movement

	* @result      returns an error code

	*/
	unsigned char getAngleWithIK_tanFormula(double px, double py, double pz, double& q1, double& q2, double& q3);
	/*@function
	* @abstract
	* @param
	* @result
	*
	*
	
	*/
	unsigned char interpolationAngleEndposition(double qend, double qhome, double (&interpolatedAngleMovement)[10], double& movementSpeed);
	
	unsigned char getAngleWithIK(double px, double py, double pz, double& q1, double& q2, double& q3);
	
	~MovementController();

}; //MovementController

#endif //__MOVEMENTCONTROLLER_H__
