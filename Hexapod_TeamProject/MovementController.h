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
	/*@function    getAngleWithIK
	* @abstract    calculates the angles q1, q2 and q3 for the desired movement in px, py and pz
	* @param       px, py, py
	*			   desired coordinates
	* @param       q1, q2, q3 (return-by-reference)
	*			   calculated angles for the desired movement

	* @result      returns an error code

	*/
	unsigned char getAngleWithIK(double px, double py, double pz, double& q1, double& q2, double& q3);
	/*@function
	* @abstract
	* @param
	* @result
	*
	*
	
	*/
	unsigned char interpolationAngleEndposition(double qend, double qhome, double (&interpolatedAngleMovement)[10], double& movementSpeed);
	~MovementController();

}; //MovementController

#endif //__MOVEMENTCONTROLLER_H__
