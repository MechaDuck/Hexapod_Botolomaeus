/* 
* MovementController.h
*
* Created: 22.04.2018 21:39:35
* Author: Tomislav Romic
*/

/**
@file MovementController.h
*/

#ifndef __MOVEMENTCONTROLLER_H__
#define __MOVEMENTCONTROLLER_H__

#include "Leg.h"
#include "Arduino.h"
#include "AX12A.h"
#include "MotionSequence.h"
#include "PtPMotion.h"

enum move_direction {dir_positive=1, dir_negative=-1};
enum move_mode {pushWith236=1, pushWith145=2};
enum state{st_setHomePosition, st_lift236AndPush145, st_lift145AndPush236,st_lower145AndLift236, st_lower236AndLift145, st_moveToFlyingHome, st_lower145,st_lower236, st_finished,
st_initStep,st_lift236AndGoHomeAndLower236AndPush145,st_lift145AndGoHomeAndLower145AndPush236};


/**

Rough structure of the hexapod to understand the naming of the six legs. To find the "UP" position please look on the backside of the robot.
<pre>
\verbatim
leg2	    	      leg1
<===\     /"UP"\     /===>
	 \==|--------|==/
		|--------|
leg4	|--------|	leg3
<=======|--------|=======>
		|--------|
		|--------|
leg6 /==|--------|==\leg5
<===/   |--------|	 \===>
\endverbatim
</pre>

*/
/**   
Rough structure of the hexapod to understand the naming of the three servos on one leg. 
\verbatim
         (LeftLeg)                      Hexapod	                         (RightLeg)	
							/////######################\\\\\		
	   _____________	 BodyServo				     BodyServo          _____________
	   LowerLegServo	  /////#########################\\\\\		    LowerLegServo
	 /////      \\\\\    /////							 \\\\\		  /////      \\\\\
	/////		 \\\\\  /////							  \\\\\		 /////		  \\\\\
   /////		MiddleLegServo		     				   MiddleLegServo	       \\\\\
____\/__________________________________ Ground______________________________________\/______															 
\endverbatim
*/


class MovementController
{
//variables
public:
	///@{
	/**
	*@brief Three objects of the class AX12A are created. To enable parallel communication with three serial ports on the Arduino. 
	*/
	AX12A m_ServoComObject1; /**< Communication object */
	AX12A m_ServoComObject2; /**< Communication object */
	AX12A m_ServoComObject3; /**< Communication object */
	///@}	

	///@{
	/**
	*@brief Every leg on the hexapod can be individually controlled by following objects 
	*/
	Leg m_Leg1;
	Leg m_Leg2;
	Leg m_Leg3;
	Leg m_Leg4;
	Leg m_Leg5;
	Leg m_Leg6;
	Leg* pLegs[6];
	
	state robotCur_state;
	///@}
private:
	unsigned char TimerCounter;
//functions
public:
	MovementController();
	/**
	* @function initRobot
	* @brief initializes robot parameters, f.e. max and min angles of the robot servos
	* @result (not implemented yet) returns error code, if problems occur. In current state is always returns 1
	*/
	unsigned char initRobot();
	
	///@{
	/**  
	* @name worldCoordinatesToLegCoordinates                                                        
	* @brief Converts world coordinates to the specific leg coordinate system.
	*/
	/**
	* @param legNumber
	*			The leg to that the coordinate system is transformed
	* @param pkXold, pkYold, pkZold [mm]
	*			Current position of the leg in the leg coordinate system
	* @param pwX, pwY, pwZ [mm]
	*			New desired position. Given in the world coordinate system.
	* @param pkX, pkY, pkZ [mm](return-by-reference)
	*			New calculated position in specific leg coordinate system.
	* @retval Returns an error code, if problems occur. Else it returns 1
	*/
	unsigned char getLegCoordinatesFromWorldCoordinates(unsigned char legNumber,float pkXold, float pkYold, float pkZold, float pwX, float pwY, float pwZ, float& pkX, float& pkY, float& pkZ);
	unsigned char worldPositionToLegOnePosition(float pkXold, float pkYold, float pkZold, float pwX, float pwY, float pwZ, float& pkX, float& pkY, float& pkZ);
	unsigned char worldPositionToLegTwoPosition(float pkXold, float pkYold, float pkZold, float pwX, float pwY, float pwZ, float& pkX, float& pkY, float& pkZ);
	unsigned char worldPositionToLegThreePosition(float pkXold, float pkYold, float pkZold, float pwX, float pwY, float pwZ, float& pkX, float& pkY, float& pkZ);
	unsigned char worldPositionToLegFourPosition(float pkXold, float pkYold, float pkZold, float pwX, float pwY, float pwZ, float& pkX, float& pkY, float& pkZ);
	unsigned char worldPositionToLegFivePosition(float pkXold, float pkYold, float pkZold, float pwX, float pwY, float pwZ, float& pkX, float& pkY, float& pkZ);
	unsigned char worldPositionToLegSixPosition(float pkXold, float pkYold, float pkZold, float pwX, float pwY, float pwZ, float& pkX, float& pkY, float& pkZ);
	///@}
	
	///@{
	/**
	*@name getWorldCoordinatesFromFK 
	*@param legNumber
	*			 The leg to that the coordinate system is transformed
	*@param q1, q2, q3 [deg] 
	*			Angles that are currently set on the servos. 
	*@param px, py, pz [mm] (return-by-reference)
	*			Calculated position where the chosen leg currently is. Coordinates are given in respect to the world coordinate system.
	*/
	unsigned char getWorldCoordinatesFromFK(unsigned char legNumber, float q1, float q2, float q3, float& px, float& py, float& pz);
	unsigned char legOneFKCalculation(float q1, float q2, float q3, float& px, float& py, float& pz);
	unsigned char legTwoFKCalculation(float q1, float q2, float q3, float& px, float& py, float& pz);
	unsigned char legThreeFKCalculation(float q1, float q2, float q3, float& px, float& py, float& pz);
	unsigned char legFourFKCalculation(float q1, float q2, float q3, float& px, float& py, float& pz);
	unsigned char legFiveFKCalculation(float q1, float q2, float q3, float& px, float& py, float& pz);
	unsigned char legSixFKCalculation(float q1, float q2, float q3, float& px, float& py, float& pz);
	///@}

	/**
	* @function    getAngleWithIK_tanFormula
	* @brief	    calculates the angles q1 [rad/sec], q2 [rad/sec] and q3 [rad/sec] for the desired movement in px [mm], py [mm] and pz [mm]
					A error from this formula can be expected, due to the use of the tan-function. For better results, use the 'getAngleWithIK(...)' function.
	* @param       px, py, py [mm]
	*			   desired coordinates
	* @param       q1, q2, q3 [deg](return-by-reference)
	*			   calculated angles for the desired movement. Angle values are adapted to the servo settings
	* @result      returns an error code, if problems occur. Else it returns 1
	*/
	unsigned char getAngleWithIK_tanFormula(float px, float py, float pz, float& q1, float& q2, float& q3);
		
	/**
	* @function    getAngleWithIK
	* @brief	   calculates the angles q1 [rad/sec], q2 [rad/sec] and q3 [rad/sec] for the desired movement in px [mm], py [mm] and pz [mm]
	* @param       px, py, py [mm]
	*			   desired coordinates
	* @param       q1, q2, q3 [deg](return-by-reference)
	*			   calculated angles for the desired movement. Angle values are adapted to the servo settings
	* @result      (not implemented yet) returns error code, if problems occur. In current state is always returns 1
	*/
	unsigned char getAngleWithIK(float px, float py, float pz, float& q1, float& q2, float& q3);
	
	/**
	*@function moveAllLegsToHomePosWithLiftingLegs
	*@brief Moves all legs to the home position. The position chang is done by lifting the legs up, so the robot stays on the same spot while positioning
	*@result returns 0 if an error occurs. In current state not possible! Returns always 1
	*/
	unsigned char moveAllLegsToHomePosWithLiftingLegs();
	/**
	*@function doContinuesSteps
	*@brief Main function for the movement of the hexapod in x,y,z direction without rotation
	*@brief When called it runs the state that is set in the "robotCur_state" member variable of this class. 
	*@param px, py, pz [mm]
	*		position given in world coordinates
	*@result (not implemented yet) returns error code, if problems occur. In current state is always returns 1
	*/
	unsigned char doContinuesSteps(float px, float py, float pz);
	/**
	*@function doContinuesRotation
	*@brief Main function for the rotation of the hexapod with a given angle direction
	*@brief When called it runs the state that is set in the "robotCur_state" member variable of this class. 
	*@param angle [rad]
	*@result (not implemented yet) returns error code, if problems occur. In current state is always returns 1
	*/
	unsigned char doContinuesRotation(float angle);
	unsigned char doContinuesRotationOrStep(float px, float py, float pz,float angle);
	/**
	*@function calculateLinearMotion
	*@brief  number of interpolations is given by the size of the variable "var". If the variable is initialized
			 to keep track of the velocity and position, those variables are also calculated, interpolated and stored.
			 Depending on the value of the angles q1, q2 and q3 in one interpolation step, the velocity is linearized, 
			 so every angle reaches the end position in the same time.
	*@param legNumber ['1', '2','3','4','5','6']
			The leg that needs to be interpolated
	*@param pxold, pyold, pzold [mm]
			Old position of the robot, given in world coordinates
	*@param px, py, pz [mm]
			New desired position, given in world coordinates
	*@param var [return-by-reference]
			Object that stores the calculated motion sequence and if enabled the velocity sequence and position sequence (position given in leg coordinates pkx,pky,pkz). 
	*@param dirX, dirY, dirZ [dir_positive, dir_negative]	
			Changes the sign of the parameters px, py ,pz	
	*@result (not implemented yet) returns error code, if problems occur. In current state is always returns 1
	*/
	unsigned char calculateLinearMotion(unsigned char legNumber, float pxold, float pyold, float pzold, float px, float py, float pz, MotionSequence &var, move_direction dirX, move_direction dirY, move_direction dirZ);	
	
	/**
	*@function calculateLinearMotionWithRaisingLeg
	*@brief  number of interpolations is given by the size of the variable "var". If the variable is initialized
			 to keep track of the velocity and position, those variables are also calculated, interpolated and stored.
			 Depending on the value of the angles q1, q2 and q3 in one interpolation step, the velocity is linearized, 
			 so every angle reaches the end position in the same time. Additionally it lifts and lowers the leg by the given parameter "raiseDis".
			 This is realized with a modified ramp function that is added to the interpolated and linearized end position sequence.
			 At the beginning it immediately raises the leg to the value given by "directRaise" in the z-coordinate. In the middle of the sequence 
			 the leg reaches the z-coordinate value that is given by the raiseDis parameter. At the end of the sequence this offset is removed.
			 
	*@param legNumber ['1', '2','3','4','5','6']
			The leg that needs to be interpolated
	*@param pxold, pyold, pzold [mm]
			Old position of the robot, given in world coordinates
	*@param px, py, pz [mm]
			New desired position, given in world coordinates
	*@param var [return-by-reference]
			Object that stores the calculated motion sequence and if enabled the velocity sequence and position sequence (position given in leg coordinates pkx,pky,pkz). 
	*@param dirX, dirY, dirZ [dir_positive, dir_negative]	
			Changes the sign of the parameters px, py ,pz
	*@param raiseDis [mm]
			Peak distance to which the leg should be raised.
	*@param directRaise [mm]
			Raise at the very beginning of the moving sequence.
	*@result (not implemented yet) returns error code, if problems occur. In current state is always returns 1
	*/
	unsigned char calculateLinearMotionWithRaisingLeg(unsigned char legNumber, float pxold, float pyold, float pzold, float px, float py, float pz, MotionSequence &var, 
														float directRaise,float raiseDis, move_direction dirX, move_direction dirY, move_direction dirZ);
	/**
	*@function calculateLinearMotionWithRaisingLeg2
	*@brief Same functionality as function [calculateLinearMotionWithRaisingLeg]. Only difference is that desired end position is given in leg coordinates.	 
	*@param legNumber ['1', '2','3','4','5','6']
			The leg that needs to be interpolated
	*@param pxold, pyold, pzold [mm]
			Old position of the robot, given in world coordinates
	*@param pk, pky, pkz [mm]
			New desired position, given in leg coordinates
	*@param var [return-by-reference]
			Object that stores the calculated motion sequence and if enabled the velocity sequence and position sequence (position given in leg coordinates pkx,pky,pkz). 
	*@param dirX, dirY, dirZ [dir_positive, dir_negative]	
			Changes the sign of the parameters px, py ,pz
	*@param raiseDis [mm]
			Peak distance to which the leg should be raised.
	*@param directRaise [mm]
			Raise at the very beginning of the moving sequence.
	*@result (not implemented yet) returns error code, if problems occur. In current state is always returns 1
	*/														
	unsigned char calculateLinearMotionWithRaisingLeg2(unsigned char legNumber, float pxold, float pyold, float pzold, float pkx, float pky, float pkz, MotionSequence &var, float directRaise, float raiseDis);
	
	/**
	*@function calculateLinearMotion2
	*@brief Same functionality as function [calculateLinearMotion]. Only difference is that desired end position is given in leg coordinates.
	*@param legNumber ['1', '2','3','4','5','6']
			The leg that needs to be interpolated
	*@param pxold, pyold, pzold [mm]
			Old position of the robot, given in world coordinates
	*@param pkx, pky, pkz [mm]
			New desired position, given in world coordinates
	*@param var [return-by-reference]
			Object that stores the calculated motion sequence and if enabled the velocity sequence and position sequence (position given in leg coordinates pkx,pky,pkz). 
	*@result (not implemented yet) returns error code, if problems occur. In current state is always returns 1
	*/
	unsigned char calculateLinearMotion2(unsigned char legNumber, float pxold, float pyold, float pzold, float pkx, float pky, float pkz, MotionSequence &var, move_direction dir);
	
	/**
	*@function calculatePtpMotion
	*@brief Calculates and stores ptp motion (Point-To-Point-Motion) to the desired position in "var"
	*@param legNumber ['1', '2','3','4','5','6']
			Leg that should be used for the ptp motion
	*@param pxold, pyold, pzold [mm]
			Old position of the robot, given in world coordinates
	*@param px, py, pz [mm]
			New desired position, given in world coordinates
	*@param var [return-by-reference]
			Object that stores the calculated motion sequence and if enabled the velocity sequence and position sequence (position given in leg coordinates pkx,pky,pkz).
	*@result (not implemented yet) returns error code, if problems occur. In current state is always returns 1			
	*/
	unsigned char calculatePtpMotion(unsigned char legNumber, float pxold, float pyold, float pzold, float px, float py, float pz, PtPMotion &var);
	
	/**
	*@function moveLegsSimultanouslyInterpolated
	*@brief	   Every step of the sequence is first registered in every motor. And then a broadcast is send to all motors, that contains the command for driving to the registered position.
	*@param datalegs
			Pointer to six MotionSequence objects. These objects need to have position tracking enabled and they need to have the same size!
	*@result 0 if size of the MotionSequence objects varies. 
			 0 if position tracking of the MotionSequence objects is disabled.
			 0 if pLegs member object is not initialized yet
			 else 1
	*/
	unsigned char moveLegsSimultanouslyInterpolated(MotionSequence* (&datalegs)[6]);
	/**
	*@function moveLegsSimultanouslyInterpolated
	*@brief	   Every step of the sequence is first registered in every motor. And then a broadcast is send to all motors, that contains the command for driving to the 
				registered position with the registered speed.
	*@param datalegs
			Pointer to six MotionSequence objects. These objects need to have position tracking enabled and they need to have the same size!
	*@result 0 if size of the MotionSequence objects varies. 
			 0 if position tracking of the MotionSequence objects is disabled.
			 0 if speed sequence tracking of the MotionSequence objects is disabled.
			 0 if pLegs member object is not initialized yet
			 
			 else 1
	*/
	unsigned char moveLegsSimultanouslyInterpolatedWithSpeed(MotionSequence* (&datalegs)[6]);
	
	/**
	*@function moveLegsSimultanouslyPtp
	*@brief Moves all six legs of the hexapod simultaneously with the default speed.
	*@param datalegs
			Pointer to six MotionSequence objects.
	*@result 
			 0 if pLegs member object is not initialized yet	 
			 else 1
	*/											
	unsigned char moveLegsSimultanouslyPtp(struct PtPMotion dataLeg1, struct PtPMotion dataLeg2, struct PtPMotion dataLeg3, struct PtPMotion dataLeg4, struct PtPMotion dataLeg5,struct PtPMotion dataLeg6);
	
	/**
	*@function lowerLegs
	*@brief Lowers the legs of the hexapod with a ptp motion, that sets the z-coordinate to 0
	*@param lowerLeg1, lowerLeg2, lowerLeg3, lowerLeg4, lowerLeg5, lowerLeg6
				Selection of the legs that should be lowered
	*@result (not implemented yet) returns error code, if problems occur. In current state is always returns 1			
	*/
	unsigned char lowerLegs(bool lowerLeg1, bool lowerLeg2, bool lowerLeg3, bool lowerLeg4, bool lowerLeg5, bool lowerLeg6);
	
	/**
	*@function liftLegs
	*@brief lifts the legs of the hexapod with a ptp motion
	*@param lowerLeg1, lowerLeg2, lowerLeg3, lowerLeg4, lowerLeg5, lowerLeg6
				Selection of the legs that should be lift
	*@result (not implemented yet) returns error code, if problems occur. In current state is always returns 1			
	*/
	unsigned char liftLegs(bool liftLeg1, bool liftLeg2, bool liftLeg3, bool liftLeg4, bool liftLeg5, bool liftLeg6);
	
	/**
	*@function moveBodyServosToHome
	*@brief Moves the body servos to the home position
	*@param lowerLeg1, lowerLeg2, lowerLeg3, lowerLeg4, lowerLeg5, lowerLeg6
				Selection of the legs that whose body servo should be moved to home position
	*@result (not implemented yet) returns error code, if problems occur. In current state is always returns 1
	*/
	unsigned char moveBodyServosToHome(bool XYHomeLeg1, bool XYHomeLeg2, bool XYHomeLeg3, bool XYHomeLeg4, bool XYHomeLeg5, bool XYHomeLeg6);
	
	void printMotionSequence(MotionSequence& dataLeg);
	void printMotionSequenceForMatlab(int indexData, MotionSequence& dataLeg, bool endLog);
	/*######################################################################################################################################*/
	//TODO: Missing implementation
	unsigned char goToSleep();
	unsigned char goToWakeUp();
	
	//TODO: Functions that need to be removed  or replaced!
	unsigned char conversionFromMathematicalModelToMechanicalModel(unsigned char legNumber,float& q1,float& q2,float& q3);
	unsigned char registerliftLeg(unsigned char legNumber, float pkxOld, float pkyOld,float pkzOld);

	//TODO: Function that need testing
	
	//TODO: Incomplete implementation
	unsigned char doOneStepWith236(float px, float py, float pz);	
	unsigned char doOneStepWith145(float px, float py, float pz);
	unsigned char moveAllLegsToHomePos();
	unsigned char moveLegs145_SimultanouslyInterpolated(MotionSequence& dataLeg1, MotionSequence& dataLeg4, MotionSequence& dataLeg5);
	unsigned char moveLegs236_SimultanouslyInterpolated(MotionSequence& dataLeg2, MotionSequence& dataLeg3, MotionSequence& dataLeg6);
	unsigned char calculateLinearMotionWithRaisingLeg3(unsigned char legNumber, float pxold, float pyold, float pzold, float px, float py, float pz, MotionSequence &var, float raiseDis);
	unsigned char calculateLinearMotion3(unsigned char legNumber, float pxold, float pyold, float pzold, float px, float py, float pz, MotionSequence &var, move_direction dir);
	unsigned char doOneStep(float px, float py, float pz,move_mode mode);
	
	//TODO: More or less test functions that were used or were replaced
	unsigned char moveLegs(float pxold, float pyold, float pzold, float px, float py, float pz);
	unsigned char interpolationAngleEndposition(float qend, float qhome, float (&interpolatedAngleMovement)[10], float& movementSpeed);
	unsigned char interpolationAngleForSyncLinMovement(float deltaQ, float tb, float tv, float *interpolatedAngleMovement, float *interpolatedVelocity, int size);
	unsigned char moveLegOneWithInterpolatedPosition(float q1old,float q2old,float q3old, float q1, float q2, float q3);
	
	
	void setContinuesSteps(bool val);
	

	~MovementController();
private:
	unsigned char calcAveragedSpeed(const float q1,const float q2,const float q3, float& vq1,float& vq2,float& vq3);


}; //MovementController

#endif //__MOVEMENTCONTROLLER_H__
