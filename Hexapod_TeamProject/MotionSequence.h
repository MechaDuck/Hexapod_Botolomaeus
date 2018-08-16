/* 
* MotionSequence.h
*
* Created: 16.06.2018 14:36:39
* Author: Tomislav Romic
*/
/**
*@file MotionSequence.h
*/

#ifndef __MOTIONSEQUENCE_H__
#define __MOTIONSEQUENCE_H__

enum SpeedMode{SpeedSequence=1, SingleSpeed=0};
enum PositionMode{EnablePositionTracking=1, DisablePositionTracking=0};

/*!
*@brief Helper class to store motion sequences with desired velocities. Used for interpolated movement.
*/
class MotionSequence
{
//variables
private:
	int size; /*!< Number of steps between start and end position. */
	float* p_motionSequenceForQ1; /*!< Angle sequence from start to end position for angle q1 */
	float* p_motionSequenceForQ2; /*!< Angle sequence from start to end position for angle q2 */
	float* p_motionSequenceForQ3; /*!< Angle sequence from start to end position for angle q3 */
	
	float* p_VelocitySequenceForQ1; /*!< Velocity sequence from start to end position for angle q1 */
	float* p_VelocitySequenceForQ2; /*!< Velocity sequence from start to end position for angle q2 */
	float* p_VelocitySequenceForQ3; /*!< Velocity sequence from start to end position for angle q3 */
	
	float* p_motionSequenceX;		/*!< Position sequence for X coordinates from start to end position  */
	float* p_motionSequenceY;		/*!< Position sequence for Y coordinates from start to end position  */
	float* p_motionSequenceZ;  /*!< Position sequence for Z coordinates from start to end position  */
	
	bool m_VelocityEnabled;		/*!< Enables velocity tracking */
	bool m_PositionEnabled;		/*!< Enables posiition tracking */
//functions
public:
	MotionSequence();
	/**
	*@function MotionSequence
	*@brief Arrays are allocated with a desired size that are used for storing the angular sequence.
			If enabled, also arrays are allocated that store velocity and position sequences.
	*/
	MotionSequence(int size, SpeedMode valSpeedMode, PositionMode valPositionMode);
	/**
	*@function getSize
	*@result returns size of allocated arrays.
	*/
	int getSize();
	/**
	*@function getAngleSequenceAt
	*@brief Returns values on desired array position.
	*@param index
				Desired array position 
	*@param valQ1, valQ2, valQ3 [return-by-reference]
				returns values for angular sequence on desired position 
	*@result returns 0 if index is out of bounds.
	*/
	unsigned char getAngleSequenceAt(int index, float& valQ1, float& valQ2, float& valQ3);
	
	/**
	*@function getVelocitySequenceAt
	*@brief Returns values on desired array position.
	*@param index
				Desired array position 
	*@param valVQ1, valVQ2, valVQ3 [return-by-reference]
				returns values for velocity sequence on desired position 
	*@result returns 0 if index is out of bounds.
	*/	
	unsigned char getVelocitySequenceAt(int index, float& valVQ1, float& valVQ2, float& valVQ3);
	/**
	*@function getMotionSequenceAt
	*@brief Returns values on desired array position.
	*@param index
				Desired array position 
	*@param valX, valY, valZ [return-by-reference]
				returns values for position sequence on desired position 
	*@result returns 0 if index is out of bounds.
	*/	
	unsigned char getMotionSequenceAt(int index, float& valX, float& valY, float& valZ);
	
	float getQ1AngleSequenceAt(int index);
	float getQ2AngleSequenceAt(int index);
	float getQ3AngleSequenceAt(int index);
	
	float getXMotionSequenceAt(int index);
	float getYMotionSequenceAt(int index);
	float getZMotionSequenceAt(int index);
	float getLastXValue();
	float getLastYValue();
	float getLastZValue();
	
	float getQ1VeloSequenceAt(int index);
	float getQ2VeloSequenceAt(int index);
	float getQ3VeloSequenceAt(int index);
	
	
	/**
	*@function setAngleSequenceAt
	*@brief sets angles at desired position.
	*@param index
				Desired array position where given values should be stored.
	*@param valQ1, valQ2, valQ3
				Values for angular sequence that should be stored on desired position
	*@result returns 0 if index is out of bounds.
	*/
	unsigned char setAngleSequenceAt(int index, float valQ1, float valQ2, float valQ3);
	/**
	*@function setVelocitySequenceAt
	*@brief sets velocity at desired position.
	*@param index
				Desired array position where given values should be stored.
	*@param valVQ1, valVQ2, valVQ3
				Values for velocity sequence that should be stored on desired position
	*@result returns 0 if index is out of bounds.
	*/
	unsigned char setVelocitySequenceAt(int index, float valVQ1, float valVQ2, float valVQ3);
	
	/**
	*@function setMotionSequenceAt
	*@brief sets position sequence at desired position.
	*@param index
				Desired array position where given values should be stored.
	*@param valX, valY, valZ
				Values for position sequence that should be stored on desired position
	*@result returns 0 if index is out of bounds.
	*/
	unsigned char setMotionSequenceAt(int index, float valX, float valY, float valZ);
	/**
	*@function getVelocityEnableStatus
	*@result returns true if velocity sequence tracking is enabled
	*/
	bool getVelocityEnableStatus();
	/**
	*@function getPositionEnableStatus
	*@result returns true if position sequence tracking is enabled
	*/
	bool getPositionEnableStatus();
	
	~MotionSequence();

}; //MotionSequence

#endif //__MOTIONSEQUENCE_H__
