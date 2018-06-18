/* 
* MotionSequence.h
*
* Created: 16.06.2018 14:36:39
* Author: Tomislav Romic
*/


#ifndef __MOTIONSEQUENCE_H__
#define __MOTIONSEQUENCE_H__

enum SpeedMode{SpeedSequence=1, SingleSpeed=0};
enum PositionMode{EnablePositionTracking=1, DisablePositionTracking=0};


class MotionSequence
{
//variables
private:
	int size;
	float* p_motionSequenceForQ1;
	float* p_motionSequenceForQ2;
	float* p_motionSequenceForQ3;
	
	float* p_VelocitySequenceForQ1;
	float* p_VelocitySequenceForQ2;
	float* p_VelocitySequenceForQ3;
	
	float* p_motionSequenceX;
	float* p_motionSequenceY;
	float* p_motionSequenceZ;
//functions
public:
	MotionSequence();
	MotionSequence(int size, SpeedMode valSpeedMode, PositionMode valPositionMode);
	int getSize();
	unsigned char getAngleSequenceAt(int index, float& valQ1, float& valQ2, float& valQ3);
	unsigned char getVelocitySequenceAt(int index, float& valVQ1, float& valVQ2, float& valVQ3);
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
	
	unsigned char setAngleSequenceAt(int index, float valQ1, float valQ2, float valQ3);
	unsigned char setVelocitySequenceAt(int index, float valVQ1, float valVQ2, float valVQ3);
	unsigned char setMotionSequenceAt(int index, float valX, float valY, float valZ);
	
	~MotionSequence();

}; //MotionSequence

#endif //__MOTIONSEQUENCE_H__
