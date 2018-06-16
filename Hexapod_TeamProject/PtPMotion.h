/* 
* PtPMotion.h
*
* Created: 16.06.2018 15:42:23
* Author: Tomislav Romic
*/


#ifndef __PTPMOTION_H__
#define __PTPMOTION_H__
#include "MotionSequence.h"

class PtPMotion : public MotionSequence{
//variables
public:
	
//functions
public:
	PtPMotion(SpeedMode valSpeedMode, PositionMode valPositionMode);
	~PtPMotion();

}; //PtPMotion

#endif //__PTPMOTION_H__
