/* 
* PtPMotion.cpp
*
* Created: 16.06.2018 15:42:23
* Author: Tomislav Romic
*/


#include "PtPMotion.h"

// default constructor
PtPMotion::PtPMotion(SpeedMode valSpeedMode, PositionMode valPositionMode):MotionSequence(2,valSpeedMode,valPositionMode){
} //PtPMotion

// default destructor
PtPMotion::~PtPMotion()
{
} //~PtPMotion
