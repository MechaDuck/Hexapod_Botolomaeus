/* 
* MotionSequence.cpp
*
* Created: 16.06.2018 14:36:39
* Author: Tomislav Romic
*/


#include "MotionSequence.h"

// default constructor
MotionSequence::MotionSequence(int size, SpeedMode valSpeedMode, PositionMode valPositionMode){
	this->size=size;
	p_motionSequenceForQ1= new float[size];
	p_motionSequenceForQ2= new float[size];
	p_motionSequenceForQ3= new float[size];

	if(valSpeedMode==SpeedSequence){
		p_VelocitySequenceForQ1 =new float[size];
		p_VelocitySequenceForQ2 =new float[size];
		p_VelocitySequenceForQ3 =new float[size];
	}else{
		p_VelocitySequenceForQ1=0;
		p_VelocitySequenceForQ2=0;
		p_VelocitySequenceForQ3=0;
	}
	if(valPositionMode==EnablePositionTracking){
		p_motionSequenceX =new float[size];
		p_motionSequenceY =new float[size];
		p_motionSequenceZ =new float[size];
	}else{
		p_motionSequenceX=0;
		p_motionSequenceY=0;
		p_motionSequenceZ=0;
	}
	
} //MotionSequence

int MotionSequence::getSize(){
	return size;
}

unsigned char MotionSequence::getAngleSequenceAt(int index, float& valQ1, float& valQ2, float& valQ3){
	if(index >= size || p_motionSequenceForQ1 == 0 || p_motionSequenceForQ2 == 0 || p_motionSequenceForQ1 == 0){
		return 0;
	}
	valQ1=p_motionSequenceForQ1[index];
	valQ2=p_motionSequenceForQ2[index];
	valQ3=p_motionSequenceForQ3[index];
	return 1;
}

unsigned char MotionSequence::getVelocitySequenceAt(int index, float& valVQ1, float& valVQ2, float& valVQ3){
	if(index >= size || p_VelocitySequenceForQ1 == 0 || p_VelocitySequenceForQ2 == 0 || p_VelocitySequenceForQ3 == 0){
		return 0;
	}
	valVQ1=p_VelocitySequenceForQ1[index];
	valVQ2=p_VelocitySequenceForQ2[index];
	valVQ3=p_VelocitySequenceForQ3[index];
	return 1;
}

unsigned char MotionSequence::setVelocitySequenceAt(int index, float valVQ1, float valVQ2, float valVQ3){
	if(index >= size || p_VelocitySequenceForQ1 == 0 || p_VelocitySequenceForQ2 == 0 || p_VelocitySequenceForQ3 == 0){
		return 0;
	}
	p_VelocitySequenceForQ1[index]=valVQ1;
	p_VelocitySequenceForQ2[index]=valVQ2;
	p_VelocitySequenceForQ3[index]=valVQ3;
	return 1;
}

unsigned char MotionSequence::setMotionSequenceAt(int index, float valX, float valY, float valZ){
	if(index >= size || p_motionSequenceX == 0 || p_motionSequenceY == 0 || p_motionSequenceZ == 0){
		return 0;
	}
	p_motionSequenceX[index]=valX;
	p_motionSequenceY[index]=valY;
	p_motionSequenceZ[index]=valZ;
	return 1;
}

unsigned char MotionSequence::getMotionSequenceAt(int index, float& valX, float& valY, float& valZ){
	if(index >= size || p_motionSequenceX == 0 || p_motionSequenceY == 0 || p_motionSequenceZ == 0){
		return 0;
	}
	valX=p_motionSequenceX[index];
	valY=p_motionSequenceY[index];
	valZ=p_motionSequenceZ[index];
	return 1;
}

float MotionSequence::getQ1AngleSequenceAt(int index){
	if(index >= size){
		return -1;
	}
	return p_motionSequenceForQ1[index];
}

float MotionSequence::getQ2AngleSequenceAt(int index){
	if(index >= size){
		return -1;
	}
	return p_motionSequenceForQ2[index];
}

float MotionSequence::getQ3AngleSequenceAt(int index){
	if(index >= size){
		return -1;
	}
	return p_motionSequenceForQ3[index];
}

float MotionSequence::getXMotionSequenceAt(int index){
	if(index >= size || p_motionSequenceX == 0){
		return 0;
	}
	return p_motionSequenceX[index];
}

float MotionSequence::getYMotionSequenceAt(int index){
	if(index >= size || p_motionSequenceY == 0){
		return 0;
	}
	return p_motionSequenceY[index];
}

float MotionSequence::getZMotionSequenceAt(int index){
	if(index >= size || p_motionSequenceZ == 0){
		return 0;
	}
	return p_motionSequenceZ[index];
}

unsigned char MotionSequence::setAngleSequenceAt(int index, float valQ1, float valQ2, float valQ3){
	if(index >= size || p_motionSequenceForQ1 == 0 || p_motionSequenceForQ2 == 0 || p_motionSequenceForQ1 == 0){
		return 0;
	}
	p_motionSequenceForQ1[index]=valQ1;
	p_motionSequenceForQ2[index]=valQ2;
	p_motionSequenceForQ3[index]=valQ3;
	
	return 1;
}

// default destructor
MotionSequence::~MotionSequence(){
	delete[] p_motionSequenceForQ1;
	delete[] p_motionSequenceForQ2;
	delete[] p_motionSequenceForQ3;
	if(p_VelocitySequenceForQ1 != 0 || p_VelocitySequenceForQ2 != 0 || p_VelocitySequenceForQ3 != 0){
		delete[] p_VelocitySequenceForQ1;
		delete[] p_VelocitySequenceForQ2;
		delete[] p_VelocitySequenceForQ3;
	}
	if(p_motionSequenceX != 0 || p_motionSequenceY != 0 || p_motionSequenceZ != 0){
		delete[] p_motionSequenceX;
		delete[] p_motionSequenceY;
		delete[] p_motionSequenceZ;
	}
} //~MotionSequence
