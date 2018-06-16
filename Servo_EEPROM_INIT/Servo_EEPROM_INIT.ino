/*
 * Example showing how to use endless mode (wheel mode) on AX-12A
 * Be sure you removed all mechanical assemblies (hinges) before using this code !
 */
#include "Arduino.h"
#include "AX12A.h"

#define DirectionPin 	(10u)
#define BaudRate  		(1000000ul)
#define ID				(1u)

unsigned char newID;
unsigned char maxTmp;
//
//    CW Angle Limit: the minimum value of Goal Position (0°-150°) (0-512)
//    CCW Angle Limit: the maximum value of Goal Position (150°-300°) (512-1023)

int CWLimit;
int CCWLimit;

unsigned char DVoltage; //Down limit voltage (Range: 50 ~ 160 ->5V - 16V)
unsigned char UVoltage; //Up limit voltage


//It is the torque value of maximum output. 0 to 1,023 (0x3FF) can be used, and the unit is about 0.1%.
//For example, Data 1,023 (0x3FF) means that Dynamixel will use 100% of the maximum torque it can produce
//while Data 512 (0x200) means that Dynamixel will use 50% of the maximum torque. When the power is turned on,
//Torque Limit (Address 34 and 35) uses the value as the initial value.
int MaxTorque;

unsigned char SRL;
unsigned char RDT;

//See table http://emanual.robotis.com/docs/en/dxl/ax/ax-12a/#alarm-led -> 2.4.11
unsigned char LEDAlarm;
unsigned char SALARM;



void setup()
{
	ax12a.begin(BaudRate, DirectionPin, &Serial1);
 //Values for initialization of motors
 newID=18;
 //maxTmp=??;
 CWLimit=307;
 CCWLimit=724;
 //DVoltage=??;
 //UVoltage=??;
 //MaxTorque=??;
 //SRL=??;
 //RDT=??;
 LEDAlarm=0;
 SALARM==0b00001111;;
 initMotor();
}

void loop()
{
//test
  ax12a.moveSpeed(newID,140/0.29,100);
  delay(3000);
  ax12a.moveSpeed(newID,160/0.29,100);
  delay(3000);

}
void initMotor(){
//  ax12a.torqueStatus(3,0);
  ax12a.setID(1,newID);
  //ax12a.torqueStatus(newID,false);
  //setTempLimit(newID,maxTmp);
  //ax12a.setAngleLimit(newID,CWLimit,CCWLimit);
  //setVoltageLimit(newID,DVoltage,UVoltage);
  //setMaxTorque(newID,MaxTorque);
  //setSRL(newID,SRL);
  //setRDT(newID,RDT);
 // ax12a.setLEDAlarm(newID,LEDAlarm);
 // ax12a.setShutdownAlarm(newID,SALARM);
 // ax12a.torqueStatus(newID,true);
}

