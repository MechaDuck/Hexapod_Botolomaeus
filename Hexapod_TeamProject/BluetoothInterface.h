/*
* BluetoothInterface.h
*
* Created: 24.05.2018 09:35:26
* Author: henri
*/


#ifndef __BLUETOOTHINTERFACE_H__
#define __BLUETOOTHINTERFACE_H__

#define BluetoothSerial Serial3

class BluetoothInterface
{
	//variables

	public:

	char appValue;
	int batteryStatus;
	bool batteryEmpty;
	bool btserialAvailable;

	private:

	int valX;
	int valY;
	int valRot;
	char dir_lr; //direction of Joystick
	char dir_ud; //direction of Joystick


	//functions
	public:
	BluetoothInterface();
	unsigned char readInput();
	int sendData();
	int getDirectionX();
	int getDirectionY();
	int getRotation();
	int setDirectionX(char x);
	int setDirectionY(char y);
	int setRotation(char rot);
	unsigned char hello();
	
	int setAccuWaechter();
	int getBatteryStatus();
	int sendBatteryStatus();
	int testSend();
	
	
	int char2int(char temp);
	
	
	~BluetoothInterface();
	int calcAngle(int appValue);
	


}; //BluetoothInterface

#endif //__BLUETOOTHINTERFACE_H__
