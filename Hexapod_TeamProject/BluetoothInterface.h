/*
* BluetoothInterface.h
*
* Created: 24.05.2018 09:35:26
* Author: Henri
*/
/**
*@file BluetoothInterface.h
*/
/**
*@brief Enables bluetooth communication.
*/

#ifndef __BLUETOOTHINTERFACE_H__
#define __BLUETOOTHINTERFACE_H__

#define BluetoothSerial Serial3
#define BLUETOOTH_SERIAL_BAUDRATE 9600

class BluetoothInterface
{
	//variables

	public:

	char appValue;
	int batteryStatus;
	bool batteryEmpty;
	bool btserialAvailable;
	int batteryState;
	
	private:

	int valX;
	int valY;
	int valRot;
	char dir_lr; //direction of Joystick
	char dir_ud; //direction of Joystick
	int lr; //used for readInput2
	int ud; //used for readInput2
	int currentValue; //used for readInput2
	typedef enum {  NONE, GET_X, GET_Y, GET_ROT } states; //used for readInput2
	states state = NONE;

	//functions
	public:
	BluetoothInterface();
	unsigned char readInput();
	void handlePreviousState();
	unsigned char readInput2(const char c);
	int getDirectionX();
	int getDirectionY();
	float getRotation();
	int setDirectionX(char x);
	int setDirectionY(char y);
	int setRotation(char rot);
	int sendBatteryStatus(float val);
	
	int setAccuWaechter();
	int getBatteryStatus();
	int sendBatteryStatus();
	int testSend();
	
	
	int char2int(char temp);
	
	
	~BluetoothInterface();
	int calcAngle(int appValue);

}; //BluetoothInterface

#endif //__BLUETOOTHINTERFACE_H__
