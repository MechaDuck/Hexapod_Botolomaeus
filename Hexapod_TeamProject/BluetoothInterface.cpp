/* 
* BluetoothInterface.cpp
*
* Created: 24.05.2018 09:35:26
* Author: henri
*/


#include "BluetoothInterface.h"
#include "HardwareSerial.h"
#include <Arduino.h>
#define waechterPin 3
#define BLUETOOTH_SERIAL_BAUDRATE 9600


// default constructor
BluetoothInterface::BluetoothInterface(){
pinMode(12, OUTPUT); //also pin 12 as LED output
BluetoothSerial.begin(BLUETOOTH_SERIAL_BAUDRATE);
pinMode(waechterPin, INPUT);
//Serial.begin(9600);
valX=0;
valY=0;
valRot=0;


} //BluetoothInterface



unsigned char BluetoothInterface::readInput()
{
	/*	The following switch case is for synchronizing the output of the Android Smartphone with the Arduino. The counter only rises, if an expected appears.
	/	Android sends a string of chars with following layout:	X(L/R)(0...9)Y(U/D)(0...9)Z(0...2)
	/															X		-> Indicates, that the next value will be for Joystick left/right movement
	/															L/R		-> Value, if Joystick moves left or right
	/															0...9	-> actual value, how fast the Hexapod shall move
	/															Y		-> Indicates, that the next value will be for Joystick up/down movement
	/															U/D		-> Value, if Joystick moves up or down
	/															0...9	-> actual value, how fast the Hexapod shall move
	/															Z		-> Indicates, that the next value will be for the rotation
	/															0...2	-> Direction of rotation:	0 => straight forward, 
	/																								1 => move left
	/																								2 => move right						 
	*/	
		char data;
		char dir_X; // Value of Joystick in X Direction
		char dir_Y; // Value of Joystick in Y Direction
		char rot; //Rotation (0=stillstand, 1 = , 2 = )
		int count = 0;
		

		while(BluetoothSerial.available() > 0){		
			
				btserialAvailable = true;
				data = BluetoothSerial.read();
				if(data==-1){
					return 0;
				}									

				switch (count){
					case 0:				//first value, expecting X
					if(data == 'X'){
						count++;
					}
					
					break;
				
					case 1:		// LR
					if(data == 'L' | data == 'R'){
						dir_lr = data;
						count++;
					}
					break;
						
					case 2:			//value 0 ... 9
					if(char2int(data) >= 0 && char2int(data) < 10){
						dir_X = data;
						setDirectionX(dir_X);
						count++;
					}
					break;
				
				
					case 3: // Y
					if(data == 'Y'){
						count++;
					}
					break;
				
				
					case 4:
					if(data == 'U' | data == 'D'){
						dir_ud = data;
						count++;	
					}
					break;
				
				
					case 5: //Y value
					if(char2int(data) >= 0 && char2int(data) < 10){
						dir_Y = data;
						setDirectionY(dir_Y);			
						count++;
					}
					break;
				
				
					case 6: // Rotation
				
					if(data == 'Z'){
						count++;
					}
				
					break;				
				
				
					case 7: //0 ; 1; 2
					if(char2int(data) == 0 | char2int(data) == 1 | char2int(data) == 2){
						rot = data;
						setRotation(rot);
						count = 0;
					}else{
						setRotation(0);
					}
					BluetoothSerial.flush();
					break;
					
					default:
						count=0;
					break;
					}
		}
}


void BluetoothInterface::handlePreviousState()
{
	switch (state)
	{
		case GET_X:
		valX = currentValue * lr;
				Serial.print("X value: ");
				Serial.println(valX);
		lr = 1;
		break;
		
		case GET_Y:
		valY = currentValue * ud;
						Serial.print("Y value: ");
						Serial.println(valY);
		ud = 1;
		break;
		
		case GET_ROT:
		valRot = currentValue;
		break;
		
		default:
		break;
	}  // end of switch

	currentValue = 0;
}

unsigned char BluetoothInterface::readInput2(char c)
{
	btserialAvailable = true;

	if(isdigit(c)){
		currentValue = char2int(c);

	}

	else{
		// The end of the number signals a state change
		handlePreviousState ();
		if(c == 'L' | c == 'R'){
			if(c == 'L'){
				lr = -1;
			}
			
			c = 'X';
		}
		else if(c == 'U' | c == 'D'){
			if(c == 'D'){
				ud = -1;
			}			
			
			c = 'Y';
		}

		// set the new state, if we recognize it
		switch (c)
		{
			case 'X':
			state = GET_X;
			Serial.println("GET_X");
			break;
	
			case 'Y':
			state = GET_Y;
			Serial.println("GET_Y");
			break;
			
			case 'Z':
			state = GET_ROT;
			break;
			
			default:
			state = NONE;
			break;
		}  // end of switch on incoming byte
	} // end of not digit
	
	
	
}
/*
unsigned char BluetoothInterface::readInput3(char c)
{
	char data;
	char dir_X; // Value of Joystick in X Direction
	char dir_Y; // Value of Joystick in Y Direction
	char rot; //Rotation (0=stillstand, 1 = , 2 = )
	int count = 0;
	
	if(BluetoothSerial.available() > 0){
		while(count != 10){
			btserialAvailable = true;
			data = BluetoothSerial.read();
			switch (count){
				case 0:				//first value, expecting X
				
				if(data == 'X')
				state = GET_X;
				count++;
				
				break;
				
				case 1:		// LR
				if(data == 'L' | data == 'R'){
					dir_lr = data;
					count++;
				}
				break;
				
				case 2:			//value 0 ... 9
				if(state == GET_X){
					if(char2int(data) >= 0 && char2int(data) < 10){
						dir_X = data;
						setDirectionX(dir_X);
						count++;
						//Serial.print("Value X = ");
						//Serial.println(v_x);
					}
				}
				break;
			return 0;
			
		}
	}
}*/

int BluetoothInterface::sendData()
{

	if(BluetoothSerial.available ()>0)
	{
		Serial.println("BT is available");
	
		char buffer_value = BluetoothSerial.read();
		Serial.println(buffer_value);
		if(buffer_value == 'a' || buffer_value == 'A')
		{
			digitalWrite(13, HIGH);    //Turn ON LED
			Serial.println("LED ON");  //Arduino Terminal of Desktop
			BluetoothSerial.println("LED ON"); //Bluetooth Terminal on Mobile
		}
		else if(buffer_value == 'b' || buffer_value == 'B')
		{
			digitalWrite(13, LOW);      //Turn OFF LED
			Serial.println("LED OFF");  //Arduino Terminal on Desktop
			BluetoothSerial.println("LED OFF"); //Bluetooth Terminal on Mobile
		}
	}
}

int BluetoothInterface::char2int(char temp){
	int integer = (int) temp - '0';
	return integer;
}

int BluetoothInterface::calcAngle(int appValue){
	int param = 10;
	//int appValue = (int) tempappValue,
	
	appValue = appValue*param;
	return appValue;

}
int BluetoothInterface::getDirectionX(){
	float tmp;

	tmp=map(valX, -9, 9, -50, 50);
	if(tmp >=-50 && tmp<=50){
		
		return tmp;
		}else{
		return 0;
	}
	

}

int BluetoothInterface::getDirectionY(){
	float tmp;

	tmp=map(valY, -9, 9, -50, 50);
	if(tmp >=-50 && tmp<=50){
		return tmp;
		}else{
		return 0;
	}
}

int BluetoothInterface::getRotation(){
	if(valRot==0){
		return 0;
	}else if(valRot==1){
		return (-25.0/180.0*M_PI);
		
	}else if(valRot==2){
		return (25.0/180.0*M_PI);
	}
	return 0;
}

int BluetoothInterface::setDirectionX(char x)
{
	valX = char2int(x);
	if(dir_lr == 'L')
	{
		valX = -valX;
	}
}

int BluetoothInterface::setDirectionY(char y)
{
	valY = char2int(y);
	if(dir_ud == 'D')
	{
		valY = -valY;
	}

}

int BluetoothInterface::setRotation(char rot)
{
	valRot = char2int(rot);

}

unsigned char BluetoothInterface::hello()
{
	Serial.println("Class is working");
	delay(50);
}


//This function reads the Pin of the Waechter which is setting an alarm, when the Battery is very low.
int BluetoothInterface::setAccuWaechter()
{
	batteryEmpty = digitalRead(waechterPin);

}
 // This function is for reading the battery values. 
int BluetoothInterface::getBatteryStatus()
{
	
}
 // Here the battery status will be send to the android device.
 // If the waechter alarms, a warning will appear on the screen of the android app
 int BluetoothInterface::sendBatteryStatus()
{
	
	if(batteryEmpty)
	{
	BluetoothSerial.println("Battery Empty String");	// Mit Philine abklären, was sie ab besten auslesen kann. 
														// Diese Funktion soll in Android einen "Alarm" auslösen, dass der Akku komplett entladen ist.
	}
	
	// Hier wird die Get Funktion der Variable von Tommy eingefügt und in den Wert 0-3 umgerechnet und weitergegeben
	
	
	
	// batteryStatus = calculation of value from Thomy
	
	BluetoothSerial.println("Hier kommt der Wert für die Battery Bar");
	BluetoothSerial.println(batteryStatus);
	
	
	
}

int BluetoothInterface::testSend()
{
	int Sendtest = 0;
	BluetoothSerial.println(Sendtest);
	Sendtest++;
	delay(1000);
	BluetoothSerial.println(Sendtest);
	Sendtest++;
	delay(1000);
	BluetoothSerial.println(Sendtest);
	Sendtest++;
	delay(1000);
	BluetoothSerial.println(Sendtest);
	Sendtest++;
	delay(1000);
	
}

// default destructor
BluetoothInterface::~BluetoothInterface()
{
} //~BluetoothInterface