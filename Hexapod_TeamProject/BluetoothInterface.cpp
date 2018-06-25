/*/*
* BluetoothInterface.cpp
*
* Created: 24.05.2018 09:35:26
* Author: henri
*/


#include "BluetoothInterface.h"
#include <HardwareSerial.h>
#include <Arduino.h>
#define waechterPin 3


// default constructor
BluetoothInterface::BluetoothInterface(){
pinMode(12, OUTPUT); //also pin 12 as LED output
BluetoothSerial.begin(9600);
pinMode(waechterPin, INPUT);
//Serial.begin(9600);
valX=0;
valY=0;
valRot=0;


} //BluetoothInterface



unsigned char BluetoothInterface::readInput()
{
char data;
char dir_X; // Value of Joystick in X Direction
char dir_Y; // Value of Joystick in Y Direction
char rot; //Rotation (0=stillstand, 1 = , 2 = )
char output[8];
int count = 0;

while(count != 10){


if(BluetoothSerial.available() > 0){


btserialAvailable = true;
data = BluetoothSerial.read();

//Serial.print("Data is: ");
//Serial.println(data);
//Serial.print("Count: ");
//Serial.println(count);

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

// 			Serial.print("Here in while, Count = ");
// 			Serial.println(count);
switch (count)
{
case 0:				//first value, expecting X
if(data == 'X'){
count++;
//Serial.print("data = ");
//Serial.println(data);
//output[count] = data;
}
else{
//Serial.println("case 0");
}

break;


case 1:		// LR

dir_lr = data;
count++;
//Serial.print("direction of joystick (lr) = ");
//Serial.println(dir_lr);


//Serial.println("case 1");
//output[count] = dir_lr;

break;


case 2:			//value 0 ... 9

if(char2int(data) >= 0 && char2int(data) < 10){
dir_X = data;
setDirectionX(dir_X);
count++;
//Serial.print("Value X = ");
//Serial.println(v_x);
}

else{
Serial.println("case 2");
}
break;


case 3: // Y

if(data == 'Y'){
count++;
//Serial.print("data = ");
//Serial.println(data);
}
else{
//Serial.println("case 3");

}
break;


case 4:

dir_ud = data;
count++;
// 				Serial.print("direction of joystick(ud) = ");
// 				Serial.println(dir_ud);

//Serial.println("case 4");
break;


case 5: //Y value
if(char2int(data) >= 0 && char2int(data) < 10){
dir_Y = data;
setDirectionY(dir_Y);
count++;
//Serial.print("Value Y = ");
//Serial.println();
}

else{
Serial.println("case 5");
}
//output[count] = dir_Y;
break;


case 6: // Rotation

if(data == 'Z'){
count++;
//Serial.print("data = ");
//Serial.println(data);
}
else{
//Serial.println("case 6");
}

break;
//output[count] = data;


case 7: //0 ; 1; 2
rot = data;
setRotation(rot);
// 				Serial.print("Value Z = ");
//
// 				Serial.println(rot);

//Serial.println("case 7");
count = 10;

//output[count] = rot;
break;
default:
Serial.println("default");


}


}
else
{
btserialAvailable = false;
}

}
}

int BluetoothInterface::sendData()
{
Serial.println("Now in Loop");
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
	Serial.println("X:");
	Serial.println(tmp);
	if(tmp >=-50 && tmp<=50){
		return tmp;
		}else{
		return 0;
	}
	

}

int BluetoothInterface::getDirectionY(){
	float tmp;
	tmp=map(valY, -9, 9, -50, 50);
	Serial.println("Y");
	Serial.println(tmp);
	if(tmp >=-50 && tmp<=50){
		return tmp;
		}else{
		return 0;
	}
}

int BluetoothInterface::getRotation(){
if(btserialAvailable)
{
Serial.print("ROT = ");
Serial.println(valRot);
}
return valRot;
}

int BluetoothInterface::setDirectionX(char x)
{
valX = char2int(x);
if(dir_lr == 'L')
{
valX = -valX;
}
// 	Serial.print("Value X = ");
// 	Serial.println(valX);
}

int BluetoothInterface::setDirectionY(char y)
{
valY = char2int(y);
if(dir_ud == 'D')
{
valY = -valY;
}
// 	Serial.print("Value Y = ");
// 	Serial.println(valY);

}

int BluetoothInterface::setRotation(char rot)
{
valRot = char2int(rot);
// 	Serial.print("Value Rotation = ");
// 	Serial.println(valRot);

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




