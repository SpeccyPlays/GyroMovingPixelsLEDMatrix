/*
Use a GY-521 6050 to move pixels around on an 8x32 MAX 7219 powered LED matrix
Board Arduino Uno

## LED Pins ##
CLK -> PIN 13
CS -> PIN 10
DIN -> PIN 11

## GY-521 pins ##
SDA and SCL -> marked SDA and SCL pins on board

*/

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "LEDMatrix.h"
#include "Pixels.h"

// op codes for MAX7219
#define NO_OP 0x00
#define DECODE_MODE 9
#define INTENSITY 10
#define SCAN_LIMIT 11
#define SHUTDOWN 12
#define DISPLAY_TEST 15
//CS pin
#define CSPIN 10
//LED Matrix details 
const byte numOfDevices = 4;
const byte MODULES2 = 16;
const byte MODULES3 = 24;
const byte MODULES4 = 32;
const byte ROWWIDTH = 8;
const byte COLHEIGHT = 8;
byte screenMemory[numOfDevices][8]{0};
//Pixel objects
class pixels {
  byte x = 0;
  byte y = 0;
  byte oldX = 0;
  byte oldY = 0;
  byte speedX = 0;
  byte speedY = 0;
};
//GY-521 details
const int MPU=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

void updateAll(uint16_t cmd, uint8_t data);
void clearAllDisplays();
void readGy521();

void setup() {
  Serial.begin(9600);
  //Start GY-521
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); 
  Wire.write(0);    
  Wire.endTransmission(true);
  // LED Matrix start
  pinMode(CSPIN, OUTPUT);
	SPI.begin();
  clearAllDisplays();
	updateAll(INTENSITY, 0);
	updateAll(DISPLAY_TEST, 0);
	updateAll(DECODE_MODE, 0);
	updateAll(SCAN_LIMIT, 7);
	updateAll(SHUTDOWN, 1);
}

void loop() {
  // put your main code here, to run repeatedly:
  //readGy521();
  for (byte i = 0; i < COLHEIGHT; i ++){
    updateAll(i+1, (1 << i) );
    delay(100);
  }
    for (byte i = 0; i < COLHEIGHT; i ++){
    updateAll(i+1, (128 >> i) );
    delay(100);
  }

}

void readGy521(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,12,true);  
  
  GyX=Wire.read()<<8;//|Wire.read();  
  GyY=Wire.read()<<8;//|Wire.read();  
  GyZ=Wire.read()<<8;//|Wire.read();  
  Serial.print("Gyroscope: ");
  Serial.print("X = "); Serial.print(GyX);
  Serial.print(" | Y = "); Serial.print(GyY);
  Serial.print(" | Z = "); Serial.println(GyZ);
  Serial.println(" ");
  delay(333);
}
void updateAll(uint16_t cmd, uint8_t data){
//used for sending operation codes
	uint16_t x = (cmd << 8) | data;
	SPI.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE0));
	digitalWrite(CSPIN, LOW);
	for (byte i = 0; i < numOfDevices; i++){
	  SPI.transfer16(x);
	}
	digitalWrite(CSPIN, HIGH);
	SPI.endTransaction();
}
void clearAllDisplays(){
//turns all LEDs on all displays off
  for (byte j = 1; j <= COLHEIGHT; j++){ //1 to 8 as a 0 means no op
	  for (byte i = 0; i < numOfDevices; i++){
			SPI.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE0));
			digitalWrite(CSPIN, LOW);			
			uint16_t x = (j << 8) | B00000000;
			SPI.transfer16(x);
			digitalWrite(CSPIN, HIGH);
			SPI.endTransaction();
		}
	}
}