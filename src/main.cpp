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
#include <I2Cdev.h>
#include <MPU6050.h>
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
byte screenBuffer[numOfDevices][8]{0};
//Pixel objects
class pixels {
  public :
  uint8_t x = 4;
  uint8_t y = 4;
  //these are used to check what movement is happening
  int8_t oldMovementX = 0;
  int8_t oldMovementY = 0;
  int8_t movementX = 0;
  int8_t movementY = 0;
  void updateMovement(int8_t newMoveX, int8_t newMoveY){
    //see what the change in movement is
    oldMovementX = movementX;
    oldMovementY = movementY;
    movementX = newMoveX;
    movementY = newMoveY;
  }
  void updateXandY(){
    
    if (x < 128 && movementX > oldMovementX){
      x = x << 1;
    }
    if (x > 1 && movementX < oldMovementX){
      x = x >> 1;
    }
    if (y > 0 && movementY < oldMovementY){
      y --;
    }
    if (y < COLHEIGHT - 1 && movementY > oldMovementY){
      y ++;
    }
  }
};
pixels pixel;
//GY-521 details
const int MPU=0x68;
MPU6050 gyro;
const int16_t minGyroRead = -32768;
const int16_t maxGyroRead = 32767;
int16_t accelX, accelY, accelZ, gyroX, gyroY, gyroZ;

void updateAll(uint16_t cmd, uint8_t data);
void sendScreenBuffer();
void wipeDisplays();

void setup() {
  Serial.begin(9600);
  Wire.begin();
  // GY 521 setup
  Serial.print("Starting gyro");
  gyro.initialize();
  /*gyro.setXGyroOffset(220);
  gyro.setYGyroOffset(76);
  gyro.setZGyroOffset(-85);*/
  // LED Matrix setup
  pinMode(CSPIN, OUTPUT);
	SPI.begin();
  wipeDisplays();
	updateAll(INTENSITY, 0);
	updateAll(DISPLAY_TEST, 0);
	updateAll(DECODE_MODE, 0);
	updateAll(SCAN_LIMIT, 7);
	updateAll(SHUTDOWN, 1);
}

void loop() {
  // put your main code here, to run repeatedly:
  /*gyro.getMotion6(&accelX, &accelY, &accelZ, &gyroX, &gyroY, &gyroZ);
  pixel.updateMovement(map(gyroX, minGyroRead, maxGyroRead, -50, 50), map(gyroY, minGyroRead, maxGyroRead, -50, 50));
  pixel.updateXandY();
  updateAll(pixel.y + 1, pixel.x);
  delay(100);
  clearAllDisplays();*/
  uint8_t x = random(0, MODULES3);
  uint8_t y = random(0, 8);
  Serial.print("Initial number : ");
  Serial.println(x);
  Serial.print("Divided by 8 : ");
  Serial.println(floor(x / 8)); //use this for getting the right module number
  Serial.print("Modelo : ");
  Serial.println(x % 8);
  uint8_t temp = 128; //set HSB
  screenBuffer[int(floor(x /8))][y] = temp >> int(x % 8); //move down from HSB otherwise it appears at the wrong end
  sendScreenBuffer();
  delay(10000);
  wipeDisplays();
  delay(1000);
}

void sendScreenBuffer(){
  //updates the matrixes with the screen buffer contents
	for (byte j = 0; j < COLHEIGHT; j++){
  	SPI.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE0));
		digitalWrite(CSPIN, LOW);
		for (byte i = 0; i < numOfDevices; i++){
			uint16_t temp = (j + 1) << 8 | screenBuffer[i][j];
      SPI.transfer16(temp);
		}
		digitalWrite(CSPIN, HIGH);
		SPI.endTransaction();	
	}
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
void wipeDisplays(){
  for (byte colNumber = 1; colNumber <= COLHEIGHT; colNumber++){
    updateAll(colNumber, 0);
  }
}