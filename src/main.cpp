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
const byte numOfModules = 4;
//don't think these are needed
const byte MODULES2 = 16;
const byte MODULES3 = 24;
const byte MODULES4 = 32;
const byte ROWWIDTH = 8;
const byte COLHEIGHT = 8;
byte screenBuffer[numOfModules][8]{0};
//Pixel objects
class pixels {
  public :
  uint8_t x = 0;
  uint8_t y = 0;
  //these are used to check what movement is happening
  int8_t oldMovementX = 0;
  int8_t oldMovementY = 0;
  int8_t movementX = 0;
  int8_t movementY = 0;
  void generateXYValues(){
    //co ordinates start at 0 so one matrix will be x 0-7 and y 0-7
    x = random(0, (numOfModules * ROWWIDTH) - 1);
    y = random(0, COLHEIGHT - 1);
  }
  void updateMovement(int8_t newMoveX, int8_t newMoveY){
    //update the change in movement
    oldMovementX = movementX;
    oldMovementY = movementY;
    movementX = newMoveX;
    movementY = newMoveY;
  }
  void updateXandY(){
    if (x < ((numOfModules * ROWWIDTH) - 1) && x < movementX){
      x ++;
    }
    else if (x > 0 && x > movementX){
      x--;
    }
    if (y > 0 && y > movementY){
      y --;
    }
    else if (y < COLHEIGHT - 1 && y < movementY){
      y ++;
    }
    if (y == movementY){
      uint8_t bounce = random(-2, 2);
      y += bounce;
    }
    if (x == movementX){
      uint8_t bounce = random(-3, 3);
      x += bounce;
    }
    if (x < 0){
      x = 0;
    }
    if (x > 31){
      x = 31;
    }
    if (y < 0){
      y = 0;
    }
    if (y > 7){
      y = 7;
    }
      /*
    if (x < ((numOfModules * ROWWIDTH) - 1) && movementX > oldMovementX){
      x ++;
    }
    else if (x > 0 && movementX < oldMovementX){
      x--;
    }
    if (y > 0 && movementY < oldMovementY){
      y --;
    }
    else if (y < COLHEIGHT - 1 && movementY > oldMovementY){
      y ++;
    }*/
  }
};
pixels pixel[8];
//GY-521 details
const int MPU=0x68;
MPU6050 gyro;
const int16_t minGyroRead = -32768;
const int16_t maxGyroRead = 32767;
int16_t accelX, accelY, accelZ, gyroX, gyroY, gyroZ;

void updateAll(uint16_t cmd, uint8_t data);
void sendScreenBuffer();
void wipeDisplays();
void drawPixel(byte x, byte y);
void wipeScreenBuffer();

void setup() {
  Serial.begin(115200);
  Wire.begin();
  // GY 521 setup
  Serial.print("Starting gyro");
  gyro.initialize();
  /*gyro.CalibrateGyro(20);
  gyro.setXGyroOffset(220);
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
  for (byte i = 0; i < sizeof(pixel)/sizeof(pixel[0]); i++){
    pixel[i].generateXYValues();
    drawPixel(pixel[i].x, pixel[i].y);
  }
  Serial.println("Draw buffer");
  sendScreenBuffer();
}

void loop() {
  // put your main code here, to run repeatedly:
  gyro.getMotion6(&accelX, &accelY, &accelZ, &gyroX, &gyroY, &gyroZ);
  uint8_t mappedX = map(gyroX, minGyroRead, maxGyroRead, 0, 31);
  uint8_t mappedY = map(gyroY, minGyroRead, maxGyroRead, 7, 0);
  for (byte i = 0; i < sizeof(pixel)/sizeof(pixel[0]); i++){
    pixel[i].updateMovement(mappedX, mappedY);
    pixel[i].updateXandY();
    drawPixel(pixel[i].x, pixel[i].y);
  }
  sendScreenBuffer();
  wipeScreenBuffer();
  delay(50);
}
void drawPixel(byte x, byte y){
  //draw a pixel in screen memory
  uint8_t temp = 128; //set MSB
  uint8_t module = int(floor(x /8));
  screenBuffer[module][y] = screenBuffer[module][y] | (temp >> int(x % 8));
}
void wipeScreenBuffer(){
  //zero the whole screen buffer
  for (byte i = 0; i < numOfModules; i++){
    for (byte j = 0; j < COLHEIGHT; j++){
      screenBuffer[i][j] = 0;
	  }
	}
}

void sendScreenBuffer(){
  //updates the matrixes with the screen buffer contents
	for (byte j = 0; j < COLHEIGHT; j++){
  	SPI.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE0));
		digitalWrite(CSPIN, LOW);
		for (byte i = 0; i < numOfModules; i++){
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
	for (byte i = 0; i < numOfModules; i++){
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