/*
Use a GY-521 6050 to move pixels around on an 8x32 MAX 7219 powered LED matrix
Board Arduino Uno

## LED Pins ##
CLK -> PIN 13
CS -> PIN 10
DIN -> PIN 11
Orientation of matrix - input on right side

## GY-521 pins ##
SDA and SCL -> marked SDA and SCL pins on board
Orientation of unit - pins are on right hand side

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
//Pixel object
class pixels {
  public :
  uint8_t x = 0;
  uint8_t y = 0;
  uint8_t mass = 0;
  void generateXYValues(){
    //co ordinates start at 0 so one matrix will be x 0-7 and y 0-7
    x = random(0, (numOfModules * ROWWIDTH) - 1);
    y = random(0, COLHEIGHT - 1);
    mass = random(1, 3); 
  }
  void updateXandY(int8_t moveX, int8_t moveY){
    //Update position as long as pixel not at edge of matrix
    if ((x + (moveX * mass)) < 0){
      x = 0;
    }
    else if ((x + (moveX * mass)) > ((numOfModules * ROWWIDTH) - 1)){
      x = (numOfModules * ROWWIDTH) - 1;
    }
    else {
      x += (moveX * mass);
    }
    if ((y + (moveY * mass)) < 0){
      y = 0;
    }
    else if ((y + (moveY * mass)) > (COLHEIGHT -1)){
      y = (COLHEIGHT -1);
    }
    else{
      y += (moveY * mass);
    }
  }
};
//create the pixel object
pixels pixel[60];
//GY-521 details
const int MPU=0x68;
MPU6050 gyro;
int16_t accelX, accelY, accelZ, gyroX, gyroY, gyroZ;
int8_t xMoveIncrement = 0;
int8_t yMoveIncrement = 0;
int16_t offset = 800; //mine kept getting false readings between -600, +600


void updateAll(uint16_t cmd, uint8_t data);
void sendScreenBuffer();
void wipeDisplays();
void drawPixel(byte x, byte y);
void wipeScreenBuffer();
void decideMoveIncrements(int16_t &xMove, int16_t &yMove);


void setup() {
  Serial.begin(115200);
  Wire.begin();
  // GY 521 setup
  Serial.println("Starting gyro");
  gyro.initialize();
  //below offsets are for my gy521 only
  gyro.setXAccelOffset(1024);
  gyro.setYAccelOffset(1188);
  gyro.setXGyroOffset(35);
  gyro.setYGyroOffset(302);
  gyro.setZGyroOffset(-4);
  // LED Matrix setup
  pinMode(CSPIN, OUTPUT);
	SPI.begin();
  wipeDisplays();
	updateAll(INTENSITY, 0);
	updateAll(DISPLAY_TEST, 0);
	updateAll(DECODE_MODE, 0);
	updateAll(SCAN_LIMIT, 7);
	updateAll(SHUTDOWN, 1);
  //set x and y values for all pixel objects
  for (byte i = 0; i < sizeof(pixel)/sizeof(pixel[0]); i++){
    pixel[i].generateXYValues();
    drawPixel(pixel[i].x, pixel[i].y);
  }
  Serial.println("Draw buffer");
  sendScreenBuffer();
}

void loop() {
  // put your main code here, to run repeatedly:
  
  /*Serial.print("Accel x : ");
  Serial.println(accelX);
  Serial.print("Accel Y : ");
  Serial.println(accelY);*/
  
  //rather than batch update all the pixels, it looks better taking a reading for each
  //it does slow things down though
  for (byte i = 0; i < sizeof(pixel)/sizeof(pixel[0]); i++){
    gyro.getMotion6(&accelX, &accelY, &accelZ, &gyroX, &gyroY, &gyroZ);
    decideMoveIncrements(accelX, accelY);
    pixel[i].updateXandY(xMoveIncrement, yMoveIncrement);
  }
  //collision detect
  for (byte i = 1;  i < sizeof(pixel)/sizeof(pixel[0]); i++){
    if ((pixel[i].x == pixel[i-1].x)){
      pixel[i].updateXandY(-xMoveIncrement, 0);
    }
    if (pixel[i].y == pixel[i-1].y){
      pixel[i].updateXandY(0, -yMoveIncrement);
    }
    drawPixel(pixel[i].x, pixel[i].y);
  }
  sendScreenBuffer();
  wipeScreenBuffer();
  delay(0);
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
void decideMoveIncrements(int16_t &xMove, int16_t &yMove){
  //check it's not a false reading then decide which direction to go
  if (xMove > -offset && xMove < offset){
    xMoveIncrement = 0;
  }
  else if (xMove > offset){
    xMoveIncrement = 1;
  }
  else if (xMove < -offset){
    xMoveIncrement = -1;
  }
  if (yMove > -offset && yMove < offset){
    yMoveIncrement = 0;
  }
  else if (yMove > offset){
    yMoveIncrement = -1;
  }
  else if (yMove < -offset){
    yMoveIncrement = 1;
  }
};