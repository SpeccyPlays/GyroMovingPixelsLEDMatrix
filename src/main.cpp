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
#include "LedMatrix.h"

LedMatrix matrix (4, 3);
//Pixel object
class pixels {
  public :
  uint8_t x = 0;
  uint8_t y = 0;
  int8_t vx = 0;
  int8_t vy = 0;
  int8_t xMoveIncrement = 0;
  int8_t yMoveIncrement = 0;
  void generateXYValues(){
    //co ordinates start at 0 so one matrix will be x 0-7 and y 0-7
    x = random(0, 31);
    y = random(0, 23);
  }
  void decideMoveIncrements(int16_t &xMove, int16_t &yMove, int16_t offset){
  //check it's not a false reading then decide which direction to go
  //increase velocity
    vx ++;
    vy ++;
    if (xMove > -offset && xMove < offset){
      xMoveIncrement = 0;
    }
    else if (xMove > offset){
      xMoveIncrement = 1 * vx;
    }
    else if (xMove < -offset){
      xMoveIncrement = -1 * vx;
    }
    if (yMove > -offset && yMove < offset){
      yMoveIncrement = 0;
    }
    else if (yMove > offset){
      yMoveIncrement = -1 * vy;
    }
    else if (yMove < -offset){
      yMoveIncrement = 1 * vy;
    }
    if (vx > 4){
      vx = 4;
    }
    if (vy > 4){
      vy = 4;
    }
  };
  void updateXandY(int8_t moveX, int8_t moveY){
    //Update position as long as pixel not at edge of matrix
    //if it is at edge of matrix, stop it's velocity
    if (x + moveX < 0){
      vx = 0;
      x = 0;
    }
    else if (x + moveX > ((32) - 1)){
      vx = 0;
      x = (32) - 1;
    }
    else {
      x += moveX;
    }
    if (y + moveY < 0){
      vy = 0;
      y = 0;
    }
    else if (y + moveY > (24 -1)){
      vy = 0;
      y = (24 -1);
    }
    else{
      y += moveY;
    }
  }
};
//create the pixel object
//100 is pretty much the max due to the qsort process used later
//higher than this and too much RAM will be used and the Arduino locks up
pixels pixel [100];
int compare(const void* a, const void* b);
//GY-521 details
const int MPU=0x68;
MPU6050 gyro;
int16_t accelX, accelY;
int16_t offset = 800; //mine kept getting false readings between -600, +600
void setup() {
  Serial.begin(115200);
  Wire.begin();
  // GY 521 setup
  Serial.println("Starting gyro");
  gyro.initialize();
  //below offsets are for my gy521 only
  gyro.setXAccelOffset(1024);
  gyro.setYAccelOffset(1188);
  SPI.begin();
  matrix.init();
  //set x and y values for all pixel objects
  for (byte i = 0; i < sizeof(pixel)/sizeof(pixel[0]); i++){
    pixel[i].generateXYValues();
    matrix.drawPixel(pixel[i].x, pixel[i].y);
  }
  Serial.println("Draw buffer");
  matrix.sendScreenBuffer();
}

void loop() {
  //rather than batch update all the pixels, it looks better taking a reading for each
  //it does slow things down though
  for (byte i = 0; i < sizeof(pixel)/sizeof(pixel[0]); i++){
    accelX = gyro.getAccelerationX();
    accelY = gyro.getAccelerationY();
    pixel[i].decideMoveIncrements(accelX, accelY, offset);
    pixel[i].updateXandY(pixel[i].xMoveIncrement, pixel[i].yMoveIncrement);
  }
  //the sand behaves a bit better with a sort on y values
  //the sand will kind of stack up then slowly fall down
  qsort(pixel, sizeof(pixel)/sizeof(pixel[0]),  sizeof(pixels), compare);

  //very basic collision detect
  for (byte i = 1;  i < sizeof(pixel)/sizeof(pixel[0]); i++){
    if ((pixel[i].x == pixel[i-1].x)){
      pixel[i].vx = 0;
      pixel[i].updateXandY(-pixel[i].xMoveIncrement, 0);
    }
    if ((pixel[i].y == pixel[i-1].y)){
      pixel[i].vy = 0;
      pixel[i].updateXandY(0, -pixel[i].yMoveIncrement);
    }
    matrix.drawPixel(pixel[i].x, pixel[i].y);
  }
  matrix.sendScreenBuffer();
  matrix.wipeScreenBuffer();
};
int compare(const void* a, const void* b)
//used in the qsort
//copied this from a qsort guide then adapted to my use
{
	const pixels* x = (pixels*) a;
	const pixels* y = (pixels*) b;

	if (x->y > y->y)
		return 1;
	else if (x->y < y->y)
		return -1;

	return 0;
}