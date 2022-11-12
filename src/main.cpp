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
  uint8_t mass = 0;
  void generateXYValues(){
    //co ordinates start at 0 so one matrix will be x 0-7 and y 0-7
    x = random(0, 31);
    y = random(0, 23);
    mass = 1; 
  }
  void updateXandY(int8_t moveX, int8_t moveY){
    //Update position as long as pixel not at edge of matrix
    if ((x + (moveX * mass)) < 0){
      x = 0;
    }
    else if ((x + (moveX * mass)) > ((32) - 1)){
      x = (32) - 1;
    }
    else {
      x += (moveX * mass);
    }
    if ((y + (moveY * mass)) < 0){
      y = 0;
    }
    else if ((y + (moveY * mass)) > (24 -1)){
      y = (24 -1);
    }
    else{
      y += (moveY * mass);
    }
  }
};
//create the pixel object
pixels pixel[120];
void decideMoveIncrements(int16_t &xMove, int16_t &yMove);
//GY-521 details
const int MPU=0x68;
MPU6050 gyro;
int16_t accelX, accelY;
int8_t xMoveIncrement = 0;
int8_t yMoveIncrement = 0;
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
    matrix.drawPixel(pixel[i].x, pixel[i].y);
  }
  matrix.sendScreenBuffer();
  matrix.wipeScreenBuffer();
  delay(0);
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