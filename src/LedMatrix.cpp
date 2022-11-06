#include "LedMatrix.h"
#include <Arduino.h>
#include <SPI.h>

LedMatrix::LedMatrix(uint8_t noOfModulesWide, uint8_t noOfModulesHigh){
	byte buffer[noOfModulesHigh * noOfModulesWide][8]{0};
    //byte buffer[(noOfModulesWide + noOfModulesHigh) * ROWWIDTH]{0};
    this -> numOfModulesWide = noOfModulesWide;
    this -> numOfModulesHigh = noOfModulesHigh;
	screenBuffer = buffer;
}
void LedMatrix::init(){
	pinMode(CSPIN, OUTPUT);
	SPI.begin();
  	wipeDisplays();
	updateAll(INTENSITY, 0);
	updateAll(DISPLAY_TEST, 0);
	updateAll(DECODE_MODE, 0);
	updateAll(SCAN_LIMIT, 7);
	updateAll(SHUTDOWN, 1);
}
void LedMatrix::drawPixel(byte x, byte y){
  //draw a pixel in screen memory
	uint8_t temp = 128; //set MSB
	uint8_t module = int(floor(x / 8));
	screenBuffer[(module * ROWWIDTH) + y] = screenBuffer[(module * ROWWIDTH) + y] | (temp >> int(x % 8));
	//screenBuffer[module][y] = screenBuffer[module][y] | (temp >> int(x % 8));
}
void LedMatrix::wipeScreenBuffer(){
  //zero the whole screen buffer
	for (byte i = 0; i < numOfModulesWide; i++){
    for (byte j = 0; j < COLHEIGHT; j++){
      screenBuffer[i][j] = 0;
	  }
	}
}
void LedMatrix::sendScreenBuffer(){
  //updates the matrixes with the screen buffer contents
	for (byte j = 0; j < COLHEIGHT; j++){
  	SPI.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE0));
	digitalWrite(CSPIN, LOW);
		for (byte i = 0; i < numOfModulesWide; i++){
			uint16_t temp = (j + 1) << 8 | screenBuffer[i][j];
    		SPI.transfer16(temp);
		}
	digitalWrite(CSPIN, HIGH);
	SPI.endTransaction();	
	}
}
void LedMatrix::updateAll(uint16_t cmd, uint8_t data){
//used for sending operation codes
	uint16_t x = (cmd << 8) | data;
	SPI.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE0));
	digitalWrite(CSPIN, LOW);
	for (byte i = 0; i < numOfModulesWide; i++){
	  SPI.transfer16(x);
	}
	digitalWrite(CSPIN, HIGH);
	SPI.endTransaction();
}
void LedMatrix::wipeDisplays(){
  for (byte colNumber = 1; colNumber <= COLHEIGHT; colNumber++){
    updateAll(colNumber, 0);
  }
};