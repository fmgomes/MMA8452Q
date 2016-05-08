/******************************************************************************
SparkFun_MMA8452Q.cpp
SparkFun_MMA8452Q Library Source File
Jim Lindblom @ SparkFun Electronics
Original Creation Date: June 3, 2014
https://github.com/sparkfun/MMA8452_Accelerometer

This file implements all functions of the MMA8452Q class. Functions here range
from higher level stuff, like reading/writing MMA8452Q registers to low-level,
hardware I2C reads and writes.

Development environment specifics:
	IDE: Arduino 1.0.5
	Hardware Platform: Arduino Uno

	**Updated for Arduino 1.6.4 5/2015**
	
This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/

#include "MMA8452Q.h"
#include <Arduino.h>
#include <Wire.h>

// CONSTRUCTUR
//   This function, called when you initialize the class will simply write the
//   supplied address into a private variable for future use.
//   The variable addr should be either 0x1C or 0x1D, depending on which voltage
//   the SA0 pin is tied to (GND or 3.3V respectively).
MMA8452Q::MMA8452Q(byte addr)
{
	address = addr; // Store address into private variable
}

// INITIALIZATION
//	This function initializes the MMA8452Q. 
byte MMA8452Q::init()
{

	// AN4072, pag 17, TAP and Double-TAP Detection
	writeRegister(CTRL_REG1, 0x10); 			//200 Hz, Standby Mode
//	writeRegister(PULSE_CFG, 0x3F); 			//Enable X, Y, Z Single Pulse and X, Y and Z Double Pulse with DPA = 0 no double pulse abort
	writeRegister(PULSE_CFG, 0x7F); 			//Enable X, Y, Z Single Pulse and X, Y and Z Double Pulse with DPA = 0 no double pulse abort, Flags latched
	writeRegister(PULSE_THSX, 0x20); 			//Set X Threshold to 2g
//	writeRegister(PULSE_THSY, 0x20); 			//Set Y Threshold to 2g
	writeRegister(PULSE_THSY, 0x10); 			//Set Y Threshold to 2g
	writeRegister(PULSE_THSZ, 0x40); 			//Set Z Threshold to 4g
	writeRegister(PULSE_TMLT, 0x18); 			//60 ms
	writeRegister(PULSE_LTCY, 0x28); 			//200 ms
	writeRegister(PULSE_WIND, 0x3C); 			//300 ms
	writeRegister(CTRL_REG3, 0x02); 			//Interrupt polarity active high (FG)
	writeRegister(CTRL_REG4, 0x08); 			//Enable Pulse Interrupt Block in System CTRL_REG4
	writeRegister(CTRL_REG5, 0x08); 			//Route Pulse Interrupt Block to INT1 hardware Pin CTRL_REG5
	uint8_t CTRL_REG1_Data = readRegister(CTRL_REG1); //Read out the contents of the register
	CTRL_REG1_Data |= 0x01; 					//Change the value in the register to Active Mode.
	writeRegister(CTRL_REG1, CTRL_REG1_Data); 	//Write in the updated value to put the device in Active Mode

	
/*
// Approach proposed by David Diaz Marin to my question at NXP - https://community.freescale.com/thread/384908
	writeRegister(CTRL_REG1, 0x0C);         // ODR = 400Hz, Reduced noise, Standby mode
	writeRegister(XYZ_DATA_CFG, 0x00);      // +/-2g range -> 1g = 16384/4 = 4096 counts
	writeRegister(CTRL_REG2, 0x02);         // High Resolution mode
//	writeRegister(PULSE_CFG, 0x15);         //Enable X, Y and Z Single Pulse
	writeRegister(PULSE_CFG, 0x7F); 		//Enable X, Y, Z Single Pulse and X, Y and Z Double Pulse with DPA = 0 no double pulse abort
//	writeRegister(PULSE_THSX, 0x20);        //Set X Threshold to 2.016g
	writeRegister(PULSE_THSX, 0x28);        //Set X Threshold to 2.5g
	writeRegister(PULSE_THSY, 0x20);        //Set Y Threshold to 2.016g
	writeRegister(PULSE_THSZ, 0x2A);        //Set Z Threshold to 2.646g
	writeRegister(PULSE_TMLT, 0x28);        //Set Time Limit for Tap Detection to 25 ms
	writeRegister(PULSE_LTCY, 0x28);        //Set Latency Time to 50 ms
	writeRegister(CTRL_REG3, 0x02); 		//Interrupt polarity active high (FG)
	writeRegister(CTRL_REG4, 0x08);         //Pulse detection interrupt enabled
	writeRegister(CTRL_REG5, 0x08);         //Route INT1 to system interrupt
	uint8_t CTRL_REG1_Data = readRegister(CTRL_REG1);   //Active Mode
	CTRL_REG1_Data |= 0x01;
	writeRegister(CTRL_REG1, CTRL_REG1_Data);
*/
	return 1;
}

// READ ACCELERATION DATA
//  This function will read the acceleration values from the MMA8452Q. After
//	reading, it will update two triplets of variables:
//		* int's x, y, and z will store the signed 12-bit values read out
//		  of the acceleromter.
//		* floats cx, cy, and cz will store the calculated acceleration from
//		  those 12-bit values. These variables are in units of g's.
void MMA8452Q::read()
{
	byte rawData[6];  // x/y/z accel register data stored here

	readRegisters(OUT_X_MSB, rawData, 6);  // Read the six raw data registers into data array
	
	x = ((short)(rawData[0]<<8 | rawData[1])) >> 4;
	y = ((short)(rawData[2]<<8 | rawData[3])) >> 4;
	z = ((short)(rawData[4]<<8 | rawData[5])) >> 4;

}

// READ TAP STATUS
//	This function returns any taps read by the MMA8452Q. If the function 
//	returns no new taps were detected. Otherwise the function will return the
//	lower 7 bits of the PULSE_SRC register.
byte MMA8452Q::readTap()
{
	byte tapStat = readRegister(PULSE_SRC);
	if (tapStat & 0x80) // Read EA bit to check if a interrupt was generated
	{
		return tapStat & 0x7F;
	}
	else
		return 0;
}


// WRITE A SINGLE REGISTER
// 	Write a single byte of data to a register in the MMA8452Q.
void MMA8452Q::writeRegister(MMA8452Q_Register reg, byte data)
{
	writeRegisters(reg, &data, 1);
}

// WRITE MULTIPLE REGISTERS
//	Write an array of "len" bytes ("buffer"), starting at register "reg", and
//	auto-incrmenting to the next.
void MMA8452Q::writeRegisters(MMA8452Q_Register reg, byte *buffer, byte len)
{
	Wire.beginTransmission(address);
	Wire.write(reg);
	for (int x = 0; x < len; x++)
		Wire.write(buffer[x]);
	Wire.endTransmission(); //Stop transmitting
}

// READ A SINGLE REGISTER
//	Read a byte from the MMA8452Q register "reg".
byte MMA8452Q::readRegister(MMA8452Q_Register reg)
{
	Wire.beginTransmission(address);
	Wire.write(reg);
	Wire.endTransmission(false); //endTransmission but keep the connection active

	Wire.requestFrom(address, (byte) 1); //Ask for 1 byte, once done, bus is released by default

	while(!Wire.available()) ; //Wait for the data to come back

	return Wire.read(); //Return this one byte
}

// READ MULTIPLE REGISTERS
//	Read "len" bytes from the MMA8452Q, starting at register "reg". Bytes are stored
//	in "buffer" on exit.
void MMA8452Q::readRegisters(MMA8452Q_Register reg, byte *buffer, byte len)
{
	Wire.beginTransmission(address);
	Wire.write(reg);
	Wire.endTransmission(false); //endTransmission but keep the connection active

	Wire.requestFrom(address, len); //Ask for bytes, once done, bus is released by default

	while(Wire.available() < len); //Hang out until we get the # of bytes we expect

	for(int x = 0 ; x < len ; x++)
		buffer[x] = Wire.read();    
}