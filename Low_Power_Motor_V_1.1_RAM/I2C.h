// TI File $Revision: /main/2 $
// Checkin $Date: March 22, 2007   10:40:22 $
//###########################################################################
//
// FILE:   DSP2833x_I2c.h
//
// TITLE:  DSP2833x Enhanced Quadrature Encoder Pulse Module
//         Register Bit Definitions.
//
//###########################################################################
// $TI Release: 2833x/2823x Header Files V1.32 $
// $Release Date: June 28, 2010 $
//###########################################################################

#ifndef I2C_H_
#define I2C_H_

void I2cWrite(unsigned char);
unsigned char I2cRead(unsigned char);
void I2cInit(void);
void I2cStart(void);
void I2cStop(void);
#endif /*I2C_H_*/
