//==================================================================================================================================================
// FILE NAME        : I2C.c
// DATE             : 25-Jan-2017
// Project          : VARIABLE FREQUENCY DRIVE FOR COMPRESSOR CONTROL APPLICATIONS
// Project Code     : PEG 124B
// Author           : Manju R[CDAC],Razila K R[CDAC] & Rinu J Vijyan [CDAC]
//==================================================================================================================================================
// Include Header Files....
//==================================================================================================================================================
#include "DSP28234_Device.h"                                // Includes all header files
#include "Variable.h"                                       // Include the Variables
#include "DSP28234_GlobalPrototypes.h"
#include "IQmathLib.h"

#define SDA GpioDataRegs.GPBDAT.bit.GPIO32
#define SCL GpioDataRegs.GPBDAT.bit.GPIO33

unsigned int I2cCount = 0; 
unsigned int I2cBuff = 0;
int i2c_data = 0;
int SerialDataBuffer[128];
int RxByteCount,OneTimeFlag;


void I2cDelay(){
    int i;
    for(i=0; i<50; i++);
    }

void Delay1(){
    asm(" NOP");
    asm(" NOP");
    asm(" NOP");
    asm(" NOP");
    asm(" NOP");
    asm(" NOP");
    asm(" NOP");
    asm(" NOP");
    }

void SdaLow(){
    GpioCtrlRegs.GPBDIR.bit.GPIO32 = 1;
    GpioDataRegs.GPBDAT.bit.GPIO32 = 0;
    }

void SdaHigh(){
    GpioCtrlRegs.GPBDIR.bit.GPIO32 = 0;
    }


void SclLow(){
    GpioCtrlRegs.GPBDIR.bit.GPIO33 = 1;
    GpioDataRegs.GPBDAT.bit.GPIO33 = 0;
    }

void SclHigh(){
    GpioCtrlRegs.GPBDIR.bit.GPIO33 = 0;
    }


void I2cStart(){
    SdaHigh();
    SclHigh();
    Delay1();
    SdaLow();
    I2cDelay();
    SclLow();
    }

void I2cStop(){
    SclHigh();
    I2cDelay();
    SdaHigh();
    }

void I2cWrite(unsigned char data){
    I2cBuff = 0;
    for(I2cCount = 0; I2cCount<8; I2cCount++){      // Do until full byte is shifted out
        I2cBuff = data & 0x80;                      // Get MSB
        if(I2cBuff > 0)
           {                            // If high
            SdaHigh();                              // if '1' SDA high
            }
        else{
            SdaLow();                               // else if '0' SDA low
            }
        SclHigh();                                  // SCL high
        I2cDelay();                                 // ON time
        SclLow();                                   // SCL low
        I2cDelay();
        data = data << 1;                           // Eject MSB
        }
    SdaHigh();                                      // SDA high
    SclHigh();                                      // SCL high (9th clock)
    I2cDelay();                                     // ON time
    if(SDA == 0){                                   // If ACK
        asm(" NOP");
        }
    else{                                           // If NACK
        asm(" NOP");
        }
    SclLow();                                       // SCL Low
    SdaLow();                                       // SDA Low
    }

unsigned char I2cRead(unsigned char ack){
    SdaHigh();
    SclLow();
    I2cBuff = 0;
    for(I2cCount = 0; I2cCount<8; I2cCount++){
        I2cBuff = I2cBuff << 1;
        SclHigh();
        I2cDelay();
        if(SDA == 1){
            I2cBuff = I2cBuff | 0x01;
            }
        else{
            I2cBuff = I2cBuff & 0xFE;
            }
        SclLow();
        I2cDelay();
        }
    if(ack == 1){
        SdaHigh();
        }
    else{
        SdaLow();
        }
    SclHigh();
    I2cDelay();
    SclLow();
    I2cDelay();
    SdaLow();
    return I2cBuff;
}

void I2cTransitionDelay(){
    unsigned int i5;
    for(i5=0; i5<500; i5++){
        I2cDelay();
        }
}

void I2C_WriteRead()
{
//--------------------------------------------------------------------------------------------------------------------------------------------------
        asm(" EALLOW");
        EPROM_Write_Protect = 0;            // EPROM Write protect enable

        if(OneTimeFlag==0)
        {
            I2cStart();                     // I2c Start
            I2cWrite(0xA0);                 // Slave Address
            I2cWrite(0x00);                 // Higher Address
            I2cWrite(0x00);                 // Lower Address
            for(RxByteCount=0; RxByteCount<128; RxByteCount++){
                i2c_data = i2c_data +1;
                I2cWrite(i2c_data);
                }
            I2cStop();
            OneTimeFlag = 1;
        }
        I2cTransitionDelay();
//--------------------------------------------------------------------------------------------------------------------------------------------------
// EPROM read data
//--------------------------------------------------------------------------------------------------------------------------------------------------
        if(OneTimeFlag==1)
        {
            I2cStart();                     // I2c Start
            I2cWrite(0xA0);                 // Slave address
            I2cWrite(0x00);                 // Higher Address
            I2cWrite(0x00);                 // Lower Address
            I2cStart();                     // I2c Repeated Start
            I2cWrite(0xA1);                 // Slave Address(Read Mode)
            for(RxByteCount=0; RxByteCount<128; RxByteCount++){
                SerialDataBuffer[RxByteCount] = I2cRead(0);
                }
            SerialDataBuffer[RxByteCount] = I2cRead(1);
            I2cStop();
        }
//--------------------------------------------------------------------------------------------------------------------------------------------------
}


