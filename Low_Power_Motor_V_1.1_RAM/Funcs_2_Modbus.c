//==================================================================================================================================================
// FILE NAME        : Funcs_2_Modbus.c
// DATE             : 01-Feb-2018
// Project          : VARIABLE FREQUENCY DRIVE FOR COMPRESSOR CONTROL APPLICATIONS
// Project Code     : PEG 124B
// Author           : Rohit V Thomas[ELGI], Manju R[CDAC], Sapna Ravindran[CDAC]
//==========================================================================================================================
// Include Header Files....
//==========================================================================================================================
#include "DSP28234_Device.h"                                // Includes all header files
#include "Variable.h"                                       // Include the Variables
#include "DSP28234_GlobalPrototypes.h"
#include "IQmathLib.h"
//--------------------------------------------------------------------------------------------------------------------------

#define RX_BUFFER_SIZE0 110

#define SUCCESS 1
#define FAILURE 0

#define TRUE    1
#define FALSE   0

#define Slave_ID    17
#define Datlog1_ID  11
#define Datlog2_ID  12
#define Datlog3_ID  13

#define READ_HOLDING_REG    03
#define WRITE_HOLDING_REG   16

// User_Data Structure
#define STRUCT1_START_ADDR  01
#define STRUCT1_NUM_REGS    30

// VFD_Limits Structure
#define STRUCT2_START_ADDR  31
#define STRUCT2_NUM_REGS    10

// VFD_Status Structure
#define STRUCT3_START_ADDR  41
#define STRUCT3_NUM_REGS    02

// VFD_Data Structure
#define STRUCT4_START_ADDR  43
#define STRUCT4_NUM_REGS    26

// PI_Spd Structure
#define STRUCT5_START_ADDR  69
#define STRUCT5_NUM_REGS    07

// PI_Isq Structure
#define STRUCT6_START_ADDR  76
#define STRUCT6_NUM_REGS    07

// PI_Isd Structure
#define STRUCT7_START_ADDR  83
#define STRUCT7_NUM_REGS    07

// Dat_Log1 Structure
#define LOG1_START_ADDR  01
#define LOG1_NUM_REGS    110

// Dat_Log2 Structure
#define LOG2_START_ADDR  01
#define LOG2_NUM_REGS    110

// Dat_Log3 Structure
#define LOG3_START_ADDR  01
#define LOG3_NUM_REGS    110


int16 RxBuffer[RX_BUFFER_SIZE0];
int offset;
static unsigned char RXByteMax=0;
unsigned int StartAdd=0;
unsigned int StartAddOrg=0;
unsigned int offsetOrg=0;
unsigned int NoOfBytes=0;
unsigned int NoOfBytesOrg=0;
unsigned int NoOfBytesPrc=0;
//unsigned int NoOfReg;
float tempfltWrite;
long iqparameter;
int counter = 0;
Uint16 Received_char ;
int n_variables = 8 ;
int datalogger[7];
int z = 0;

void Check_modbus_data(void);
static unsigned char Modbus_Except_FunCode(void);
static unsigned char Modbus_Except_Address(void);
static unsigned char Modbus_ReadMode_Resp(void);
static unsigned char Modbus_WriteMode_Resp(void);
static unsigned char CRC16T(unsigned char slave, unsigned char len,unsigned char check);


static unsigned char const auchCRCHi[]=
{
    0x00u, 0xC1u, 0x81u, 0x40u, 0x01u, 0xC0u, 0x80u, 0x41u, 0x01u, 0xC0u, 0x80u, 0x41u, 0x00u, 0xC1u, 0x81u,
    0x40u, 0x01u, 0xC0u, 0x80u, 0x41u, 0x00u, 0xC1u, 0x81u, 0x40u, 0x00u, 0xC1u, 0x81u, 0x40u, 0x01u, 0xC0u,
    0x80u, 0x41u, 0x01u, 0xC0u, 0x80u, 0x41u, 0x00u, 0xC1u, 0x81u, 0x40u, 0x00u, 0xC1u, 0x81u, 0x40u, 0x01u,
    0xC0u, 0x80u, 0x41u, 0x00u, 0xC1u, 0x81u, 0x40u, 0x01u, 0xC0u, 0x80u, 0x41u, 0x01u, 0xC0u, 0x80u, 0x41u,
    0x00u, 0xC1u, 0x81u, 0x40u, 0x01u, 0xC0u, 0x80u, 0x41u, 0x00u, 0xC1u, 0x81u, 0x40u, 0x00u, 0xC1u, 0x81u,
    0x40u, 0x01u, 0xC0u, 0x80u, 0x41u, 0x00u, 0xC1u, 0x81u, 0x40u, 0x01u, 0xC0u, 0x80u, 0x41u, 0x01u, 0xC0u,
    0x80u, 0x41u, 0x00u, 0xC1u, 0x81u, 0x40u, 0x00u, 0xC1u, 0x81u, 0x40u, 0x01u, 0xC0u, 0x80u, 0x41u, 0x01u,
    0xC0u, 0x80u, 0x41u, 0x00u, 0xC1u, 0x81u, 0x40u, 0x01u, 0xC0u, 0x80u, 0x41u, 0x00u, 0xC1u, 0x81u, 0x40u,
    0x00u, 0xC1u, 0x81u, 0x40u, 0x01u, 0xC0u, 0x80u, 0x41u, 0x01u, 0xC0u, 0x80u, 0x41u, 0x00u, 0xC1u, 0x81u,
    0x40u, 0x00u, 0xC1u, 0x81u, 0x40u, 0x01u, 0xC0u, 0x80u, 0x41u, 0x00u, 0xC1u, 0x81u, 0x40u, 0x01u, 0xC0u,
    0x80u, 0x41u, 0x01u, 0xC0u, 0x80u, 0x41u, 0x00u, 0xC1u, 0x81u, 0x40u, 0x00u, 0xC1u, 0x81u, 0x40u, 0x01u,
    0xC0u, 0x80u, 0x41u, 0x01u, 0xC0u, 0x80u, 0x41u, 0x00u, 0xC1u, 0x81u, 0x40u, 0x01u, 0xC0u, 0x80u, 0x41u,
    0x00u, 0xC1u, 0x81u, 0x40u, 0x00u, 0xC1u, 0x81u, 0x40u, 0x01u, 0xC0u, 0x80u, 0x41u, 0x00u, 0xC1u, 0x81u,
    0x40u, 0x01u, 0xC0u, 0x80u, 0x41u, 0x01u, 0xC0u, 0x80u, 0x41u, 0x00u, 0xC1u, 0x81u, 0x40u, 0x01u, 0xC0u,
    0x80u, 0x41u, 0x00u, 0xC1u, 0x81u, 0x40u, 0x00u, 0xC1u, 0x81u, 0x40u, 0x01u, 0xC0u, 0x80u, 0x41u, 0x01u,
    0xC0u, 0x80u, 0x41u, 0x00u, 0xC1u, 0x81u, 0x40u, 0x00u, 0xC1u, 0x81u, 0x40u, 0x01u, 0xC0u, 0x80u, 0x41u,
    0x00u, 0xC1u, 0x81u, 0x40u, 0x01u, 0xC0u, 0x80u, 0x41u, 0x01u, 0xC0u, 0x80u, 0x41u, 0x00u, 0xC1u, 0x81u,
    0x40u
} ;
/*****************************************************************/
static unsigned char const auchCRCLo[] =
{

    0x00u, 0xC0u, 0xC1u, 0x01u, 0xC3u, 0x03u, 0x02u, 0xC2u, 0xC6u, 0x06u, 0x07u, 0xC7u, 0x05u, 0xC5u, 0xC4u,
    0x04u, 0xCCu, 0x0Cu, 0x0Du, 0xCDu, 0x0Fu, 0xCFu, 0xCEu, 0x0Eu, 0x0Au, 0xCAu, 0xCBu, 0x0Bu, 0xC9u, 0x09u,
    0x08u, 0xC8u, 0xD8u, 0x18u, 0x19u, 0xD9u, 0x1Bu, 0xDBu, 0xDAu, 0x1Au, 0x1Eu, 0xDEu, 0xDFu, 0x1Fu, 0xDDu,
    0x1Du, 0x1Cu, 0xDCu, 0x14u, 0xD4u, 0xD5u, 0x15u, 0xD7u, 0x17u, 0x16u, 0xD6u, 0xD2u, 0x12u, 0x13u, 0xD3u,
    0x11u, 0xD1u, 0xD0u, 0x10u, 0xF0u, 0x30u, 0x31u, 0xF1u, 0x33u, 0xF3u, 0xF2u, 0x32u, 0x36u, 0xF6u, 0xF7u,
    0x37u, 0xF5u, 0x35u, 0x34u, 0xF4u, 0x3Cu, 0xFCu, 0xFDu, 0x3Du, 0xFFu, 0x3Fu, 0x3Eu, 0xFEu, 0xFAu, 0x3Au,
    0x3Bu, 0xFBu, 0x39u, 0xF9u, 0xF8u, 0x38u, 0x28u, 0xE8u, 0xE9u, 0x29u, 0xEBu, 0x2Bu, 0x2Au, 0xEAu, 0xEEu,
    0x2Eu, 0x2Fu, 0xEFu, 0x2Du, 0xEDu, 0xECu, 0x2Cu, 0xE4u, 0x24u, 0x25u, 0xE5u, 0x27u, 0xE7u, 0xE6u, 0x26u,
    0x22u, 0xE2u, 0xE3u, 0x23u, 0xE1u, 0x21u, 0x20u, 0xE0u, 0xA0u, 0x60u, 0x61u, 0xA1u, 0x63u, 0xA3u, 0xA2u,
    0x62u, 0x66u, 0xA6u, 0xA7u, 0x67u, 0xA5u, 0x65u, 0x64u, 0xA4u, 0x6Cu, 0xACu, 0xADu, 0x6Du, 0xAFu, 0x6Fu,
    0x6Eu, 0xAEu, 0xAAu, 0x6Au, 0x6Bu, 0xABu, 0x69u, 0xA9u, 0xA8u, 0x68u, 0x78u, 0xB8u, 0xB9u, 0x79u, 0xBBu,
    0x7Bu, 0x7Au, 0xBAu, 0xBEu, 0x7Eu, 0x7Fu, 0xBFu, 0x7Du, 0xBDu, 0xBCu, 0x7Cu, 0xB4u, 0x74u, 0x75u, 0xB5u,
    0x77u, 0xB7u, 0xB6u, 0x76u, 0x72u, 0xB2u, 0xB3u, 0x73u, 0xB1u, 0x71u, 0x70u, 0xB0u, 0x50u, 0x90u, 0x91u,
    0x51u, 0x93u, 0x53u, 0x52u, 0x92u, 0x96u, 0x56u, 0x57u, 0x97u, 0x55u, 0x95u, 0x94u, 0x54u, 0x9Cu, 0x5Cu,
    0x5Du, 0x9Du, 0x5Fu, 0x9Fu, 0x9Eu, 0x5Eu, 0x5Au, 0x9Au, 0x9Bu, 0x5Bu, 0x99u, 0x59u, 0x58u, 0x98u, 0x88u,
    0x48u, 0x49u, 0x89u, 0x4Bu, 0x8Bu, 0x8Au, 0x4Au, 0x4Eu, 0x8Eu, 0x8Fu, 0x4Fu, 0x8Du, 0x4Du, 0x4Cu, 0x8Cu,
    0x44u, 0x84u, 0x85u, 0x45u, 0x87u, 0x47u, 0x46u, 0x86u, 0x82u, 0x42u, 0x43u, 0x83u, 0x41u, 0x81u, 0x80u,
    0x40u
} ;

//==========================================================================================================================
// Modbus Communication
//==========================================================================================================================
void Modbus()
{
//--------------------------------------------------------------------------------------------------------------------------
// Check Transmit or Receive mode for Modbus Communication
//--------------------------------------------------------------------------------------------------------------------------
    if (VFD_ModbusMode.Mode == Mode_RX)                       // Checking to ensure if RS485 is in the Receive mode
    {
        if (VFD_ModbusMode.Status == 0)
            Check_modbus_data();                              // Enter Receive mode
        if (VFD_ModbusMode.Status >= 10 && VFD_ModbusMode.Status < 12)
            Modbus_ReadMode_Resp();                       // Function for response to Modbus Function Code 3
        if (VFD_ModbusMode.Status >= 20 && VFD_ModbusMode.Status < 22)
            Modbus_WriteMode_Resp();                      // Function for response to Modbus Function Code 16
    }
//--------------------------------------------------------------------------------------------------------------------------
// Sending Response frame based on the Modbus Function code received
//--------------------------------------------------------------------------------------------------------------------------
    if (VFD_ModbusMode.Mode == Mode_TX)                       // Checking to ensure if RS485 is in the Transmit mode
    {
        if (TxCount < RXByteMax)                              // Check for the required byte count
        {
            UartWrite(RxBuffer[TxCount]);                     // Byte write through SCI
            if (VFD_ModbusMode.Status == 30)                  // Check if data byte successfully transmitted
                TxCount++;                                    // Upon successful transmission of previous byte, increment the byte count to transmit the next byte
        }
        else
        {
            if (ScibRegs.SCICTL2.all == 0x00C0)               // When all bytes have been successfully transmitted,
                VFD_ModbusMode.Status = 40;                   // Update Status
        }
    }
//--------------------------------------------------------------------------------------------------------------------------
// Re-initialize variables and flag bits after successful updation of Modbus data
//--------------------------------------------------------------------------------------------------------------------------
    if (VFD_ModbusMode.Status == 40)
    {
        if (TxCount == RXByteMax)                             // Reset the variable count when max count has been reached
        {
            TxCount = 0;
            RxCount = 0;
        }
        VFD_ModbusMode.Index    = 0;                          // Reset the Buffer Array Index
        VFD_ModbusMode.Mode     = 0;                          // Reset the Modbus Status to receive mode
        VFD_ModbusMode.Status   = 0;
    }
}

//==========================================================================================================================
// Check Modubus Data
//==========================================================================================================================
void Check_modbus_data(void)
{
    SCIB_RxTx_En = 0;                                               // Receiver Enable
    delay(20);                                                      // Delay of 3us
    if (ScibRegs.SCIRXST.bit.RXRDY == 1)                            // If Character ready to be read from SCI Receive Buffer
    {
        timingCount         = 0;                                    // Initialize a timing count variable
        RxBuffer[RxCount]   = ScibRegs.SCIRXBUF.all;                // Read data received in SCI Receiver Status Register
        RxCount++;                                                  // Increment the Receive count
    }
    if (ScibRegs.SCIRXST.bit.RXRDY == 0)                            // When Character read is complete
        timingCount++;                                              // Increment a timing count variable

    if ( (RxBuffer[0] == Slave_ID || RxBuffer[0] == Datlog1_ID || RxBuffer[0] == Datlog2_ID || RxBuffer[0] == Datlog3_ID) && (timingCount > 6) )
                                        // Byte 0-->Slave ID match occurs
    {                                   // timing count = 5.4 for inter frame interval of 3.5 characters transmitted (ensures full frame has been received)
                                        // at Baud rate of 19200 for the selected switching frequency of 3kHz
                                        // time to transmit 3.5 characters at Baud rate of 19200 is 3.5*520.083usec = 1820.29usec
                                        // Switching Frequency= 3kHz (333.33usec)
                                        // Timing count = 1820.29/333.33 =5.4608; approximated to 6

//--------------------------------------------------------------------------------------------------------------------------
// Check for Valid Function codes
// Valid function codes are: Read Analog Output Holding Registers (FC=03),Write Multiple Analog Output Holding Registers (FC=16)
//--------------------------------------------------------------------------------------------------------------------------
        if ( !((RxBuffer[1]==READ_HOLDING_REG) || (RxBuffer[1]==WRITE_HOLDING_REG)) && (RxCount>2U) )
            Modbus_Except_FunCode();                                // Exception Handling

//--------------------------------------------------------------------------------------------------------------------------
// Check for Valid Address Range
// Valid Address Range is between 1 and 110
//--------------------------------------------------------------------------------------------------------------------------
        else  if ((1 >((RxBuffer[2]<<8)| (RxBuffer[3]&0xff))||((RxBuffer[2]<<8)| (RxBuffer[3]&0xff))>110) && (RxCount>4U))
            Modbus_Except_Address();                                // Exception Handling

//--------------------------------------------------------------------------------------------------------------------------
//  Response to Read Analog Output Holding Registers (Function codes=03)
//--------------------------------------------------------------------------------------------------------------------------
        else if ((RxBuffer[1]==READ_HOLDING_REG)&&(RxCount==8U))    // Byte1-->Function code 03 matches and Frame count indicates end of frame
        {
            if ((CRC16T(0u,6,1))==1)                                // CRC check matches
            {
                VFD_ModbusMode.Status = 10;                         // Read Mode
                NoOfBytesPrc = 0;
            }
        }

//--------------------------------------------------------------------------------------------------------------------------
//  Response to Write Multiple Analog Output Holding Registers (Function codes=16)
//--------------------------------------------------------------------------------------------------------------------------
        else if ((RxBuffer[1]==WRITE_HOLDING_REG)&&(RxCount>7U))    // Byte1-->Function code 16 matches and and minimum frame count for write function
        {
            if ((CRC16T(0u,RxBuffer[6]+7,1))==1)                    // CRC check matches
            {
                VFD_ModbusMode.Status = 20;                         // Write Mode
                NoOfBytesPrc = 0;
            }
        }
    }

//--------------------------------------------------------------------------------------------------------------------------
//  Invalid Slave ID
//--------------------------------------------------------------------------------------------------------------------------
    else if((RxBuffer[0]!=Slave_ID) && (timingCount>6))             // Re-initialize Array index for invalid Slave ID
    {
       VFD_ModbusMode.Index = 0;
       RxCount  = 0;
       VFD_ModbusMode.Status = 0;
    }
}

//==========================================================================================================================
// Modbus Function Code 16 - Analog Output Holding Registers: Write multiple function
//==========================================================================================================================
static unsigned char Modbus_WriteMode_Resp(void)
{
    StartAdd=((RxBuffer[2]<<8u)|(RxBuffer[3]));                     // RxBuffer[2] holds Higher byte of Data Address of the first register
                                                                    // RxBuffer[3] holds Lower  byte of Data Address of the first register

//    NoOfReg         = ((RxBuffer[4]<<8u)|(RxBuffer[5]));            // RxBuffer[4] holds Higher byte of Number of registers to write
                                                                    // RxBuffer[5] holds Lower  byte of Number of registers to write

    NoOfBytesOrg   = RxBuffer[6];                                   // RxBuffer[6] holds the number of data bytes to follow (No.of registers x 2 bytes each)
//--------------------------------------------------------------------------------------------------------------------------
//  Response to write data bytes in Structure 1 (User_Data Structure)
//--------------------------------------------------------------------------------------------------------------------------
    if ((RxBuffer[0] == Slave_ID) && (STRUCT1_START_ADDR<=StartAdd) && (StartAdd<=STRUCT1_START_ADDR+STRUCT1_NUM_REGS-1))
    {                                                               // Check if the data Address falls in the address range of Structure 1
        if (VFD_ModbusMode.Status == 20)
        {
            StartAddOrg = StartAdd;                                 // Storing the Data Address in a temp variable
            offset      = StartAdd - STRUCT1_START_ADDR;            // Calculating the offset value of the data address from the Start Address of Structure 1
            offsetOrg   = offset;                                   // Storing the offset value in a temp variable
            VFD_ModbusMode.Status = 21;
        }

        if (offset<(((offsetOrg+NoOfBytesOrg) > STRUCT1_NUM_REGS ) ? STRUCT1_NUM_REGS : (offsetOrg+NoOfBytesOrg)))
        {                                                           // Iterate through the number of data bytes to write to Structure 1
            tmpwritevalue1 = RxBuffer[VFD_ModbusMode.Index+7];      // Byte 7 of the Request frame marks MSB to write
            VFD_ModbusMode.Index++;                                 // Increment the index
            tmpwritevalue2 = RxBuffer[VFD_ModbusMode.Index+7];      // Byte 8 of the Request frame marks MSB to write
            VFD_ModbusMode.Index++;                                 // Increment the index
            tmpwritevalue  = (tmpwritevalue1<<8)|(tmpwritevalue2);  // Concatenate Byte 7 & 8 to form the 16 bit value to write to register
            switch (offset)                                         // Handling different data types based on the offset value
            {
               case 0 ... 3:   // => (+1) => 1-4   from Excel
               {
                   tempfltWrite = tmpwritevalue;
                   *(StructAddrfloat[0]+(offset)) = tempfltWrite/64.0;      // Handling float type data with scaling factor 1/64.0
               } break;
               case 4 ... 11:  // => (+1) => 5-12  from Excel
               {
                   tempfltWrite = tmpwritevalue;
                   *(StructAddrfloat[0]+(offset)) = tempfltWrite/4096.0;    // Handling float type data with scaling factor 1/4096
               } break;
               case 12:        // => (+1) => 13    from Excel
               {
                   tempfltWrite = tmpwritevalue;
                   *(StructAddrfloat[0]+(offset)) = tempfltWrite/64.0;      // Handling float type data with scaling factor 1/64.0
               } break;
               case 13 ... 17: // => (+1) => 14-18 from Excel
               {
                   *(StructAddrint[0]+(offset)) = tmpwritevalue;            // Handling integer type data with scaling factor 1
               } break;
               case 18:        // => (+1) => 19    from Excel
               {
                   tempfltWrite = tmpwritevalue;
                   *(StructAddrfloat[0]+(offset)) = tempfltWrite/16384.0;   // Handling float type data with scaling factor 1/16384.0
               } break;
               case 19 ... 22: // => (+1) => 20-23 from Excel
               {
                   *(StructAddrint[0]+(offset)) = tmpwritevalue;            // Handling integer type data with scaling factor 1
               } break;
               case 23 ... 24: // => (+1) => 24-25 from Excel
               {
                   tempfltWrite = tmpwritevalue;
                   *(StructAddrfloat[0]+(offset))= tempfltWrite/64.0;       // Handling float type data with scaling factor 1/64.0
               } break;
               case 25:        // => (+1) => 26    from Excel
               {
                   *(StructAddrint[0]+(offset))= tmpwritevalue;             // Handling integer type data with scaling factor 1
               } break;
               case 26 ... 29: // => (+1) => 27-30 from Excel
               {
                   *(StructAddr[0]+(offset))=tmpwritevalue << (24-12);      // Handling IQ format data ; Converting IQ12 to IQ24
               } break;
            }
        }
        offset++;
        NoOfBytesPrc++;
    }


//--------------------------------------------------------------------------------------------------------------------------
//  Response to write data bytes in Structure  Dat_log.DBUFF1
//--------------------------------------------------------------------------------------------------------------------------
    if ((RxBuffer[0] == Datlog1_ID)&&(LOG1_START_ADDR<=StartAdd)&&(StartAdd<=LOG1_START_ADDR+LOG1_NUM_REGS-1))
    {                                                               // Check if the data Address falls in the address range of Structure 1
        if (VFD_ModbusMode.Status == 20)
        {
            StartAddOrg = StartAdd;                                 // Storing the Data Address in a temp variable
            offset      = StartAdd - LOG1_START_ADDR;               // Calculating the offset value of the data address from the Start Address of Structure 1
            offsetOrg   = offset;                                   // Storing the offset value in a temp variable
            VFD_ModbusMode.Status = 21;
        }

        if (offset<(((offsetOrg+NoOfBytesOrg) > LOG1_NUM_REGS ) ? LOG1_NUM_REGS : (offsetOrg+NoOfBytesOrg)))
        {                                                           // Iterate through the number of data bytes to write to Structure 1
            tmpwritevalue1 = RxBuffer[VFD_ModbusMode.Index+7];      // Byte 7 of the Request frame marks MSB to write
            VFD_ModbusMode.Index++;                                 // Increment the index
            tmpwritevalue2 = RxBuffer[VFD_ModbusMode.Index+7];      // Byte 8 of the Request frame marks MSB to write
            VFD_ModbusMode.Index++;                                 // Increment the index
            tmpwritevalue  = (tmpwritevalue1<<8)|(tmpwritevalue2);  // Concatenate Byte 7 & 8 to form the 16 bit value to write to register
            switch (offset)                                         // Handling different data types based on the offset value
            {
               case 100 ... 108:
               {
                   *(StructAddrint[7]+(offset - 100)) = tmpwritevalue;      // Handling integer type data with scaling factor 1
               } break;
            }
        }
        offset++;
        NoOfBytesPrc++;
    }


//--------------------------------------------------------------------------------------------------------------------------
//  Response to write data bytes in Structure  Dat_log.DBUFF2
//--------------------------------------------------------------------------------------------------------------------------
    if ((RxBuffer[0] == Datlog2_ID)&&(LOG2_START_ADDR<=StartAdd)&&(StartAdd<=LOG2_START_ADDR+LOG2_NUM_REGS-1))
    {                                                               // Check if the data Address falls in the address range of Structure 1
        if (VFD_ModbusMode.Status == 20)
        {
            StartAddOrg = StartAdd;                                 // Storing the Data Address in a temp variable
            offset      = StartAdd - LOG2_START_ADDR;               // Calculating the offset value of the data address from the Start Address of Structure 1
            offsetOrg   = offset;                                   // Storing the offset value in a temp variable
            VFD_ModbusMode.Status = 21;
        }

        if (offset<(((offsetOrg+NoOfBytesOrg) > LOG2_NUM_REGS ) ? LOG2_NUM_REGS : (offsetOrg+NoOfBytesOrg)))
        {                                                           // Iterate through the number of data bytes to write to Structure 1
            tmpwritevalue1 = RxBuffer[VFD_ModbusMode.Index+7];      // Byte 7 of the Request frame marks MSB to write
            VFD_ModbusMode.Index++;                                 // Increment the index
            tmpwritevalue2 = RxBuffer[VFD_ModbusMode.Index+7];      // Byte 8 of the Request frame marks MSB to write
            VFD_ModbusMode.Index++;                                 // Increment the index
            tmpwritevalue  = (tmpwritevalue1<<8)|(tmpwritevalue2);  // Concatenate Byte 7 & 8 to form the 16 bit value to write to register
            switch (offset)                                         // Handling different data types based on the offset value
            {
               case 100 ... 108:
               {
                   *(StructAddrint[7]+(offset - 100)) = tmpwritevalue;      // Handling integer type data with scaling factor 1
               } break;
            }
        }
        offset++;
        NoOfBytesPrc++;
    }


//--------------------------------------------------------------------------------------------------------------------------
//  Response to write data bytes in Structure  Dat_log.DBUFF3
//--------------------------------------------------------------------------------------------------------------------------
    if ((RxBuffer[0] == Datlog3_ID)&&(LOG3_START_ADDR<=StartAdd)&&(StartAdd<=LOG3_START_ADDR+LOG3_NUM_REGS-1))
    {                                                               // Check if the data Address falls in the address range of Structure 1
        if (VFD_ModbusMode.Status == 20)
        {
            StartAddOrg = StartAdd;                                 // Storing the Data Address in a temp variable
            offset      = StartAdd - LOG3_START_ADDR;               // Calculating the offset value of the data address from the Start Address of Structure 1
            offsetOrg   = offset;                                   // Storing the offset value in a temp variable
            VFD_ModbusMode.Status = 21;
        }


        if (offset<(((offsetOrg+NoOfBytesOrg) > LOG3_NUM_REGS ) ? LOG3_NUM_REGS : (offsetOrg+NoOfBytesOrg)))
        {                                                           // Iterate through the number of data bytes to write to Structure 1
            tmpwritevalue1 = RxBuffer[VFD_ModbusMode.Index+7];      // Byte 7 of the Request frame marks MSB to write
            VFD_ModbusMode.Index++;                                 // Increment the index
            tmpwritevalue2 = RxBuffer[VFD_ModbusMode.Index+7];      // Byte 8 of the Request frame marks MSB to write
            VFD_ModbusMode.Index++;                                 // Increment the index
            tmpwritevalue  = (tmpwritevalue1<<8)|(tmpwritevalue2);  // Concatenate Byte 7 & 8 to form the 16 bit value to write to register
            switch (offset)                                         // Handling different data types based on the offset value
            {
               case 100 ... 108:
               {
                   *(StructAddrint[7]+(offset - 100)) = tmpwritevalue;      // Handling integer type data with scaling factor 1
               } break;
            }
        }
        offset++;
        NoOfBytesPrc++;
    }

//--------------------------------------------------------------------------------------------------------------------------
//  CRC Calculation for Response Frame
//--------------------------------------------------------------------------------------------------------------------------
    if (NoOfBytesPrc>NoOfBytesOrg)
    {
        (void)CRC16T(0u,6u,0);      // CRC calculation and setting of CRC bytes to be appended at the end of the response frame
        RXByteMax=8u;               // Set count of data bytes to be sent in response frame
        TxCount=0;                  // Initialize the Transmission byte count
        VFD_ModbusMode.Mode=1;      // Set Mode Status flag to high to enable Response frame Transmission
    }
}

//==========================================================================================================================
// Modbus Function Code 3 - Analog Output Holding Registers: Read function
//==========================================================================================================================
static unsigned char Modbus_ReadMode_Resp(void)
{
    if (VFD_ModbusMode.Status == 10)
    {
        VFD_ModbusMode.Index=0;                                     // Initialise Index to zero
        StartAdd        = ((RxBuffer[2]<<8u)|(RxBuffer[3]));        // RxBuffer[2] holds Higher byte of Data Address of the first register
                                                                    // RxBuffer[3] holds Lower byte of Data Address of the first register

        NoOfBytes       = ((RxBuffer[4]<<8u)|(RxBuffer[5]));        // RxBuffer[4] holds Higher byte of Number of registers requested
                                                                    // RxBuffer[5] holds Lower byte of Number of registers requested
        NoOfBytesOrg    = NoOfBytes;                                // Storing the number of registers requested in a temp variable
    }
//--------------------------------------------------------------------------------------------------------------------------
//  Response to read data bytes in Structure 1 (User_Data Structure)
//--------------------------------------------------------------------------------------------------------------------------
    if ((RxBuffer[0] == Slave_ID)&&(STRUCT1_START_ADDR<=StartAdd)&&(StartAdd<=STRUCT1_START_ADDR+STRUCT1_NUM_REGS-1))
    {                                                               // Check if the data address falls in the address range of Structure 1
        if (VFD_ModbusMode.Status == 10)
        {
            StartAddOrg = StartAdd;                                 // Storing the data address requested in a temp variable
            offset      = StartAdd - STRUCT1_START_ADDR;            // Calculating the offset value of the data address from the Start Address of Structure 1
            offsetOrg   = offset;                                   // Storing the offset value in a temp variable
            VFD_ModbusMode.Status = 11;
        }

        if (offset<(((offsetOrg+NoOfBytes) > STRUCT1_NUM_REGS ) ? STRUCT1_NUM_REGS : (offsetOrg+NoOfBytes)))
        {                                                                   // Iterate through the number of registers to read from Structure 1
            switch (offset)                                                 // Handling different data types based on the offset value
            {
                case 0 ... 3:   // => (+1) => 1-4   from Excel
                {
                    iqparameter =*(StructAddrfloat[0]+(offset)) * 64.0;     // Handling float type data with scaling factor 64.0
                } break;
                case 4 ... 11:  // => (+1) => 5-12  from Excel
                {
                    iqparameter =*(StructAddrfloat[0]+(offset)) * 4096.0;   // Handling float type data with scaling factor 32768.0
                } break;
                case 12:        // => (+1) => 13    from Excel
                {
                    iqparameter =*(StructAddrfloat[0]+(offset)) * 64.0;     // Handling float type data with scaling factor 64.0
                } break;
                case 13 ... 17: // => (+1) => 14-18 from Excel
                {
                    iqparameter =*(StructAddrint[0]+(offset));              // Handling integer type data with scaling factor 1
                } break;
                case 18:        // => (+1) => 19    from Excel
                {
                    iqparameter =*(StructAddrfloat[0]+(offset)) * 16384.0;  // Handling float type data with scaling factor 16384.0
                } break;
                case 19 ... 22: // => (+1) => 20-23 from Excel
                {
                    iqparameter =*(StructAddrint[0]+(offset));              // Handling integer type data with scaling factor 1
                } break;
                case 23 ... 24: // => (+1) => 24-25 from Excel
                {
                    iqparameter =*(StructAddrfloat[0]+(offset)) * 64.0;     // Handling float type data with scaling factor 64.0
                } break;
                case 25:        // => (+1) => 26    from Excel
                {
                    iqparameter =*(StructAddrint[0]+(offset));              // Handling integer type data with scaling factor 1
                } break;
                case 26 ... 29: // => (+1) => 27-30 from Excel
                {
                    iqparameter =*(StructAddr[0]+(offset)) >> (24-12);      // Handling IQ format data ; Converting IQ24 to IQ12
                } break;
            }
            unsigned char tmpiqval1 = (iqparameter>>8) & 0x00FF;    // Forming 16 bit data from 32 bits; extracting MSB from lower 16 bits
            unsigned char tmpiqval2 = iqparameter & 0x00FF;         // Forming 16 bit data from 32 bits; extracting LSB from lower 16 bits
            RxBuffer[VFD_ModbusMode.Index+3] = tmpiqval1;           // Copying MSB to Byte 3 of the array to transmit
            VFD_ModbusMode.Index++;                                 // Increment the index
            RxBuffer[VFD_ModbusMode.Index+3] = tmpiqval2;           // Copying LSB to Byte 4 of the array to transmit
            VFD_ModbusMode.Index++;                                 // Increment the index
        }
        offset++;
        NoOfBytesPrc++;

        if ((StartAdd+offset) > (STRUCT1_START_ADDR+STRUCT1_NUM_REGS) && (StartAdd+NoOfBytes) > (STRUCT1_START_ADDR+STRUCT1_NUM_REGS))
        {                                                           // If the requested register address overlap with Structure 2
            VFD_ModbusMode.Status = 10;
            StartAdd    = STRUCT2_START_ADDR;                       // Assign the Starting Address of Structure 2 as the next Start Address
            NoOfBytes   = NoOfBytes-((STRUCT1_START_ADDR+STRUCT1_NUM_REGS-1)-StartAddOrg+1);
                                                                    // Calculate the number of registers requested from Structure 2
        }
    }

//--------------------------------------------------------------------------------------------------------------------------
//  Response to read data bytes in Structure 2 (VFD_Limits Structure)
//--------------------------------------------------------------------------------------------------------------------------
    if ((RxBuffer[0] == Slave_ID)&&(STRUCT2_START_ADDR<=StartAdd)&&(StartAdd<=STRUCT2_START_ADDR+STRUCT2_NUM_REGS-1))
    {                                                               // Check if the data address falls in the address range of Structure 1
        if (VFD_ModbusMode.Status == 10)
        {
            StartAddOrg     = StartAdd;                             // Storing the data address requested in a temp variable
            offset          = StartAdd - STRUCT2_START_ADDR;        // Calculating the offset value of the data address from the Start Address of Structure 2
            offsetOrg       = offset;                               // Storing the offset value in a temp variable
            VFD_ModbusMode.Status = 11;
        }

        if (offset<(((offsetOrg+NoOfBytes) > STRUCT2_NUM_REGS ) ? STRUCT2_NUM_REGS : (offsetOrg+NoOfBytes)))
        {                                                                   // Iterate through the number of registers to read from Structure 1
            switch (offset)                                                 // Handling different data types can be done based on the offset value
            {
                case 0 ... 6:
                {
                    iqparameter = *(StructAddr[1]+(offset)) >> (24-12);     // Handling IQ format data ; Converting IQ24 to IQ12
                } break;
                case 7 ... 9:
                {
                    iqparameter = *(StructAddr[1]+(offset)) >> (15-8);      // Handling IQ format data ; Converting IQ15 to IQ8
                } break;
            }
            unsigned char tmpiqval1 = (iqparameter>>8) & 0x00FF;    // Forming 16 bit data from 32 bits; extracting MSB from lower 16 bits
            unsigned char tmpiqval2 = iqparameter & 0x00FF;         // Forming 16 bit data from 32 bits; extracting LSB from lower 16 bits
            RxBuffer[VFD_ModbusMode.Index+3] = tmpiqval1;           // Copying MSB to Byte 3 of the array to transmit
            VFD_ModbusMode.Index++;                                 // Increment the index
            RxBuffer[VFD_ModbusMode.Index+3] = tmpiqval2;           // Copying LSB to Byte 4 of the array to transmit
            VFD_ModbusMode.Index++;                                 // Increment the index
        }
        offset++;
        NoOfBytesPrc++;

        if ((StartAdd+NoOfBytes) > (STRUCT2_START_ADDR+STRUCT2_NUM_REGS) && (StartAdd+offset) > (STRUCT2_START_ADDR+STRUCT2_NUM_REGS))
        {                                                           // If the requested register address overlap with Structure 3
            VFD_ModbusMode.Status = 10;
            StartAdd    = STRUCT3_START_ADDR;                       // Assign the Starting Address of Structure 3 as the next Start Address
            NoOfBytes   = NoOfBytes-((STRUCT2_START_ADDR+STRUCT2_NUM_REGS-1)-StartAddOrg+1);
                                                                    // Calculate the number of registers requested from Structure 3
        }
    }

//--------------------------------------------------------------------------------------------------------------------------
//  Response to read data bytes in Structure 3 (VFD_Status Structure)
//--------------------------------------------------------------------------------------------------------------------------
    if ((RxBuffer[0] == Slave_ID)&&(STRUCT3_START_ADDR<=StartAdd)&&(StartAdd<=STRUCT3_START_ADDR+STRUCT3_NUM_REGS-1))
    {                                                               // Check if the data address falls in the address range of Structure 3
        if (VFD_ModbusMode.Status == 10)
        {
            StartAddOrg     = StartAdd;                             // Storing the data address requested in a temp variable
            offset          = StartAdd - STRUCT3_START_ADDR;        // Calculating the offset value of the data address from the Start Address of Structure 3
            offsetOrg       = offset;                               // Storing the offset value in a temp variable
            VFD_ModbusMode.Status = 11;
        }

        if (offset<(((offsetOrg+NoOfBytes) > STRUCT3_NUM_REGS ) ? STRUCT3_NUM_REGS : (offsetOrg+NoOfBytes)))
        {                                                                   // Iterate through the number of registers to read from Structure 1
            switch (offset)                                                 // Handling different data types can be done based on the offset value
            {
                case 0 ... 1:
                {
                    iqparameter =*(StructAddrint[2]+(offset));              // Handling integer type data with scaling factor 1
                } break;

            }
            unsigned char tmpiqval1 = (iqparameter>>8) & 0x00FF;    // Forming 16 bit data from 32 bits; extracting MSB from lower 16 bits
            unsigned char tmpiqval2 = iqparameter & 0x00FF;         // Forming 16 bit data from 32 bits; extracting LSB from lower 16 bits
            RxBuffer[VFD_ModbusMode.Index+3] = tmpiqval1;           // Copying MSB to Byte 3 of the array to transmit
            VFD_ModbusMode.Index++;                                 // Increment the index
            RxBuffer[VFD_ModbusMode.Index+3] = tmpiqval2;           // Copying LSB to Byte 4 of the array to transmit
            VFD_ModbusMode.Index++;                                 // Increment the index
        }
        offset++;
        NoOfBytesPrc++;

        if ((StartAdd+NoOfBytes) > (STRUCT3_START_ADDR+STRUCT3_NUM_REGS) && (StartAdd+offset) > (STRUCT3_START_ADDR+STRUCT3_NUM_REGS))
        {                                                           // If the requested register address overlap with Structure 4
            VFD_ModbusMode.Status = 10;
            StartAdd    = STRUCT4_START_ADDR;                       // Assign the Starting Address of Structure 4 as the next Start Address
            NoOfBytes   = NoOfBytes-((STRUCT3_START_ADDR+STRUCT3_NUM_REGS-1)-StartAddOrg+1);
                                                                    // Calculate the number of registers requested from Structure 4
        }
    }

//--------------------------------------------------------------------------------------------------------------------------
//  Response to read data bytes in Structure 4 (VFD_Data Structure)
//--------------------------------------------------------------------------------------------------------------------------
    if ((RxBuffer[0] == Slave_ID)&&(STRUCT4_START_ADDR<=StartAdd)&&(StartAdd<=STRUCT4_START_ADDR+STRUCT4_NUM_REGS-1))
    {                                                               // Check if the data address falls in the address range of Structure 4
        if (VFD_ModbusMode.Status == 10)
        {
            StartAddOrg = StartAdd;                                 // Storing the data address requested in a temp variable
            offset      = StartAdd - STRUCT4_START_ADDR;            // Calculating the offset value of the data address from the Start Address of Structure 4
            offsetOrg   = offset;                                   // Storing the offset value in a temp variable;
            VFD_ModbusMode.Status = 11;
        }

        if (offset<(((offsetOrg+NoOfBytes) > STRUCT4_NUM_REGS ) ? STRUCT4_NUM_REGS : (offsetOrg+NoOfBytes)))
        {                                                                   // Iterate through the number of registers to read from Structure 1
            switch (offset)                                                 // Handling different data types can be done based on the offset value
            {
                case 0 ... 6:
                {
                    iqparameter =*( StructAddr[3]+(offset)) >> (24-12);     // Handling IQ format data ; Converting IQ24 to IQ12
                } break;
                case 7 ... 9:
                {
                    iqparameter =*( StructAddr[3]+(offset)) >> (15-6);      // Handling IQ format data ; Converting IQ15 to IQ8
                } break;
                case 10 ... 24:
                {
                    iqparameter =*( StructAddr[3]+(offset)) >> (24-12);     // Handling IQ format data ; Converting IQ24 to IQ12
                } break;
                case 25:
                {
                    iqparameter =*(StructAddrint[3]+(offset));              // Handling integer type data with scaling factor 1
                } break;
            }
            unsigned char tmpiqval1 = (iqparameter>>8) & 0x00FF;    // Forming 16 bit data from 32 bits; extracting MSB from lower 16 bits
            unsigned char tmpiqval2 = iqparameter & 0x00FF;         // Forming 16 bit data from 32 bits; extracting LSB from lower 16 bits
            RxBuffer[VFD_ModbusMode.Index+3] = tmpiqval1;           // Copying MSB to Byte 3 of the array to transmit
            VFD_ModbusMode.Index++;                                 // Increment the index
            RxBuffer[VFD_ModbusMode.Index+3] = tmpiqval2;           // Copying LSB to Byte 4 of the array to transmit
            VFD_ModbusMode.Index++;                                 // Increment the index
        }
        offset++;
        NoOfBytesPrc++;

        if ((StartAdd+NoOfBytes) > (STRUCT4_START_ADDR+STRUCT4_NUM_REGS) && (StartAdd+offset) > (STRUCT4_START_ADDR+STRUCT4_NUM_REGS))
        {                                                           // If the requested register address overlap with Structure 5
            VFD_ModbusMode.Status = 10;
            StartAdd    = STRUCT5_START_ADDR;                       // Assign the Starting Address of Structure 5 as the next Start Address
            NoOfBytes   = NoOfBytes-((STRUCT4_START_ADDR+STRUCT4_NUM_REGS-1)-StartAddOrg+1);
                                                                    // Calculate the number of registers requested from Structure 5
        }
    }

//--------------------------------------------------------------------------------------------------------------------------
//  Response to read data bytes in Structure 5 (PI_Spd Structure)
//--------------------------------------------------------------------------------------------------------------------------
    if ((RxBuffer[0] == Slave_ID)&&(STRUCT5_START_ADDR<=StartAdd)&&(StartAdd<=STRUCT5_START_ADDR+STRUCT5_NUM_REGS-1))
    {                                                               // Check if the data address falls in the address range of Structure 5
        if (VFD_ModbusMode.Status == 10)
        {
            StartAddOrg = StartAdd;                                 // Storing the data address requested in a temp variable
            offset      = StartAdd - STRUCT5_START_ADDR;            // Calculating the offset value of the data address from the Start Address of Structure 5
            offsetOrg   = offset;                                   // Storing the offset value in a temp variable;
            VFD_ModbusMode.Status = 11;
        }

        if (offset<(((offsetOrg+NoOfBytes) > STRUCT5_NUM_REGS ) ? STRUCT5_NUM_REGS : (offsetOrg+NoOfBytes)))
        {                                                                   // Iterate through the number of registers to read from Structure 1
            switch (offset)                                                 // Handling different data types can be done based on the offset value
            {
                case 0 ... 6:
                {
                    iqparameter =*( StructAddr[4]+(offset)) >> (24-12);     // Handling IQ format data ; Converting IQ24 to IQ12
                } break;
            }
            unsigned char tmpiqval1 = (iqparameter>>8) & 0x00FF;    // Forming 16 bit data from 32 bits; extracting MSB from lower 16 bits
            unsigned char tmpiqval2 = iqparameter & 0x00FF;         // Forming 16 bit data from 32 bits; extracting LSB from lower 16 bits
            RxBuffer[VFD_ModbusMode.Index+3] = tmpiqval1;           // Copying MSB to Byte 3 of the array to transmit
            VFD_ModbusMode.Index++;                                 // Increment the index
            RxBuffer[VFD_ModbusMode.Index+3] = tmpiqval2;           // Copying LSB to Byte 4 of the array to transmit
            VFD_ModbusMode.Index++;                                 // Increment the index
        }
        offset++;
        NoOfBytesPrc++;

        if ((StartAdd+NoOfBytes) > (STRUCT5_START_ADDR+STRUCT5_NUM_REGS) && (StartAdd+offset) > (STRUCT5_START_ADDR+STRUCT5_NUM_REGS))
        {                                                           // If the requested register address overlap with Structure 6
            VFD_ModbusMode.Status = 10;
            StartAdd=STRUCT6_START_ADDR;                            // Assign the Starting Address of Structure 6 as the next Start Address
            NoOfBytes=NoOfBytes-((STRUCT5_START_ADDR+STRUCT5_NUM_REGS-1)-StartAddOrg+1);
                                                                    // Calculate the number of registers requested from Structure 6
        }
    }

//--------------------------------------------------------------------------------------------------------------------------
//  Response to read data bytes in Structure 6 (PI_Isq Structure)
//--------------------------------------------------------------------------------------------------------------------------
    if ((RxBuffer[0] == Slave_ID)&&(STRUCT6_START_ADDR<=StartAdd)&&(StartAdd<=STRUCT6_START_ADDR+STRUCT6_NUM_REGS-1))
    {                                                               // Check if the data address falls in the address range of Structure 6
        if (VFD_ModbusMode.Status == 10)
        {
            StartAddOrg     = StartAdd;                             // Storing the data address requested in a temp variable
            offset          = StartAdd - STRUCT6_START_ADDR;        // Calculating the offset value of the data address from the Start Address of Structure 6
            offsetOrg       = offset;                               // Storing the offset value in a temp variable;
            VFD_ModbusMode.Status = 11;
        }

        if (offset<(((offsetOrg+NoOfBytes) > STRUCT6_NUM_REGS ) ? STRUCT6_NUM_REGS : (offsetOrg+NoOfBytes)))
        {                                                                   // Iterate through the number of registers to read from Structure 1
            switch (offset)                                                 // Handling different data types can be done based on the offset value
            {
                case 0 ... 6:
                {
                    iqparameter =*( StructAddr[5]+(offset)) >> (24-12);     // Handling IQ format data ; Converting IQ24 to IQ12
                } break;
            }
            unsigned char tmpiqval1 = (iqparameter>>8) & 0x00FF;    // Forming 16 bit data from 32 bits; extracting MSB from lower 16 bits
                unsigned char tmpiqval2 = iqparameter & 0x00FF;     // Forming 16 bit data from 32 bits; extracting LSB from lower 16 bits
                RxBuffer[VFD_ModbusMode.Index+3] = tmpiqval1;       // Copying MSB to Byte 3 of the array to transmit
            VFD_ModbusMode.Index++;                                 // Increment the index
            RxBuffer[VFD_ModbusMode.Index+3] = tmpiqval2;           // Copying LSB to Byte 4 of the array to transmit
            VFD_ModbusMode.Index++;                                 // Increment the index
        }
        offset++;
        NoOfBytesPrc++;

        if ((StartAdd+NoOfBytes) > (STRUCT6_START_ADDR+STRUCT6_NUM_REGS) && (StartAdd+offset) > (STRUCT6_START_ADDR+STRUCT6_NUM_REGS))                 // If the requested register address overlap with Structure 7
        {
            VFD_ModbusMode.Status = 10;
            StartAdd    = STRUCT7_START_ADDR;                       // Assign the Starting Address of Structure 7 as the next Start Address
            NoOfBytes   = NoOfBytes-((STRUCT6_START_ADDR+STRUCT6_NUM_REGS-1)-StartAddOrg+1);
                                                                    // Calculate the number of registers requested from Structure 7
        }
    }

//--------------------------------------------------------------------------------------------------------------------------
//  Response to read data bytes in Structure 7 (PI_Isd Structure)
//--------------------------------------------------------------------------------------------------------------------------
    if ((RxBuffer[0] == Slave_ID)&&(STRUCT7_START_ADDR<=StartAdd)&&(StartAdd<=STRUCT7_START_ADDR+STRUCT7_NUM_REGS-1))
    {                                                               // Check if the data address falls in the address range of Structure 7
        if (VFD_ModbusMode.Status == 10)
        {
            StartAddOrg     = StartAdd;                             // Storing the data address requested in a temp variable
            offset          = StartAdd - STRUCT7_START_ADDR;        // Calculating the offset value of the data address from the Start Address of Structure 7
            offsetOrg       = offset;                               // Storing the offset value in a temp variable;
            VFD_ModbusMode.Status = 11;
        }

        if (offset<(((offsetOrg+NoOfBytes) > STRUCT7_NUM_REGS ) ? STRUCT7_NUM_REGS : (offsetOrg+NoOfBytes)))
        {                                                                   // Iterate through the number of registers to read from Structure 1
            switch (offset)                                                 // Handling different data types can be done based on the offset value
            {
                case 0 ... 6:
                {
                    iqparameter =*( StructAddr[6]+(offset)) >> (24-12);     // Handling IQ format data ; Converting IQ24 to IQ12
                } break;
            }
            unsigned char tmpiqval1 = (iqparameter>>8) & 0x00FF;    // Forming 16 bit data from 32 bits; extracting MSB from lower 16 bits
            unsigned char tmpiqval2 = iqparameter & 0x00FF;         // Forming 16 bit data from 32 bits; extracting LSB from lower 16 bits
            RxBuffer[VFD_ModbusMode.Index+3] = tmpiqval1;           // Copying MSB to Byte 3 of the array to transmit
            VFD_ModbusMode.Index++;                                 // Increment the index
            RxBuffer[VFD_ModbusMode.Index+3] = tmpiqval2;           // Copying LSB to Byte 4 of the array to transmit
            VFD_ModbusMode.Index++;                                 // Increment the index
        }
        offset++;
        NoOfBytesPrc++;
    }

//--------------------------------------------------------------------------------------------------------------------------
//  Response to read data bytes in Structure  Dat_log.DBUFF1
//--------------------------------------------------------------------------------------------------------------------------
    if ((RxBuffer[0] == Datlog1_ID)&&(LOG1_START_ADDR<=StartAdd)&&(StartAdd<=LOG1_START_ADDR+LOG1_NUM_REGS-1))
    {                                                               // Check if the data address falls in the address range of Structure 7
        if (VFD_ModbusMode.Status == 10)
        {
            StartAddOrg     = StartAdd;                             // Storing the data address requested in a temp variable
            offset          = StartAdd - LOG1_START_ADDR;           // Calculating the offset value of the data address from the Start Address of Structure 7
            offsetOrg       = offset;                               // Storing the offset value in a temp variable;
            VFD_ModbusMode.Status = 11;
        }

        if (offset<(((offsetOrg+NoOfBytes) > LOG1_NUM_REGS ) ? LOG1_NUM_REGS : (offsetOrg+NoOfBytes)))
        {                                                                   // Iterate through the number of registers to read from Structure 1
            switch (offset)                                                 // Handling different data types can be done based on the offset value
            {
                case 0 ... 99:
                {
                    iqparameter =*(StructAddrint[4]+(offset));              // Handling integer type data with scaling factor 1
                } break;

                case 100 ... 109:
                {
                    iqparameter =*(StructAddrint[7]+(offset - 100));        // Handling integer type data with scaling factor 1
                } break;
            }
            unsigned char tmpiqval1 = (iqparameter>>8) & 0x00FF;    // Forming 16 bit data from 32 bits; extracting MSB from lower 16 bits
            unsigned char tmpiqval2 = iqparameter & 0x00FF;         // Forming 16 bit data from 32 bits; extracting LSB from lower 16 bits
            RxBuffer[VFD_ModbusMode.Index+3] = tmpiqval1;           // Copying MSB to Byte 3 of the array to transmit
            VFD_ModbusMode.Index++;                                 // Increment the index
            RxBuffer[VFD_ModbusMode.Index+3] = tmpiqval2;           // Copying LSB to Byte 4 of the array to transmit
            VFD_ModbusMode.Index++;                                 // Increment the index
        }
        offset++;
        NoOfBytesPrc++;
    }

//--------------------------------------------------------------------------------------------------------------------------
//  Response to read data bytes in Structure  Dat_log.DBUFF2
//--------------------------------------------------------------------------------------------------------------------------
    if ((RxBuffer[0] == Datlog2_ID)&&(LOG2_START_ADDR<=StartAdd)&&(StartAdd<=LOG2_START_ADDR+LOG2_NUM_REGS-1))
    {                                                               // Check if the data address falls in the address range of Structure 7
        if (VFD_ModbusMode.Status == 10)
        {
            StartAddOrg     = StartAdd;                             // Storing the data address requested in a temp variable
            offset          = StartAdd - LOG2_START_ADDR;           // Calculating the offset value of the data address from the Start Address of Structure 7
            offsetOrg       = offset;                               // Storing the offset value in a temp variable;
            VFD_ModbusMode.Status = 11;
        }

        if (offset<(((offsetOrg+NoOfBytes) > LOG2_NUM_REGS ) ? LOG2_NUM_REGS : (offsetOrg+NoOfBytes)))
        {                                                                   // Iterate through the number of registers to read from Structure 1
            switch (offset)                                                 // Handling different data types can be done based on the offset value
            {
                case 0 ... 99:
                {
                    iqparameter =*(StructAddrint[5]+(offset));              // Handling integer type data with scaling factor 1
                } break;

                case 100 ... 109:
                {
                    iqparameter =*(StructAddrint[7]+(offset - 100));        // Handling integer type data with scaling factor 1
                } break;
            }
            unsigned char tmpiqval1 = (iqparameter>>8) & 0x00FF;    // Forming 16 bit data from 32 bits; extracting MSB from lower 16 bits
            unsigned char tmpiqval2 = iqparameter & 0x00FF;         // Forming 16 bit data from 32 bits; extracting LSB from lower 16 bits
            RxBuffer[VFD_ModbusMode.Index+3] = tmpiqval1;           // Copying MSB to Byte 3 of the array to transmit
            VFD_ModbusMode.Index++;                                 // Increment the index
            RxBuffer[VFD_ModbusMode.Index+3] = tmpiqval2;           // Copying LSB to Byte 4 of the array to transmit
            VFD_ModbusMode.Index++;                                 // Increment the index
        }
        offset++;
        NoOfBytesPrc++;
    }

//--------------------------------------------------------------------------------------------------------------------------
//  Response to read data bytes in Structure  Dat_log.DBUFF3
//--------------------------------------------------------------------------------------------------------------------------
    if ((RxBuffer[0] == Datlog3_ID)&&(LOG3_START_ADDR<=StartAdd)&&(StartAdd<=LOG3_START_ADDR+LOG3_NUM_REGS-1))
    {                                                               // Check if the data address falls in the address range of Structure 7
        if (VFD_ModbusMode.Status == 10)
        {
            StartAddOrg     = StartAdd;                             // Storing the data address requested in a temp variable
            offset          = StartAdd - LOG3_START_ADDR;           // Calculating the offset value of the data address from the Start Address of Structure 7
            offsetOrg       = offset;                               // Storing the offset value in a temp variable;
            VFD_ModbusMode.Status = 11;
        }

        if (offset<(((offsetOrg+NoOfBytes) > LOG3_NUM_REGS ) ? LOG3_NUM_REGS : (offsetOrg+NoOfBytes)))
        {                                                                   // Iterate through the number of registers to read from Structure 1
            switch (offset)                                                 // Handling different data types can be done based on the offset value
            {
                case 0 ... 99:
                {
                    iqparameter =*(StructAddrint[6]+(offset));              // Handling integer type data with scaling factor 1
                } break;

                case 100 ... 109:
                {
                    iqparameter =*(StructAddrint[7]+(offset - 100));        // Handling integer type data with scaling factor 1
                } break;
            }
            unsigned char tmpiqval1 = (iqparameter>>8) & 0x00FF;    // Forming 16 bit data from 32 bits; extracting MSB from lower 16 bits
            unsigned char tmpiqval2 = iqparameter & 0x00FF;         // Forming 16 bit data from 32 bits; extracting LSB from lower 16 bits
            RxBuffer[VFD_ModbusMode.Index+3] = tmpiqval1;           // Copying MSB to Byte 3 of the array to transmit
            VFD_ModbusMode.Index++;                                 // Increment the index
            RxBuffer[VFD_ModbusMode.Index+3] = tmpiqval2;           // Copying LSB to Byte 4 of the array to transmit
            VFD_ModbusMode.Index++;                                 // Increment the index
        }
        offset++;
        NoOfBytesPrc++;
    }

//--------------------------------------------------------------------------------------------------------------------------
//  CRC Calculation for Response Frame
//--------------------------------------------------------------------------------------------------------------------------
    if (NoOfBytesPrc>NoOfBytesOrg)
    {
        RxBuffer[2] = NoOfBytesOrg*2;
        (void)CRC16T(0u,RxBuffer[2]+3u,0);  // Calculation of CRC
        RXByteMax   = 5u+(RxBuffer[2]);     // Set count of data bytes to be sent in response frame
        VFD_ModbusMode.Mode = 1;            // Set Mode Status flag to high to enable Response frame Transmission
        NoOfBytesPrc = 0;
    }
}

//==========================================================================================================================
// Modbus Exception for Invalid Function code
//==========================================================================================================================
static unsigned char Modbus_Except_FunCode(void)
{
    RxBuffer[1] = RxBuffer[1]|0x80; // Function code in exception response
    RxBuffer[2] = 1;                // Exception code for Invalid function code
    (void)CRC16T(0u,3u,0u);         // Calculation of CRC
    RXByteMax   = 5u;               // Set the max number of bytes to transmit
    VFD_ModbusMode.Index  = 0;      // Initialize index to zero
    VFD_ModbusMode.Mode   = 1;      // Set Mode Status flag to high to enable Response frame Transmission

    return SUCCESS;
}

//==========================================================================================================================
// Modbus Exception For Invalid Address Range
//==========================================================================================================================
static unsigned char Modbus_Except_Address(void)
{
    RxBuffer[1] = RxBuffer[1]|0x80; // Function code in exception response
    RxBuffer[2] = 2;                // Exception code for Invalid address
    (void)CRC16T(0u,3u,0u);         // Calculation of CRC
    RXByteMax   = 5u;               // Set the max number of bytes to transmit
    VFD_ModbusMode.Index  = 0;      // Initialize index to zero
    VFD_ModbusMode.Mode   = 1;      // Set Mode Status flag to high to enable Response frame Transmission

    return SUCCESS;
}

//==========================================================================================================================
// Modbus CRC check and validation function
//==========================================================================================================================
static unsigned char CRC16T(unsigned char slave,unsigned char len,unsigned char check)
{
    unsigned char i=0u,j;
    unsigned char uIndex;                                                   // will index into CRC lookup table
    unsigned char uchCRCHi = 0xFFu ;                                        // high byte of CRC initialized
    unsigned char uchCRCLo = 0xFFu ;                                        // low byte of CRC initialized
    j=len;
    while (j!=0u)                                                           // pass through message buffer
    {
        uIndex      = uchCRCHi ^ RxBuffer[i];  ;
        uchCRCHi    = uchCRCLo ^ auchCRCHi[uIndex] ;
        uchCRCLo    = auchCRCLo[uIndex] ;
        i++;
        j--;
    }
    if (check==1u)
        {
            if ((RxBuffer[len] == uchCRCHi) && (RxBuffer[len+1u] == uchCRCLo ))
                return 1;
            else
                return 0;
        }
        else
        {
            RxBuffer[len+1] = uchCRCLo;
            RxBuffer[len]   = uchCRCHi;
            return 1;
        }
}

//==========================================================================================================================
// Modbus frame transmission through SCI
//==========================================================================================================================
void UartWrite(int data)
{
 //   if(ScibRegs.SCIRXST.bit.RXRDY == 0)
    {
  //      SCIB_RxTx_En = 1;                                    // Transmit Enable
  //      delay(10);                                            // Delay of 3us
    }

    while( ScibRegs.SCICTL2.bit.TXRDY ==1 && z<n_variables)
    {
       ScibRegs.SCITXBUF = datalogger[z] ; // data;
       z ++ ;
    }
    if(z == n_variables)
    z = 0;
      // VFD_ModbusMode.Status = 30;                      // Status set to check if data successfully transmitted


    //  Received_char = ScibRegs.SCIRXBUF.all ;

  //      else
  //          VFD_ModbusMode.Status = 80;
}
//==========================================================================================================================

void Uart_Serial()
{
 //   if(ScibRegs.SCIRXST.bit.RXRDY == 0)
    {
  //      SCIB_RxTx_En = 1;                                    // Transmit Enable
  //      delay(10);                                            // Delay of 3us
    }

    while( ScibRegs.SCICTL2.bit.TXRDY ==1 && z < n_variables)
    {
       ScibRegs.SCITXBUF = datalogger[z] ; // data;
       z ++ ;
    }
    if(z == n_variables)
    z = 0;
      // VFD_ModbusMode.Status = 30;                      // Status set to check if data successfully transmitted


    //  Received_char = ScibRegs.SCIRXBUF.all ;

  //      else
  //          VFD_ModbusMode.Status = 80;
}
void Update_datalogger()
{
    datalogger[0]= 1;
    datalogger[1]= 2;

    datalogger[2]= 3 ;
    datalogger[3]= 4 ;

    datalogger[4]= 5 ;
    datalogger[5]= 6 ;

    datalogger[6]= 7 ;
    datalogger[7]= 8;
}
