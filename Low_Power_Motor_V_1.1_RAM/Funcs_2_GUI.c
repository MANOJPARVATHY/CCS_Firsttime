//==================================================================================================================================================
// FILE NAME        : Funcs_2_GUI.c
// DATE             : 01-Feb-2018
// Project          : VARIABLE FREQUENCY DRIVE FOR COMPRESSOR CONTROL APPLICATIONS
// Project Code     : PEG 124B
// Author           : Manju R[CDAC],Sapna Ravindran[CDAC]
//==================================================================================================================================================
// Include Header Files....
//==================================================================================================================================================
#include "DSP28234_Device.h"								// Includes all header files
#include "Variable.h"										// Include Variables
#include "DSP28234_GlobalPrototypes.h"
#include "IQmathLib.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
//--------------------------------------------------------------------------------------------------------------------------
//==========================================================================================================================
// GUI for RS485 Communication Function Calls
//==========================================================================================================================
void GUIHandler()
{
//--------------------------------------------------------------------------------------------------------------------------
// Check Transmit or Receive mode
//--------------------------------------------------------------------------------------------------------------------------
	if ( (VFD_GUIConfigData.RxParametersMode == ModeOFF) && (VFD_GUIConfigData.BulkTransferMode == ModeOFF) &&
			(VFD_GUIConfigData.OnlineUpdateMode == ModeOFF) && (VFD_GUIConfigData.SamplingMode == ModeOFF) )
	                    // Ensuring RS485 not in Transmit mode
		GUI_SCI_Rx();   // Enter Receive mode
//--------------------------------------------------------------------------------------------------------------------------
// Check for command to fetch controller parameters from EEPROM to GUI
//--------------------------------------------------------------------------------------------------------------------------
	if (VFD_GUIConfigData.RxParametersMode == ModeON)
	{
		if (ByteCount < VFD_GUIConfigData.TxDataByteLength)             // Check for required byte count for transmitting
		{
			UartWrite(VFD_GUIConfigData.SerialDataBuffer[ByteCount]);   // Byte write through SCI

			if (VFD_GUIConfigData.DataSent == SUCCESS)                  // If data transmission successful
			    ByteCount++;                                            // ByteCount++ to transmit next byte
		}
		else
		{
			if (ScibRegs.SCICTL2.all == 0x00C0)                         // When all bytes have been transmitted,
			    VFD_GUIConfigData.Tx_status = 1;						// Set Transmit status to high
		}
	}
//--------------------------------------------------------------------------------------------------------------------------
// Check for command to update online status and acknowledgement bits from DSP to GUI
//--------------------------------------------------------------------------------------------------------------------------
	if (VFD_GUIConfigData.OnlineUpdateMode == ModeON)
	{
		if (OnlineDataCount < VFD_GUIConfigData.OnlineDataLength)       // Check for required byte count for transmitting
		{
			UartWrite(VFD_GUIConfigData.OnlineDataBuffer[OnlineDataCount]);
			                                                            // Byte write through SCI
			if (VFD_GUIConfigData.DataSent == SUCCESS)                  // If data transmission successful
			    OnlineDataCount++;										// OnlineDataCount++ to transmit next byte
		}
		else
		{
			if (ScibRegs.SCICTL2.all == 0x00C0)						    // When all bytes have been transmitted,
			    VFD_GUIConfigData.Tx_status = 1;						// set Transmit status to high
		}
	}
//--------------------------------------------------------------------------------------------------------------------------
// Re-initialise variables and flag bits after successful updation of Controller parameters and online status bits updation
//--------------------------------------------------------------------------------------------------------------------------
	if (VFD_GUIConfigData.Tx_status == SUCCESS)
	{
		VFD_GUIConfigData.RxParametersMode = 0; // Reset Controller Parameters Receive mode flag
		VFD_GUIConfigData.OnlineUpdateMode = 0;	// Reset Onine Status Updation flag
		ByteCount=0;							// Reset byte count (for Contoller Parameters receive mode) to zero
		OnlineDataCount=0;				        // Reset byte count (for Online Status Updation mode) to zero
		VFD_GUIConfigData.Tx_status        = 0;	// Reset Transmission status flag
		VFD_GUIConfigData.TxDataByteLength = 0;	// Reset Transmission byte count
	}
//--------------------------------------------------------------------------------------------------------------------------
// Data Sampling function for Buffered mode Graphical data plotting
//--------------------------------------------------------------------------------------------------------------------------
	DataSamplingController();
//--------------------------------------------------------------------------------------------------------------------------
// Check for command to transfer buffered data from DSP to GUI for Buffered mode Graphical data plotting
//--------------------------------------------------------------------------------------------------------------------------
	if (VFD_GUIConfigData.BulkTransferMode == ModeON)
	{
		if (DataCount < (VFD_GUIConfigData.noc*VFD_GUIConfigData.Memory))   // Check byte count < Req no of samples
		{
			VFD_GUIConfigData.dataBuffer = *AddrPtr;						// Read signal sample value from Ext RAM

			if (VFD_GUIConfigData.BulkByteCount == 0)						// Sending MSB of a data sample
			{
				VFD_GUIConfigData.dataBufferMSB = VFD_GUIConfigData.dataBuffer>>8;  // Extract MSB from signal sample value
				UartWrite(VFD_GUIConfigData.dataBufferMSB);           				// Send higher byte (dataBufferMSB)

				if (VFD_GUIConfigData.DataSent == SUCCESS)							// If data transmission successful
				{
					VFD_GUIConfigData.CheckSumBulkTrans ^= VFD_GUIConfigData.dataBufferMSB;   // Accumulate checksum
					VFD_GUIConfigData.BulkByteCount++;                              // Proceed to send lower byte
				}
			}
			else if (VFD_GUIConfigData.BulkByteCount > 0)							// Sending LSB of data sample
			{
				VFD_GUIConfigData.dataBufferLSB = VFD_GUIConfigData.dataBuffer & 0x00FF; // Extract LSB from signal sample value
				UartWrite(VFD_GUIConfigData.dataBufferLSB);     						 // Send lower byte

				if (VFD_GUIConfigData.DataSent == SUCCESS)	                             // If data transmission successful
				{
					VFD_GUIConfigData.CheckSumBulkTrans ^=VFD_GUIConfigData.dataBufferLSB;    // Accumulate checksum
					VFD_GUIConfigData.BulkByteCount = 0;                            // Prepare to send higher byte
					DataCount++;                                    				// Proceed to next data sample
					AddrPtr++;														// Increment pointer to next memory location
				}
			}
		}
		else if (DataCount == VFD_GUIConfigData.noc*VFD_GUIConfigData.Memory)   // If Check byte count has reached Req no of samples
		{
			AddrPtr =  SegmentStrtAddr; 						            // Re-initialise pointer to starting address

			if (VFD_GUIConfigData.alreadysent == 0)
			    UartWrite(VFD_GUIConfigData.CheckSumBulkTrans);

			if (VFD_GUIConfigData.DataSent == SUCCESS)						// If data transmission successful
			{
				VFD_GUIConfigData.alreadysent=1;							// Flag set to ensure successful checksum transmission

				if (ScibRegs.SCICTL2.all == 0x00C0)						    // When all bytes have been transmitted,
				{
					VFD_GUIConfigData.BulkTransferMode  = 0;				// Reset Bulk Transfer data mode
					VFD_GUIConfigData.alreadysent       = 0;				// Reset flag after successful checksum transmission
					VFD_GUIConfigData.BulkByteCount     = 0;				// Reset byte count
					DataCount                           = 0;				// Reset data sample count
				}
			}
		}
	}
}

//----------------------------------------------------------------------------------------------------------------------
// Channel ID and Signal Selector
//----------------------------------------------------------------------------------------------------------------------
signed int GetVal(unsigned int Id)
{
    int16 RetVal = 0;
    // Signal selection based on Signal ID sent from GUI to DSP
    switch(Id)
	{
        case 0x01 : RetVal = VFD_Ch1Data;                       // Assign Signal ID 1 to Channel
        break;
        case 0x02 : RetVal = VFD_Ch2Data;						// Assign Signal ID 2 to Channel
        break;
        case 0x03 : RetVal = VFD_Ch3Data;                       // Assign Signal ID 3 to Channel
        break;
        case 0x04 : RetVal = VFD_Ch4Data;						// Assign Signal ID 4 to Channel
        break;
        case 0x05 : RetVal = VFD_GUIConfigData.Isa;				// Assign Signal ID 5 to Channel
        break;
        case 0x06 : RetVal = VFD_GUIConfigData.Isb;				// Assign Signal ID 6 to Channel
        break;
        case 0x07 : RetVal = VFD_GUIConfigData.Isc;				// Assign Signal ID 7 to Channel
        break;
        case 0x08 : RetVal = _IQtoF(MOT_RC.RampOut)* 50;	    // Assign Signal ID 8 to Channel
        break;
        case 0x09 : RetVal = _IQtoF(VFD_Data.SineEpsilon)* 50;	// Assign Signal ID 9 to Channel
        break;
        case 0x0A : RetVal = _IQtoF(VFD_Data.CosEpsilon)* 50;	// Assign Signal ID 10 to Channel
        break;
        case 0x0B : RetVal = _IQtoF(MOT_FSE.ThetaFlux)* 50;	    // Assign Signal ID 11 to Channel
        break;
        case 0x0C : RetVal = _IQtoF(VFD_Data.SineTheta)* 50;	// Assign Signal ID 12 to Channel
        break;
        case 0x0D : RetVal = _IQtoF(VFD_Data.CosTheta)* 50;	    // Assign Signal ID 13 to Channel
        break;
    }
    return RetVal;
}

//----------------------------------------------------------------------------------------------------------------------
// GUI Receive function
//----------------------------------------------------------------------------------------------------------------------
void GUI_SCI_Rx()
{
    SCIB_RxTx_En = 0;                                         				// Receiver Enable
    delay(3);                                                               // delay of 3us

    if ((ScibRegs.SCIRXST.bit.RXRDY == 1))		               				// If Character ready to be read from SCI Receive Buffer
    {
        VFD_GUIConfigData.Rx_data[RxCnt] = ScibRegs.SCIRXBUF.all;  	// Read data received in SCI Receiver Status Register
        RxCnt++;												   			// Increment data received byte count

        if (RxCnt>RxCntLimit)											   	// Check if data received count exceeds limit
        	RxCnt = 0;											     		// Re-initialise data received byte count
    //--------------------------------------------------------------------------------------------------------------------------
    // GUI Command Receive handler
    //--------------------------------------------------------------------------------------------------------------------------
     	//--------------------------------------------------------------------------------------------------------------------------
    	// RECEIVE Controller Parameters Command from GUI
    	//--------------------------------------------------------------------------------------------------------------------------
        if ((RxByteCountGet == 0) && (VFD_GUIConfigData.Rx_data[0] == RxControllerParamDSPtoGUI))      // RECEIVE button handler; Command byte 0xB8
			RxByteCountGet++;                                               // Increment RECEIVE button frame byte count

		else if ((RxByteCountGet == 1) && (VFD_GUIConfigData.Rx_data[0] == RxControllerParamDSPtoGUI)) // RECEIVE button handler; Data length higher byte
		{                                                       			// If DLH field
			VFD_GUIConfigData.RxBtnDataByteLength = VFD_GUIConfigData.Rx_data[1];      // Copy data
			VFD_GUIConfigData.RxBtnDataByteLength <<= 8;                    // Make room for lower byte
			RxByteCountGet++;                                               // Increment RECEIVE button frame byte count
		}
		else if ((RxByteCountGet == 2) && (VFD_GUIConfigData.Rx_data[0] == RxControllerParamDSPtoGUI)) // RECEIVE button handler; Data length lower byte
		{                                                                   // If DLL field
			VFD_GUIConfigData.RxBtnDataByteLength += VFD_GUIConfigData.Rx_data[2];     // Concatenate lower byte
			RxByteCountGet  = 0;											// Clear RECEIVE button frame byte count
			RxCnt           = 0;                                            // Clear data received byte count
			RS485CommandHandler(RxControllerParamDSPtoGUI);					// RECEIVE button response function call
		}
     	//--------------------------------------------------------------------------------------------------------------------------
    	// Online Status Update COmmand from GUI
    	//--------------------------------------------------------------------------------------------------------------------------
        if ((RxByteCount == 0) && (VFD_GUIConfigData.Rx_data[0] == OnlineStatusUpate)) // Online status handler; Command byte 0xB7
        {
        	RS485CommandHandler(OnlineStatusUpate);										    // Online status response function call
            RxCnt = 0;														                // Clear data received byte count
        }
     	//--------------------------------------------------------------------------------------------------------------------------
    	// Command for Data Sampling in Buffered mode Graphical Data Plotting from GUI
    	//--------------------------------------------------------------------------------------------------------------------------
        if ((RxByteCount == 0) && (VFD_GUIConfigData.Rx_data[0] == DataSampling))      // Buffered mode Signal Sampling handler; Command byte 0xC9
        {
        	RS485CommandHandler(DataSampling);										        // Buffered mode Signal Sampling response function call
            RxCnt = 0;														                // Clear data received byte count
        }
     	//--------------------------------------------------------------------------------------------------------------------------
    	// Command for Bulk data transfer for Buffered mode Graphical Data Plotting from GUI
    	//--------------------------------------------------------------------------------------------------------------------------
        if ((RxByteCount == 0) && (VFD_GUIConfigData.Rx_data[0] == BulkDataTransfer))  // Buffered Bulk data transfer mode handler; Command byte 0xCA
        {
        	RS485CommandHandler(BulkDataTransfer);										    // Buffered Bulk data transfer mode response function call
            RxCnt = 0;														                // Clear data received byte count
        }
     	//--------------------------------------------------------------------------------------------------------------------------
    	// Common frame structure for two Commands:
        // Command I: SEND Controller Parameters Command from GUI
        // Command II: Trajectory time and memory requirements update Command for Buffered mode from GUI
    	//--------------------------------------------------------------------------------------------------------------------------
        else if ((RxByteCount == 0) && (VFD_GUIConfigData.Rx_data[0] == 0xFA))	        // SEND button handler or Trajectory time updation handler
        {                                                      				                // If start of frame (SOF byte-0xFA)
            VFD_GUIConfigData.Sof = VFD_GUIConfigData.Rx_data[0];                      // Copy received SOF byte locally
            RxByteCount++;                                                                  // Increment data received byte count
        }
        else if ((RxByteCount == 1) && (VFD_GUIConfigData.Rx_data[0] == 0xFA))     // SEND button handler or Trajectory time updation handler; Data length higher byte
        {                                                      			            // If DLH field
            VFD_GUIConfigData.DataByteLength = VFD_GUIConfigData.Rx_data[1];   // Copy data
            VFD_GUIConfigData.DataByteLength <<= 8;                                 // Make room for lower byte
            RxByteCount++;                                                          // Increment data received byte count
        }
        else if ((RxByteCount == 2) && (VFD_GUIConfigData.Rx_data[0] == 0xFA))     // SEND button handler or Trajectory time updation handler; Data length lower byte
        {                                                      				        // If DLL field
            VFD_GUIConfigData.DataByteLength += VFD_GUIConfigData.Rx_data[2];  // Concatenate lower byte
            RxByteCount++;                                                          // Increment data received byte count
        }
        else if ((RxByteCount == 3) && (VFD_GUIConfigData.Rx_data[0] == 0xFA))     // SEND button handler or Trajectory time updation handler; Command or Packet ID
        {                                                       		            // If Packet ID
            VFD_GUIConfigData.SerialDataBuffer1[RxByteCount-3] = VFD_GUIConfigData.Rx_data[RxByteCount]; // Copy received Command byte locally
            VFD_GUIConfigData.CheckSum ^= VFD_GUIConfigData.SerialDataBuffer1[RxByteCount-3];   // Calculate checksum
            RxByteCount++;                                                          // Increment data received byte count
        }
        else if ((RxByteCount > 3) && (RxByteCount<(VFD_GUIConfigData.DataByteLength+3)))   // SEND button handler or Trajectory time updation handler; Data field
        {                                                      				        // If data field
            VFD_GUIConfigData.SerialDataBuffer1[RxByteCount-3] = VFD_GUIConfigData.Rx_data[RxByteCount]; // Copy to data buffer array
            VFD_GUIConfigData.CheckSum ^= VFD_GUIConfigData.SerialDataBuffer1[RxByteCount-3];   // Calculate checksum
            RxByteCount++;                                                          // Increment data received byte count
        }
        else if ((RxByteCount == VFD_GUIConfigData.DataByteLength+3) && (VFD_GUIConfigData.Rx_data[RxByteCount] == VFD_GUIConfigData.CheckSum)) // If checksum calculated matches checksum byte received
        {
            VFD_GUIConfigData.RxCmd = VFD_GUIConfigData.SerialDataBuffer1[0];       // Copy command byte
            RxByteCount++;												            // Increment data received byte count
        }
        else if ((RxByteCount == VFD_GUIConfigData.DataByteLength+4) && (VFD_GUIConfigData.Rx_data[RxByteCount] == 0xEF)) // SEND button handler or Trajectory time updation handler
        {																	        // If end of frame (EOF byte-0xEF)
            VFD_GUIConfigData.Eof = VFD_GUIConfigData.Rx_data[RxByteCount];    // Copy received EOF byte locally
            RxByteCount = 0;												        // Clear data received byte count
            RxCnt       = 0;														// Clear data received byte count
            VFD_GUIConfigData.CheckSum = 0;											// Clear Checksum
            RS485CommandHandler(VFD_GUIConfigData.RxCmd);                           // SEND button handler or Trajectory time updation handler response function call
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------
// GUI Command Response Handler
//----------------------------------------------------------------------------------------------------------------------
void RS485CommandHandler(unsigned int Cmd)
{
 	//--------------------------------------------------------------------------------------------------------------------------
	// SEND Controller Parameters Command Response to GUI
	//--------------------------------------------------------------------------------------------------------------------------
	if  ((Cmd == SendControllerParamGUItoDSP) && (VFD_Status.bits.StartFlag != 1))   // SEND button response
    {
    	for (count = 4; count < (VFD_GUIConfigData.DataByteLength+3); count++)       // Data field
    	{
    	  VFD_GUIConfigData.data_convert_to_float[count-4] = VFD_GUIConfigData.Rx_data[count]; // Copy data from received array to a data to convert to float array
        }
    	char *array[25];						                                // Declaring Char array to hold delimiter splitted string
    	int ArrayIndex = 0, ResultArrayIndex = 0;							        // Declare local array index variables
    	array[ArrayIndex] = strtok(VFD_GUIConfigData.data_convert_to_float,"|");// Break String with Delimiter or String token "|";(Delimiter assigned is "|")

    	while (array[ArrayIndex] != NULL)
    	{
    	   array[++ArrayIndex] = strtok(NULL,"|");                               // Split string by "|" delimiter and copy data to array
    	   VFD_GUIConfigData.Rxd_data[ResultArrayIndex]=(float)atof(array[ResultArrayIndex]); // Convert string argument array to a floating-point number
    	   ResultArrayIndex++;												     // Increment array index
    	}
    	WriteReadChkFromEeprom(VFD_GUIConfigData.DataByteLength-1);              // Function call for writing Received data to EEPROM
    	UpdateToLocalParameters();
    }
 	//--------------------------------------------------------------------------------------------------------------------------
	// RECEIVE Controller Parameters Command Response to GUI
	//--------------------------------------------------------------------------------------------------------------------------
    if ((Cmd==RxControllerParamDSPtoGUI) && (VFD_Status.bits.StartFlag != 1))   // RECEIVE button response
    {
        VFD_GUIConfigData.CheckSumGet    = 0;								    // Initialize Checksum for RECEIVE button resoponse to zero
        VFD_GUIConfigData.CheckSumGet   ^= RxControllerParamDSPtoGUI;			// Checksum calculation; EXOR Command byte
        ReadFromEeprom(VFD_GUIConfigData.RxBtnDataByteLength-1);		        // Read data bytes from EEPROM corresponding to Data length send from GUI
        ByteCount   = 0;											            // Initialize Byte count to zero
        VFD_GUIConfigData.TxDataBuffer[0]       = RxControllerParamDSPtoGUI;	// Prepare a Response Transmit Byte buffer array; Copy Command byte
        VFD_GUIConfigData.TxDataByteLength      = VFD_GUIConfigData.RxBtnDataByteLength+1;  // Send Datalength of response frame
        VFD_GUIConfigData.SerialDataBuffer[0]   = VFD_GUIConfigData.TxDataBuffer[0];        // Copy transmit buffer array

        for (BuffLength = 1; BuffLength < VFD_GUIConfigData.RxBtnDataByteLength; BuffLength++)
        {
           VFD_GUIConfigData.SerialDataBuffer[BuffLength]    = VFD_GUIConfigData.TxDataBuffer[BuffLength];  // Copy data read from EEPROM to buffer array
           VFD_GUIConfigData.CheckSumGet                    ^= VFD_GUIConfigData.TxDataBuffer[BuffLength];  // Calculate Checksum by EXORing data bytes
        }
        VFD_GUIConfigData.SerialDataBuffer[VFD_GUIConfigData.RxBtnDataByteLength] = VFD_GUIConfigData.CheckSumGet;    // Copy calculated Checksum value to buffer array to transmit as response
        VFD_GUIConfigData.DspResponse       = 0xA2;                             // Assign a DSP Response frame byte- 0xA2
        VFD_GUIConfigData.RxParametersMode  = 1;						        // Set flag to fetch controller parameters from DSP to GUI
    }
 	//--------------------------------------------------------------------------------------------------------------------------
	// Trajectory time and memory requirements Command Response to GUI
	//--------------------------------------------------------------------------------------------------------------------------
    if ( Cmd == TrajTimeUpdate)											        // Trajectory time parameter updation
    {
        VFD_GUIConfigData.SamplingTimeFactor  = VFD_GUIConfigData.SerialDataBuffer1[1]; // Sample time count
        VFD_GUIConfigData.noc   = VFD_GUIConfigData.SerialDataBuffer1[2];               // Number of enabled channels
        VFD_GUIConfigData.mem1  = VFD_GUIConfigData.SerialDataBuffer1[3];			    // Sample byte count-byte1
        VFD_GUIConfigData.mem2  = VFD_GUIConfigData.SerialDataBuffer1[4];               // Sample byte count-byte2
        VFD_GUIConfigData.mem3  = VFD_GUIConfigData.SerialDataBuffer1[5];               // Sample byte count-byte3
        VFD_GUIConfigData.ChId1 = VFD_GUIConfigData.SerialDataBuffer1[6];               // Id of channel1
        VFD_GUIConfigData.ChId2 = VFD_GUIConfigData.SerialDataBuffer1[7];               // Id of channel2
        VFD_GUIConfigData.ChId3 = VFD_GUIConfigData.SerialDataBuffer1[8];               // Id of channel3
        VFD_GUIConfigData.ChId4 = VFD_GUIConfigData.SerialDataBuffer1[9];               // Id of channel4
        VFD_GUIConfigData.Memory= (VFD_GUIConfigData.mem1 << 16) +
                                  (VFD_GUIConfigData.mem2 << 8) +
                                  (VFD_GUIConfigData.mem3);                             // Total number of samples requested
        VFD_GUIConfigData.DspResponse = 0xA8;                                           // Assign a DSP Response frame byte- 0xA8
    }
 	//--------------------------------------------------------------------------------------------------------------------------
	// Response for Data Sampling in Buffered mode Graphical Data Plotting to GUI
	//--------------------------------------------------------------------------------------------------------------------------
    if ( Cmd == DataSampling)				    // Signal Sampling
    {
        VFD_GUIConfigData.DspResponse   = 0xA9; // Assign DSP Response frame byte = 0xA9
        VFD_GUIConfigData.SamplingMode  = 1;	// Set flag to start Signal sampling
    }
 	//--------------------------------------------------------------------------------------------------------------------------
	// Response for Bulk data transfer for Buffered mode Graphical Data Plotting to GUI
	//--------------------------------------------------------------------------------------------------------------------------
    if( Cmd == BulkDataTransfer)				    // Bulk Data Transfer
    {
    	VFD_GUIConfigData.TxDataBuffer[0]   = Cmd;  // Prepare Response Transmit Byte buffer array; Copy Command byte
        DataCount   = 0;							// Initialize Data sample count to zero
        VFD_GUIConfigData.BulkByteCount     = 0;	// Initialize Data byte count to zero
        VFD_GUIConfigData.CheckSumBulkTrans = 0;	// Initialize Checksum of Bulk data transfer to zero
        UartWrite(VFD_GUIConfigData.TxDataBuffer[0]);   // Byte write through SCI; Command write
        if(VFD_GUIConfigData.DataSent == SUCCESS)       // If data transmission successful
        {
            VFD_GUIConfigData.CheckSumBulkTrans ^= VFD_GUIConfigData.TxDataBuffer[0];   // Checksum calculation
            VFD_GUIConfigData.BulkTransferMode   = 1;							        // Set flag to start Bulk transfer data mode for graphical data plotting
        }
    }
 	//--------------------------------------------------------------------------------------------------------------------------
	// Response for Online Data Status Update to GUI
	//--------------------------------------------------------------------------------------------------------------------------
    if ( Cmd == OnlineStatusUpate)									// Online Status updation
    {
        VFD_GUIConfigData.OnlineDataBuffer[0] = OnlineStatusUpate;  // Prepare a Response Transmit Byte buffer array; Copy Command byte
        VFD_GUIConfigData.CheckSumOnlineStatus =0;                  // Initialise Checksum of Online status updation mode to zero
        GUI_StatusUpdate();                                         // Function to update VFD status in GUI

        VFD_GUIConfigData.OnlineDataBuffer[1] = VFD_Status.all& 0x000000FF; // LSB
        VFD_GUIConfigData.OnlineDataBuffer[2] = VFD_Status.all>>8 ;			// MSB
//---------------------------------------------------------------------------------------------------------------------------------------
        VFD_Ch1Data = _IQtoF(VFD_Data.SpeedRmp) * 120 * MOT_Data.Wb / PIx2 / MOT_Data.Poles;  // Speed Ref = p.u. * Base Speed
        VFD_Ch2Data = _IQtoF(VFD_Data.SpeedAct) * 120 * MOT_Data.Wb / PIx2 / MOT_Data.Poles;  // Speed Act = p.u. * Base Speed
        VFD_Ch3Data = _IQtoF(VFD_Data.Isq) * MOT_Data.Ib;           // Torque = p.u. * Base current
        VFD_Ch4Data = VFD_Ch1Data - VFD_Ch2Data;					// Error = Speed Ref - Speed Act
//---------------------------------------------------------------------------------------------------------------------------------------
        VFD_Status1Data =  VFD_Ch1Data;									// Speed Ref
        VFD_Status2Data =  VFD_Ch2Data;									// Speed Act
        VFD_Status3Data =  VFD_Ch3Data;									// Torque Current (Iq)
        VFD_Status4Data =  _IQtoF(VFD_Data.VdcFilt) * VFD_Data.VdcRef;  // DC Bus Voltage = p.u. * Base DC Voltage
//---------------------------------------------------------------------------------------------------------------------------------------
        VFD_GUIConfigData.OnlineDataBuffer[3]   = VFD_Ch1Data>>8;           // Copy higher byte of Channel 1 signal
        VFD_GUIConfigData.OnlineDataBuffer[4]   = VFD_Ch1Data & 0x00FF;		// Copy lower byte of Channel 1 signal

        VFD_GUIConfigData.OnlineDataBuffer[5]   = VFD_Ch2Data>>8;			// Copy higher byte of Channel 2 signal
        VFD_GUIConfigData.OnlineDataBuffer[6]   = VFD_Ch2Data & 0x00FF;		// Copy lower byte of Channel 2 signal

        VFD_GUIConfigData.OnlineDataBuffer[7]   = VFD_Ch3Data>>8;			// Copy higher byte of Channel3 signal
        VFD_GUIConfigData.OnlineDataBuffer[8]   = VFD_Ch3Data & 0x00FF;		// Copy lower byte of Channel 3 signal

        VFD_GUIConfigData.OnlineDataBuffer[9]   = VFD_Ch4Data>>8;			// Copy higher byte of Channel 4 signal
        VFD_GUIConfigData.OnlineDataBuffer[10]  = VFD_Ch4Data & 0x00FF;		// Copy lower byte of Channel 4 signal

        VFD_GUIConfigData.OnlineDataBuffer[11]  = VFD_Status1Data>>8;		// Copy higher byte of Speed Command
        VFD_GUIConfigData.OnlineDataBuffer[12]  = VFD_Status1Data & 0x00FF; // Copy lower byte of Speed Command

        VFD_GUIConfigData.OnlineDataBuffer[13]  = VFD_Status2Data>>8;		// Copy higher byte of Actual Speed
        VFD_GUIConfigData.OnlineDataBuffer[14]  = VFD_Status2Data & 0x00FF;	// Copy lower byte of Actual Speed

        VFD_GUIConfigData.OnlineDataBuffer[15]  = VFD_Status3Data>>8;		// Copy higher byte of R Phase Current
        VFD_GUIConfigData.OnlineDataBuffer[16]  = VFD_Status3Data & 0x00FF;	// Copy lower byte of R Phase Current

        VFD_GUIConfigData.OnlineDataBuffer[17]  = VFD_Status3Data>>8;		// Copy higher byte of Y Phase Current
        VFD_GUIConfigData.OnlineDataBuffer[18]  = VFD_Status3Data & 0x00FF;	// Copy lower byte of Y Phase Current

        VFD_GUIConfigData.OnlineDataBuffer[19]  = VFD_Status3Data>>8;		// Copy higher byte of B Phase Current
        VFD_GUIConfigData.OnlineDataBuffer[20]  = VFD_Status3Data & 0x00FF;	// Copy lower byte of B Phase Current

        VFD_GUIConfigData.OnlineDataBuffer[21]  = VFD_Status4Data>>8;		// Copy higher byte of DC Bus Voltage
        VFD_GUIConfigData.OnlineDataBuffer[22]  = VFD_Status4Data & 0x00FF;	// Copy lower byte of DC Bus Voltage

        VFD_GUIConfigData.OnlineDataBuffer[23]  = VFD_GUIConfigData.DspResponse;  // Copy DSP Response byte
//---------------------------------------------------------------------------------------------------------------------------------------
        if (VFD_GUIConfigData.DspResponse > 0)								// If DSP Response is non-zero, initialize to zero
        {
            VFD_GUIConfigData.DspResponse = 0;
        }
        for (BuffLength = 0; BuffLength < VFD_GUIConfigData.OnlineDataLength-1; BuffLength++)
        {
            VFD_GUIConfigData.CheckSumOnlineStatus ^= VFD_GUIConfigData.OnlineDataBuffer[BuffLength];	// Checksum calculation for Online Status updation
        }
        VFD_GUIConfigData.OnlineDataBuffer[VFD_GUIConfigData.OnlineDataLength-1] = VFD_GUIConfigData.CheckSumOnlineStatus;    // Copy Checksum for Online Status Updation
        OnlineDataCount = 0;													// Initialize array index of Online Status data array
        VFD_GUIConfigData.OnlineUpdateMode = 1;									// Set flag for Online Status Updation
    }
}

//----------------------------------------------------------------------------------------------------------------------
// Data Sampling Controller function
//----------------------------------------------------------------------------------------------------------------------
void DataSamplingController()
{
    VFD_Ch1Data = _IQtoF(VFD_Data.SpeedRmp) * 120 * MOT_Data.Wb / PIx2 / MOT_Data.Poles;  // Speed Reference = p.u. * Base Speed
    VFD_Ch2Data = _IQtoF(VFD_Data.SpeedAct) * 120 * MOT_Data.Wb / PIx2 / MOT_Data.Poles;  // Speed Reference = p.u. * Base Speed
    VFD_Ch3Data = _IQtoF(VFD_Data.Isq) * MOT_Data.Ib;			// Torque = p.u. * Base Current
    VFD_Ch4Data = VFD_Ch1Data - VFD_Ch2Data;					// Error = Speed Command - Actual Speed

    if(VFD_GUIConfigData.SamplingMode)
    {
		VFD_GUIConfigData.SampleTimeCount++;
        if(VFD_GUIConfigData.SampleTimeCount > (VFD_GUIConfigData.SamplingTimeFactor-1))// Obtain sampling time which is a factor of switching frequency
        {
            VFD_GUIConfigData.SampleTimeCount   = 0;        // Re-initialise sampling factor variable
            GpioCtrlRegs.GPBMUX1.bit.GPIO38     = 3;    	// Active low write strobe; XWE0 [External Write --> External RAM]

            switch (VFD_GUIConfigData.noc)					// noc- No. of channels
            {
				case 0x01 :
				{
					if (VFD_GUIConfigData.SampleCount < VFD_GUIConfigData.Memory)   // Sample count less then No. of required samples
					{
						*AddrPtr    = GetVal(VFD_GUIConfigData.ChId1);				// Obtain signal value and store it in External RAM location
						VFD_GUIConfigData.SampleCount= VFD_GUIConfigData.SampleCount+1 ;							//Increment sample count variable
						AddrPtr     = AddrPtr+1;									// Increment External RAM memory location
					}
					else														    // When sample count reaches required number of samples
					{
						AddrPtr     = SegmentStrtAddr;								// Re-initialise address pointer to initial starting address
						VFD_GUIConfigData.DspResponse   = 0xA7;						// Assigning hex value A7 as DSP response
						VFD_GUIConfigData.SamplingMode  = 0;						// Clearing Sampling Mode status flag
					}
				}   break;

				case 0x02 :
				{
					if (VFD_GUIConfigData.SampleCount < VFD_GUIConfigData.Memory)   // Sample count less then No. of required samples
					{
						*AddrPtr    = GetVal(VFD_GUIConfigData.ChId1);				// Obtain first signal value and store it in External RAM location
						AddrPtr     = AddrPtr+VFD_GUIConfigData.Memory;				// Incrementing Address pointer to store second signal value as next segment
						*AddrPtr    = GetVal(VFD_GUIConfigData.ChId2);				// Obtain second signal value and store it in External RAM location
						AddrPtr     = SegmentStrtAddr+VFD_GUIConfigData.SampleCount;// Pointing adress pointer back to last location where first signal was stored
						VFD_GUIConfigData.SampleCount++;							// Increment sample count variable
						AddrPtr++;												    // Increment External RAM memory location
					}
					else
					{
						AddrPtr =  SegmentStrtAddr;								    // Re-initialise address pointer to initial starting address
						VFD_GUIConfigData.DspResponse   = 0xA7;						// Assigning hex value A7 as DSP response
						VFD_GUIConfigData.SamplingMode  = 0;						// Clearing Sampling Mode status flag
					}
				}   break;

				case 0x03 :
				{
					if (VFD_GUIConfigData.SampleCount < VFD_GUIConfigData.Memory)	// Sample count less then No. of required samples
					{
						*AddrPtr    = GetVal(VFD_GUIConfigData.ChId1);				// Obtain first signal value and store it in External RAM location
						AddrPtr     = AddrPtr+VFD_GUIConfigData.Memory;				// Incrementing Address pointer to store second signal value as next segment
						*AddrPtr    = GetVal(VFD_GUIConfigData.ChId2);				// Obtain second signal value and store it in External RAM location
						AddrPtr     = AddrPtr+VFD_GUIConfigData.Memory;				// Incrementing Address pointer to store third signal value as next segment
						*AddrPtr    = GetVal(VFD_GUIConfigData.ChId3);				// Obtain third signal value and store it in External RAM location
						AddrPtr     = SegmentStrtAddr+VFD_GUIConfigData.SampleCount;// Pointing adress pointer back to last location where first signal was stored
						SampleCount++;											    // Increment sample count variable
						AddrPtr++;												    // Increment External RAM memory location
					}
					else
					{
						AddrPtr =  SegmentStrtAddr;								    // Re-initialise address pointer to initial starting address
						VFD_GUIConfigData.DspResponse   = 0xA7;						// Assigning hex value A7 as DSP response
						VFD_GUIConfigData.SamplingMode  = 0;						// Clearing Sampling Mode status flag
					}
				}   break;

				case 0x04 :
				{
					if(VFD_GUIConfigData.SampleCount < VFD_GUIConfigData.Memory)	// Sample count less then No. of required samples
					{
						*AddrPtr    = GetVal(VFD_GUIConfigData.ChId1);				// Obtain first signal value and store it in External RAM  location
						AddrPtr     = AddrPtr+VFD_GUIConfigData.Memory;				// Incrementing Address pointer to store second signal value as next segment
						*AddrPtr    = GetVal(VFD_GUIConfigData.ChId2);				// Obtain second signal value and store it in External RAM  location
						AddrPtr     = AddrPtr+VFD_GUIConfigData.Memory;				// Incrementing Address pointer to store third signal value as next segment
						*AddrPtr    = GetVal(VFD_GUIConfigData.ChId3);				// Obtain third signal value and store it in External RAM  location
						AddrPtr     = AddrPtr+VFD_GUIConfigData.Memory;				// Incrementing Address pointer to store fourth signal value as next segment
						*AddrPtr    = GetVal(VFD_GUIConfigData.ChId4);				// Obtain fourth signal value and store it in External RAM  location
						AddrPtr     = SegmentStrtAddr+VFD_GUIConfigData.SampleCount;// Pointing adress pointer back to last location where first signal was stored
						VFD_GUIConfigData.SampleCount++;							// Increment sample count variable
						AddrPtr++;												    // Increment External RAM  memory location
					}
					else
					{
						AddrPtr =  SegmentStrtAddr;								    // Re-initialise address pointer to initial starting address
						VFD_GUIConfigData.DspResponse   = 0xA7;						// Assigning hex value A7 as DSP response
						VFD_GUIConfigData.SamplingMode  = 0;						// Clearing Sampling Mode status flag
					}
				}   break;
            }
        }
    }
    else
    {
        VFD_GUIConfigData.SampleCount = 0;									        // Clearing Sampling Mode status flag
    }
}

//----------------------------------------------------------------------------------------------------------------------
// SCI Write function
//----------------------------------------------------------------------------------------------------------------------
void UartWrite(int data)
{
    SCIB_RxTx_En = 1;                           // Transmit Enable
    delay(3);                                   // Delay of 3us

    if((ScibRegs.SCICTL2.all) & 0x00C0)     	// If data transmission is successful
    {
        ScibRegs.SCITXBUF = data;				// Transmit data
        VFD_GUIConfigData.DataSent = SUCCESS;	// Set Data Transmit Status to Success
    }
    else
    {
        VFD_GUIConfigData.DataSent = FAILED;	// If data transmission failed, Set status to Failed
    }
}

//----------------------------------------------------------------------------------------------------------------------
// EEPROM Read function
//----------------------------------------------------------------------------------------------------------------------
void ReadFromEeprom(unsigned WordsToRead)
{
    unsigned EpromReadLength = 0;           // Local variable "EpromReadLength"
    I2cStart();                             // Start condition
    I2cWrite(0xA0);                         // Write Slave Address (Write)
    I2cWrite(0x00);                         // Write Start Address (Higher byte)
    I2cWrite(0x00);                         // Write Start Address (Lower Byte)
    I2cStart();                             // Repeated Start condition
    I2cWrite(0xA1);                         // Write Slave Address (Read)

    for (EpromReadLength = 1; EpromReadLength < (WordsToRead+1); EpromReadLength++) // Scan range
        VFD_GUIConfigData.TxDataBuffer[EpromReadLength] = I2cRead(0);   // Read higher byte with ACK from EEPROM and copy to buffer

    temp1 = I2cRead(1);                     // Finish Read condition by a NACK state
    I2cStop();                              // Stop condition
    temp1 = 0;                              // Re-initialise
}

//----------------------------------------------------------------------------------------------------------------------
// EEPROM Write and Read Check function
//----------------------------------------------------------------------------------------------------------------------
void WriteReadChkFromEeprom(unsigned WordsToRead)
{
    //--------------------------------------------------------------------------------------------------------------------------
    // 128-byte Page Write to EEPROM
    //--------------------------------------------------------------------------------------------------------------------------
        unsigned EpromWriteLength = 0, EpromDelayCnt = 0, EpromReadLength = 0;    // Local variable initializations
	    I2cStart();                                     					// Start condition
	    I2cWrite(0xA0);                                 					// Write Slave Address (Write)
	    I2cWrite(0x00);                                 					// Write Start Address (Higher byte)
	    I2cWrite(0x00);                                 					// Write Start Address (Lower Byte)

	    for (EpromWriteLength = 1; EpromWriteLength < 129; EpromWriteLength++)  // Traverse all bytes in one page of EEPROM
	        I2cWrite(VFD_GUIConfigData.SerialDataBuffer1[EpromWriteLength]);// 128-byte page write

	    I2cStop();											                // Stop condition

	    for (EpromDelayCnt = 0; EpromDelayCnt < 1000; EpromDelayCnt++)
	        delay(10);									                    // Delay of 10us

	    I2cStop();
	    I2cStart();                             // Start condition
	    I2cWrite(0xA0);                         // Write Slave Address (Write)
		I2cWrite(0x00);                         // Write Start Address (Higher byte)
		I2cWrite(0x80);                         // Write Start Address (Lower Byte)

	    if (WordsToRead > 127)					// Move to next 128 byte page write
	    {
	    	for (EpromWriteLength = 129; EpromWriteLength < (WordsToRead+1); EpromWriteLength++)    // Traverse all bytes in next page of EEPROM
	    		I2cWrite(VFD_GUIConfigData.SerialDataBuffer1[EpromWriteLength]); 					// 128-byte page write
	    }

	    I2cStop();								//  Stop condition

	    for (EpromDelayCnt = 0; EpromDelayCnt < 1000; EpromDelayCnt++)
	        delay(10);							// Delay of 10us

    //--------------------------------------------------------------------------------------------------------------------------
    //Byte Read from EEPROM
    //--------------------------------------------------------------------------------------------------------------------------
	    I2cStart();                             // Start condition
	    I2cWrite(0xA0);                         // Write Slave Address (Write)
	    I2cWrite(0x00);                         // Write Start Address (Higher byte)
	    I2cWrite(0x00);                         // Write Start Address (Lower Byte)
	    I2cStart();                             // Repeated Start condition
	    I2cWrite(0xA1);                         // Write Slave Address (Read)

	    for (EpromReadLength = 1; EpromReadLength < (WordsToRead+1); EpromReadLength++) // Traverse all locations to read
	    {
	        if (EpromReadLength < WordsToRead)
	        {
	            tempread = I2cRead(0);			// Read condition by a ACK state
	        }
	        else
	        {
	            tempread = I2cRead(1);			// Finish Read condition by a NACK state
	            I2cStop();						// Stop condition
	        }
	        if (VFD_GUIConfigData.SerialDataBuffer1[EpromReadLength] == tempread)       // If Checksum byte matches
	        {
	            tempread = 0;								// Re-initilaise variable
	            VFD_GUIConfigData.DspResponse = 0xA0;		// Assign a DSP Response frame byte- 0xA0 indicating successful EEPROM write
	        }
	        else
	        {
	            VFD_GUIConfigData.DspResponse = 0xA5;       // Assign a DSP Response frame byte- 0xA5 indicating EEPROM write failure
	            I2cStop();									// Stop condition
	            break;
	        }
	    }
	    tempread = 0;                                       // Re-initialize variable
	}


void UpdateToLocalParameters()
{
/*
    GUIRxParameter.Rs = VFD_GUIConfigData.Rxd_data[0];
	GUIRxParameter.Rr = VFD_GUIConfigData.Rxd_data[1];
	GUIRxParameter.Ls = VFD_GUIConfigData.Rxd_data[2];
	GUIRxParameter.Lr = VFD_GUIConfigData.Rxd_data[3];
	GUIRxParameter.Lm = VFD_GUIConfigData.Rxd_data[4];
	GUIRxParameter.Poles = (int)VFD_GUIConfigData.Rxd_data[5];
	GUIRxParameter.MOT_Rotation = (int)VFD_GUIConfigData.Rx_data[VFD_GUIConfigData.DataByteLength+1] & 0x0001;
	GUIRxParameter.IM_MotorSpeed = (int)VFD_GUIConfigData.Rxd_data[6];

	GUIRxParameter.PI_Spd_Kp = _IQ(VFD_GUIConfigData.Rxd_data[7]);
	GUIRxParameter.PI_Spd_Ki = _IQ(VFD_GUIConfigData.Rxd_data[8]);
	GUIRxParameter.PI_Current_Kp = _IQ(VFD_GUIConfigData.Rxd_data[9]);
	GUIRxParameter.PI_Current_Ki = _IQ(VFD_GUIConfigData.Rxd_data[10]);
	GUIRxParameter.PI_Spd_IsqLim = _IQ(VFD_GUIConfigData.Rxd_data[11]);
	GUIRxParameter.PI_Current_VsqLim = _IQ(VFD_GUIConfigData.Rxd_data[12]);
	GUIRxParameter.OverCurrLimit = _IQ(VFD_GUIConfigData.Rxd_data[19]);
	GUIRxParameter.RampDelay = _IQ(VFD_GUIConfigData.Rxd_data[20]);

	GUIRxParameter.IM_RatedPower = (int)VFD_GUIConfigData.Rxd_data[13];
	GUIRxParameter.IM_RatedVoltage = (int)VFD_GUIConfigData.Rxd_data[14];
	GUIRxParameter.IM_RatedCurrent = (int)VFD_GUIConfigData.Rxd_data[15];
	GUIRxParameter.IM_RatedFreq = (int)VFD_GUIConfigData.Rxd_data[16];
	GUIRxParameter.VF_ModIndex = (int)VFD_GUIConfigData.Rxd_data[17];
	GUIRxParameter.SwitchFreq = (int)VFD_GUIConfigData.Rxd_data[18];
*/



}

//--------------------------------------------------------------------------------------------------------------------------
void GUI_StatusUpdate()
{
//--------------------------------------------------------------------------------------------------------------------------
// 1 assumed to be healthy condition-[Green in GUI]
// 0 assumed to be non-healthy condition-[Red in GUI]
//--------------------------------------------------------------------------------------------------------------------------
    VFD_Status.bits.GUIBit0 =  VFD_Status.bits.StartFlag;   // VFD Start/Stop
    VFD_Status.bits.GUIBit1 =  VFD_Status.bits.FaultFlag;   // General Warning/Trip
    VFD_Status.bits.GUIBit2 =  1;                           // Spare
    VFD_Status.bits.GUIBit3 =  VFD_Status.bits.StartFlag;   // Control Ready. Actual has to be replaced with 5V,3.3V healthy voltage condition
    VFD_Status.bits.GUIBit4 =  VFD_Status.bits.StartFlag;   // Drive Ready. Actual has to be replaced with 5V,3.3V healthy voltage condition

    VFD_Status.bits.GUIBit5 =  0;                           // Drive Remote Start
    VFD_Status.bits.GUIBit6 =  VFD_Status.bits.StartFlag;   // Running
    VFD_Status.bits.GUIBit7 =  VFD_Status.bits.FaultFlag;   // Alarm
//--------------------------------------------------------------------------------------------------------------------------
    if (VFD_Data.Isq >= PI_Isq.Umax)                        // Actual Torque < Limit
        VFD_Status.bits.GUIBit8  =  0;                      // Torque Limit reached
    else
        VFD_Status.bits.GUIBit8  =  1;                      // Torque Limit not reached
//--------------------------------------------------------------------------------------------------------------------------
    VFD_Status.bits.GUIBit9  =  0;                          // Out of current range
//--------------------------------------------------------------------------------------------------------------------------
    if (VFD_Data.VdcFilt < VFD_Limits.DC_UV_Limit)
        VFD_Status.bits.GUIBit10 =  0;                      // DC Bus Under Voltage
    else
        VFD_Status.bits.GUIBit10 =  1;                      // DC Bus Voltage within range
//--------------------------------------------------------------------------------------------------------------------------
    if (VFD_Data.VdcFilt > VFD_Limits.DC_OV_Limit)
        VFD_Status.bits.GUIBit11 =  0;
    else
        VFD_Status.bits.GUIBit11 =  1;                      // DC Bus Voltage within range
//--------------------------------------------------------------------------------------------------------------------------
    if (VFD_Data.HskTem > VFD_Limits.Hsk_OT_Limit)
        VFD_Status.bits.GUIBit12 =  0;                      // Thermal Warning
    else
        VFD_Status.bits.GUIBit12 =  1;                      // No Thermal Warning
//--------------------------------------------------------------------------------------------------------------------------
    if (VFD_Data.SpeedAct < _IQ(0.1))
        VFD_Status.bits.GUIBit13 =  0;                      // Below Low Speed
    else
        VFD_Status.bits.GUIBit13 =  1;                      // Within Speed range
//--------------------------------------------------------------------------------------------------------------------------
    if ( (VFD_Data.SpeedAct > _IQ(1.5) && VFD_Data.Mot_Type == 1) || (VFD_Data.SpeedAct > _IQ(1.0) && VFD_Data.Mot_Type == 2) )
        VFD_Status.bits.GUIBit14 =  0;                      // Above High Speed
    else
        VFD_Status.bits.GUIBit14 =  1;                      // Within Speed range
//--------------------------------------------------------------------------------------------------------------------------
    VFD_Status.bits.GUIBit15 =  0;                          // Out of Speed Range
//--------------------------------------------------------------------------------------------------------------------------


}
