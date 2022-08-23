//==========================================================================================================================
// FILE NAME		: Funcs_0_Main.c
// DATE             : 01-Feb-2018
// Project			: VARIABLE FREQUENCY DRIVE FOR COMPRESSOR CONTROL APPLICATIONS
// Project Code		: PEG 124B
// Author			: Rohit V Thomas[ELGI], Manju R[CDAC] & Prashobh[CDAC] & Manoj.R [ELGI]
//==========================================================================================================================
// Include Header Files....
//==========================================================================================================================
#include "DSP28234_Device.h"								// Includes all header files
#include "Variable.h"										// Include the Variables
#include "DSP28234_GlobalPrototypes.h"
#include "IQmathLib.h"
//--------------------------------------------------------------------------------------------------------------------------
// Code Section for interrupt service routine
//--------------------------------------------------------------------------------------------------------------------------
#pragma CODE_SECTION (EPWM_INT_ISR,"INTERRUPT")
#pragma CODE_SECTION (EPWM_TZ_INT_ISR,"TZ_INTERRUPT")
#pragma CODE_SECTION (TIMER_ONE_ISR,"TMR_INTER")
//==========================================================================================================================
// MAIN CODE STARTS HERE........ 
//==========================================================================================================================

//==========================================================================================================================
// This code is for Redesigned Unit... PWM Configurations is different from the earlier Version
//==========================================================================================================================

int dac_counter = 0;
_iq zero_dummy_Isa = 0 ;
_iq zero_dummy_Isb = 0;


void main()

{

   Initialise_DSP();							// Initialize the DSP and it's peripherals

//#####################################################################
    // Control Data Source
    User_Data.Data_Srce     = 01;   // 00 => From Code
                                    // 01 => From GUI-

    // Speed Reference i/p Source
    User_Data.Refip_Srce    = 00;   // 00 => From Code
                                    // 01 => From GUI
                                    // 02 => From Analog i/p
                                    // 03 => From Digital Input

    User_Data.Motor_Type    = 01;   /* 01 => INDM   */
                                    /* 02 => PMSM   */

	Init_User_Data();               // Initialise User Data

    User_Data.Run = 0;              // GUI Start/Stop

//#####################################################################

	Initialise_Variables();						// Initialize VFD Parameters

	Init_DataLogger();                          // Initialise DATALOG module

	Init_SDRAMLogger();                         // Initialise SDRAM module



	 GpioDataRegs.GPADAT.bit.GPIO26 =  GPIO_SET;

//	I2C_WriteRead(); // This function is to write and read some value from EEPRPOM


	while(1)
    {
	//    EXT_ADC_EOC();							// External ADC End Read
	    Offset_Reading();
	    ADC_Processing();							// Internal ADC Read
		Interface();							// Drive Start/Stop Interface
		System_Faults();						// Various System Protection Functions
		  // offest 1.65V Reading
		//-------------------------------------------------------------------------------------------------------------------
    // MODBUS Handling
    //-------------------------------------------------------------------------------------------------------------------
	//    Modbus();                               // MODBUS Communication

    //-------------------------------------------------------------------------------------------------------------------
    // INTERRUPT
    //-------------------------------------------------------------------------------------------------------------------
   //		EXT_DAC_Write(0,0);
   		Wait_for_Interrupt();					// Wait for Interrupt // The same function is hided else in every 333us only SPI DAC will get updated.

   //		Uart_Serial ();

   	//	UartWrite(0x10);


 //  	 Update_datalogger ();

	}
}
//==========================================================================================================================
// Main Code Ends Here.....
//==========================================================================================================================


//==========================================================================================================================
// Routine for Wait for Interrupt... 
//==========================================================================================================================
void Wait_for_Interrupt()
{
	while(1)
	{

//	  EXT_DAC_Write((_IQtoF(MOT_SVM.Ta)* 0x1FFF),(_IQtoF(MOT_SVM.Tb)* 0x1FFF),(_IQtoF(MOT_SVM.Tc)* 0x1FFF),(_IQtoF(VFD_Data.Isa)* 0x1FFF));
	    if(VFD_Status.bits.IntrptFlag != 0)					// Wait till Interrupt Driven Flag is Set by the Interrupt Service Routine for Timer interrupt
		{
			VFD_Status.bits.IntrptFlag = 0;					// Clear the flag
			break;
		}
	}
}
//==========================================================================================================================
// Interrupt Service Routine [EPWM_INT][Timer1 Underflow Interrupt]
//==========================================================================================================================
interrupt void EPWM_INT_ISR()
{
	VFD_Status.bits.IntrptFlag	= 1;						// Set the Interrupt Driven Flag
	PieCtrlRegs.PIEACK.all 		= 0x0004;					// Clear all the PIE Acknowledgments as specified in the datasheet
	EPwm1Regs.ETCLR.bit.INT 	= 1;

	CpuTimer1Regs.TCR.bit.TSS = 0;// Clear the Event Trigger Flag for Timer underflow interrupt
 //   GpioDataRegs.GPADAT.bit.GPIO27  = GPIO_SET ;
	//	else
//	CpuTimer1Regs.TCR.bit.TSS = 1;
	EINT;													// Enable Interrupt
}															// Return from Interrupt Service Routine
//==========================================================================================================================
// Interrupt Service Routine [EPWM_TZ_INT][TripZone 1 ]
//==========================================================================================================================
interrupt void EPWM_TZ_INT_ISR()
{
	Initialise_Variables();
	VFD_Status.bits.FaultFlag  	= 1;						// Set the Fault Flag for Power Base Fault
	VFD_Status.bits.StartFlag  	= 0;						// System Off
	INVERTER_OFF();											// Inverter OFF
	PieCtrlRegs.PIEACK.all 		= 0xFFFF;					// Clear all the PIE Acknowledgments as specified in the datasheet
	EPwm1Regs.TZCLR.bit.OST		= 1;						// Clear Trip Zone Interrupt Flag for OST Latch
	EPwm1Regs.TZCLR.bit.INT		= 1;						// Clear the Trip Zone Interrupt Global Interrupt Flag
	RELAY_2_Ctrl = 1;
	EINT;													// Enable Interrupt
}															// Return from Interrupt Service Routine

//==========================================================================================================================
// Code ends...
//==========================================================================================================================


interrupt void TIMER_ONE_ISR()
{

  //  CpuTimer1Regs.TCR.bit.TSS = 1; // Stop the timer
//    GpioDataRegs.GPADAT.bit.GPIO27 =  GPIO_SET ;
//    VFD_Status.bits.IntrptFlag  = 1;

    GpioDataRegs.GPADAT.bit.GPIO26 = ~ GpioDataRegs.GPADAT.bit.GPIO26 ;

    IFR = IFR & 0xEFFF;  // Acknowledgement of CPU Timer One  ;

    EINT ;  // This is to allow the higher priority interrupts i.e ePWM and Trip Zone

    if(VFD_Data.ADC_Flag ==1)
    ADC_READ();       // ADC to be read only after initilisation of ADC

 //   EXT_DAC_Write((_IQtoF(VFD_Data.Isd)* 0x1FFF),(_IQtoF(VFD_Data.Isq)* 0x1FFF),(_IQtoF(VFD_Data.Isc)* 0x1FFF),(_IQtoF(VFD_Data.Isa)* 0x1FFF));

    Motor_Selector();    // Motor Control Function Executed every 100us

/*
    if(VFD_Data.Isa > _IQ(0.0))
    GpioDataRegs.GPADAT.bit.GPIO27 = GPIO_SET ;
    else
    GpioDataRegs.GPADAT.bit.GPIO27 = GPIO_CLEAR ;

    if(VFD_Data.Isa > _IQ(0.0))
    GpioDataRegs.GPADAT.bit.GPIO26 = GPIO_SET ;
    else
    GpioDataRegs.GPADAT.bit.GPIO26 = GPIO_CLEAR ; */


    EXT_DAC_Write((_IQtoF(VFD_Data.Isa)*0x1FFF), (_IQtoF(VFD_Data.Isb)* 0x1FFF),(_IQtoF(VFD_Data.Isc)* 0x1FFF),(_IQtoF(MOT_FSE.PsisBeta_comp)* 0x1FFF));
//    EXT_DAC_Write((_IQtoF(VFD_Data.Isa)*0x1FFF), (_IQtoF(MOT_FSE.PsisAlpha_comp) * 0x1FFF),(_IQtoF(MOT_FSE.PsisBeta)* 0x1FFF),(_IQtoF(MOT_FSE.PsisBeta_comp)* 0x1FFF));


//    GpioDataRegs.GPADAT.bit.GPIO27 =  GPIO_CLEAR ;



}

