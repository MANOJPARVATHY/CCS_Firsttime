//==================================================================================================================================================
// FILE NAME		: Funcs_0_Interlocks.c
// DATE             : 01-Feb-2018
// Project			: VARIABLE FREQUENCY DRIVE FOR COMPRESSOR CONTROL APPLICATIONS
// Project Code		: PEG 124B
// Author			: Rohit V Thomas[ELGI], Manju R[CDAC] & Prashobh[CDAC]
//==================================================================================================================================================
// Include Header Files....
//==========================================================================================================================
#include "DSP28234_Device.h"								// Includes all header files
#include "Variable.h"										// Include the Variables
#include "DSP28234_GlobalPrototypes.h"
#include "IQmathLib.h"
#include "DLOG_4CH_IQ.h"                                    // Include header for the DLOG_4CH object
//--------------------------------------------------------------------------------------------------------------------------
extern DLOG_4CH_IQ dlog;

//=========================================================================================================================
// User Interface
//=========================================================================================================================
void Interface()
{
// Digital I/P 1 for System Start/Stop from Compressor Neuron Controller
// Digital I/P 2 for System Start/Stop from On/Off Switch

    if ( (DI_IP1==SystemOn) ^ (DI_IP2==SystemOn) ^ (User_Data.Run==1) ) // XOR
	{
        RELAY_PTC_Ctrl   = 0;

        if (VFD_Status.bits.FaultFlag == 0 && Debounce_Counter > (VFD_Data.Sw_Freq * 500))  // fixed debounce of 0.5s
		{
			VFD_Status.bits.StartFlag 	= 1;
			VFD_Status.bits.RampMode    = 1;    // Ramping up with Start On
			VFD_Status.bits.UpdateVars  = 1;    // Update VFD variables
		//	GpioDataRegs.GPADAT.bit.GPIO27 =  GPIO_SET ;
			INVERTER_ON();
        }
		else
			Debounce_Counter=Debounce_Counter+1;
	}
	else
	{
	    RELAY_PTC_Ctrl   = 1;
	    Debounce_Counter 			= 0;
		VFD_Status.bits.RampMode 	= 0;    // Ramping down with Start Off
		dlog.status = 1;                    // 0 => logs data as per trig_value
		                                    // 1 => logs data instantaneously
		// Uncomment below Condition for Rampdown
//		if (MOT_RC.Freq == 0)           // !! CAUTION !! Never use actual speed for turn off as Westfilter does not hold true for <2Hz
		{
		    VFD_Status.bits.StartFlag = 0;
			INVERTER_OFF();

			if (VFD_Status.bits.UpdateVars  == 1 )  // Update VFD variables once only when SYSTEM is OFF
            {
                Init_VFD_Data_Vars();       // Initialise VFD Data Variables
                Init_FreqRamp_Vars();	    // Initialise Ramp Controller Parameters
                Init_VsdRamp_Vars();       // Initialise Torque Ramp Controller Parameters
                Init_VsqRamp_Vars();        // Initialise VsQ Ramp Controller Parameters
                Init_VbyF_Vars();		    // Initialise VbyF Control Profile Parameters
                Init_FluxSpeedEst_Vars();   // Set Induction motor time constants for Flux & Speed Estimation
                Init_PIcntrl_Vars();        // Initialise PI Control
                Init_SVM_Vars();            // Initialise SVPWM Generator Structure
                Init_RotPosEst_Vars();      // Initialise Rotor position Estimation
                Init_DynBrake_Vars();       // Initialise Dynamic Brake Control Variables

    //			Init_VFD_Data_Reset();      // Reset VFD Data Variables

                VFD_Status.bits.UpdateVars  = 0;
            }

			if (User_Data.ReInitParm == 1)          // User modification from GUI
			    VFD_Status.bits.ReInitParm = 1;

			if (VFD_Status.bits.ReInitParm  == 1 )  // Reinit VFD Parameters only when SYSTEM is OFF
			{
			    if (User_Data.Data_Srce == 0)   // if Data Source is User Code
			        Init_User_Data();       // Initialise User Data from Code

			    Initialise_Variables();     // Initialize VFD Parameters
			    VFD_Status.bits.ReInitParm  = 0;
			    User_Data.ReInitParm = 0;
			}

		}
	}
    // Relay O/P indicating motor reached set speed
    if (MOT_RG.EqualFlag == 0x1FFF)
        RELAY_1_Ctrl = 1;   // Connected as NO
    else
        RELAY_1_Ctrl = 0;

    // Relay O/P indicating VFD Fault
    if (VFD_Status.bits.FaultFlag != 0)
        RELAY_2_Ctrl = 1;
    else
        RELAY_2_Ctrl = 0;   // Connected as NC


    // Noise Suppression Relay turned on for INDM
 /*   if (VFD_Data.Mot_Type == 1)
        RELAY_NSP_Ctrl   = 1;  //   RELAY_NSP_Ctrl   = 0;
    else
        RELAY_NSP_Ctrl   = 1; */

    //------------------------------------------------------------------------------------------------------
    // Speed Reference & Acceleration Interlocks
    //------------------------------------------------------------------------------------------------------

    if (User_Data.Refip_Srce <= 01)         // Speed Reference from Code or GUI
    {
        // Setting Output Frequency
        if ((User_Data.Op_Freq / (MOT_Data.Wb / PIx2)) < 0.1)
        {
            VFD_Data.SpeedRef = _IQ(0.1);               // Speed reference (pu)  = 1.0 * 1995 = 1995rpm (Base Speed = 1995rpm)
            User_Data.Op_Freq = 0.1 * MOT_Data.Wb / PIx2;
        }
        else if ((User_Data.Op_Freq / (MOT_Data.Wb / PIx2)) > 1.0 && User_Data.Motor_Type == 2)
        {
            VFD_Data.SpeedRef = _IQ(1.0);               // PMSM Flux Weakening Region (Not Permitted)
            User_Data.Op_Freq = 1.0 * MOT_Data.Wb / PIx2;
        }
        else if ((User_Data.Op_Freq / (MOT_Data.Wb / PIx2)) > 1.0 && User_Data.Motor_Type == 1)
        {
            VFD_Data.SpeedRef = _IQ(1.0);               // INDM Flux Weakening Region (TEMP limited to 1.0)
            User_Data.Op_Freq = 1.00 * MOT_Data.Wb / PIx2;
        }
        else
            VFD_Data.SpeedRef = _IQ((float32) User_Data.Op_Freq / (MOT_Data.Wb / PIx2));
    }

    else if (User_Data.Refip_Srce == 02)    // Speed Reference from Analog i/p
    {
        // Setting Output Frequency
    /*    if (VFD_Data.SpdRef < 0.1)
            VFD_Data.SpeedRef = _IQ(0.1);               // Speed reference (pu)  = 1.0 * 1995 = 1995rpm (Base Speed = 1995rpm)
        else if (VFD_Data.SpdRef > _IQ(1.0) && User_Data.Motor_Type == 2)
            VFD_Data.SpeedRef = _IQ(1.0);      //_IQmpy(         // PMSM Flux Weakening Region (Not Permitted)
        else if (VFD_Data.SpdRef > _IQ(1.0) && User_Data.Motor_Type == 1)
            VFD_Data.SpeedRef = _IQ(1.0);               // INDM Flux Weakening Region (TEMP limited to 1.0)
        else
            VFD_Data.SpeedRef = VFD_Data.SpdRef; */
        // Setting Output Frequency
        VFD_Data.SpeedRef = _IQmpy(VFD_Data.SpdAnalConstant,(VFD_Data.SpdRef - VFD_Data.AnaMin)) + VFD_Data.SpdMin ;

        if (VFD_Data.SpeedRef < _IQ(0.5))
            VFD_Data.SpeedRef = _IQ(0.5);               // Speed reference (pu)  = 1.0 * 1995 = 1995rpm (Base Speed = 1995rpm)
        else if (VFD_Data.SpeedRef > _IQ(1.0) && User_Data.Motor_Type == 2)
            VFD_Data.SpeedRef = _IQ(1.0);      //_IQmpy(         // PMSM Flux Weakening Region (Not Permitted)
        else if (VFD_Data.SpdRef > _IQ(1.0) && User_Data.Motor_Type == 1)
            VFD_Data.SpeedRef = _IQ(1.0);               // INDM Flux Weakening Region (TEMP limited to 1.0)
        else
            VFD_Data.SpeedRef = VFD_Data.SpeedRef;
    }
    else
        VFD_Data.SpeedRef = _IQ(0.0);


    // Setting Ramp Time
    if ((User_Data.Frated / User_Data.Accel_Rate) < RmpTm_min)
    {
        MOT_RG.Delay            = RmpTm_min;            // Ramp Delay = 10 * 1s (Min Ramp time = 10s)
        User_Data.Accel_Rate    = User_Data.Frated / RmpTm_min;

    }
    else if ((User_Data.Frated / User_Data.Accel_Rate) > RmpTm_max)
    {
        MOT_RG.Delay            = User_Data.Frated / RmpTm_max; // Ramp Delay = 120 * 1s (Max Ramp time = 120s)
        User_Data.Accel_Rate    = User_Data.Frated / RmpTm_max;
    }
    else
        MOT_RG.Delay        = (User_Data.Frated / User_Data.Accel_Rate);


}


