//==================================================================================================================================================
// FILE NAME        : Funcs_4_FluxPosition.c
// DATE             : 01-Feb-2018
// Project          : VARIABLE FREQUENCY DRIVE FOR COMPRESSOR CONTROL APPLICATIONS
// Project Code     : PEG 124B
// Author           : Rohit V Thomas[ELGI] & Prashobh[CDAC]
//==================================================================================================================================================
// Include Header Files....
//==============================================================================================================================
#include "DSP28234_Device.h"                                // Includes all header files
#include "Variable.h"                                       // Include the Variables
#include "DSP28234_GlobalPrototypes.h"
#include "IQmathLib.h"
//------------------------------------------------------------------------------------------------------------------------------
//==============================================================================================================================
// Rotor Position Estimation Function
//==============================================================================================================================
void Rotor_Position_Estimator()
{
    if (PMSM_RPE.InjCnt ==  0)
    {

        PI_Isd.Kp = PI_Isq.Kp = _IQmpy(PI_Isd.Kp, _IQ(0.1));	// 1/10 of Designed Current Injected
    }

    if (PMSM_RPE.InjCnt < (VFD_Data.Sw_Freq * 1000))		// Current Count - Injection time, 10000 for 10kHz
    {
        PMSM_RPE.InjCnt++;
    //----------------------------------------------------------------------------------------------------------------------
    // Frequency Ramp Generator, depending on Speed Ref user ip (pu)
    //----------------------------------------------------------------------------------------------------------------------
        if(VFD_Status.bits.RampMode)
        {	// Speed Rampup when Start Switch is turned on
            MOT_RG.TargVal	= _IQ(0.5);	// 50Hz VFD_Data.SpeedRef
            MOT_RG.SetValue	= _IQ(0.5);	// 50Hz VFD_Data.SpeedRef, No Ramping up
        }
        else
        {	// Speed Rampdown when Start Switch is turned off
            MOT_RG.EqualFlag	= 0;
            MOT_RG.TargVal		= 0;
            MOT_RG.SetValue		= 0;	// Introduced for abrupt STOP (without Deceleration)
        }
                                                    // I/P => VFD_Data.SpeedRef:_________________#from User i/p Speed Ref (pu)
            Freq_Ramp_Gen();                        //
                                                    // O/P => MOT_RG.SetValue
            MOT_RC.Freq         = MOT_RG.SetValue;  // I/P => MOT_RG.SetValue:___________________#from Freq_Ramp_Gen ()
            Angle_Ramp();                           //
            VFD_Data.SpeedRmp   = MOT_RC.Freq;      // O/P => MOT_RC.Freq, MOT_RC.RampOut


    //----------------------------------------------------------------------------------------------------------------------
    // Transform measured ABC currents to Alpha-Beta currents
    //----------------------------------------------------------------------------------------------------------------------
											        // I/P => VFD_Data.Isa, VFD_Data.Isb:________#from EXT ADC
		Transform_3to2(VFD_Data.Isa, VFD_Data.Isb, VFD_Data.Isc);
											        //
											        // O/P => VFD_Data.Isalpha & VFD_Data.Isbeta
				
    //----------------------------------------------------------------------------------------------------------------------
    // Transform measured Alpha-Beta currents to D-Q currents
    //----------------------------------------------------------------------------------------------------------------------
        VFD_Data.CosEpsilon		= _IQcosPU(MOT_RC.RampOut);
        VFD_Data.SineEpsilon	= _IQsinPU(MOT_RC.RampOut);
		
        Transform_Forward(VFD_Data.IsAlpha, VFD_Data.IsBeta, VFD_Data.CosEpsilon, VFD_Data.SineEpsilon);
        //----------------------------------------------------------------------------------------------------------------------
											
        VFD_Data.IsqRef			= _IQ(0.0);			// 0 Torque Ref Injected
        VFD_Data.IsdRef			= _IQ(0.1);			// 1/10 of Flux Ref Injected
		
    //----------------------------------------------------------------------------------------------------------------------
    // ISD PI Controller, to generate the Vsd Ref (Vsd*)
    //----------------------------------------------------------------------------------------------------------------------
        PI_Isd.Ref				= VFD_Data.IsdRef;	// I/P => VFD_Data.FluxRef:__________________#from User i/p Flux Ref (pu)
        PI_Isd.Fbk				= VFD_Data.Isd;		// I/P => VFD_Data.Isq:______________________#from Transform_Forward()
        PI_Dcurrent();								//
		VFD_Data.VsdRef			= PI_Isd.Out;		// O/P => PI_Isd.Out

    //----------------------------------------------------------------------------------------------------------------------
    // ISQ PI Controller, to generate the Vsd Ref (Vsd*)
    //----------------------------------------------------------------------------------------------------------------------
        PI_Isq.Ref				= VFD_Data.IsqRef;	// I/P => VFD_Data.IsqRef:___________________#from User i/p Torque Ref (pu)
        PI_Isq.Fbk				= VFD_Data.Isq;		// I/P => VFD_Data.Isq:______________________#from Transform_Forward()
        PI_Qcurrent();								//		
        VFD_Data.VsqRef			= PI_Isq.Out;		// O/P => PI_Isq.Out
		
    //----------------------------------------------------------------------------------------------------------------------
    // Transform D-Q User Ref Voltages to Alpha-Beta Ref Voltages
    //----------------------------------------------------------------------------------------------------------------------
        Transform_Backward(VFD_Data.VsdRef, VFD_Data.VsqRef, VFD_Data.CosEpsilon, VFD_Data.SineEpsilon);
		
    //----------------------------------------------------------------------------------------------------------------------
    // Flux & Speed Estimator
    //----------------------------------------------------------------------------------------------------------------------
        MOT_FSE.IsAlpha     = VFD_Data.IsAlpha;	    // I/P => VFD_Data.Isalpha:__________________#from Transform_3to2()
        MOT_FSE.IsBeta      = VFD_Data.IsBeta;		// I/P => VFD_Data.Isbeta:___________________#from Transform_3to2()
        MOT_FSE.VsAlpha     = VFD_Data.VsAlphaRef;	// I/P => VFD_Data.Valpha:___________________#from Transform_Backward()
        MOT_FSE.VsBeta      = VFD_Data.VsBetaRef;	// I/P => VFD_Data.Vbeta:____________________#from Transform_Backward()

        Flux_Speed_Estimator();				        //
													// O/P => IM_FSE.ThetaFlux:__________________Estimated Rotor Flux Angle
        VFD_Data.SpeedAct    = MOT_FSE.WEstFilt;	// O/P => IM_FSE.WEstFilt:___________________Estimated Rotor Speed
        VFD_Data.CosTheta    = MOT_FSE.CosTheta;
        VFD_Data.SineTheta   = MOT_FSE.SineTheta;
        
        if ( PMSM_RPE.InjCnt > (VFD_Data.Sw_Freq * 100) && VFD_Data.Isq >= PMSM_RPE.Isq_Max )  // 1000 ms
        {	// Finding the Theta position at Max Current Peak of Isq => South Pole of PM ??
            PMSM_RPE.Isq_Max        = VFD_Data.Isq;
            MOT_FSE.ThetaFluxAtQ    = MOT_FSE.ThetaFlux;
        }
        if ( PMSM_RPE.InjCnt > (VFD_Data.Sw_Freq * 100) && VFD_Data.Isd >= PMSM_RPE.Isd_Max )   // 1000 ms
        {   // Finding the Theta position at Max Current Peak of Isd => North Pole of PM
            PMSM_RPE.Isd_Max        = VFD_Data.Isd;
            MOT_FSE.ThetaFluxAtD    = MOT_FSE.ThetaFlux;
        }

    //----------------------------------------------------------------------------------------------------------------------
    // Space Vector PWM Generator, from Vsalpha & Vsbeta
    //----------------------------------------------------------------------------------------------------------------------
        MOT_SVM.Alpha        = MOT_FSE.VsAlpha;	    // I/P => VFD_Data.VsAlphaRef:_______________#from Flux_Speed_Estimator_PMSM()
        MOT_SVM.Beta         = MOT_FSE.VsBeta;		// I/P => VFD_Data.VsBetaRef:________________#from Flux_Speed_Estimator_PMSM()
		THIPWM_Gen();                             	// PWM Generation
													// O/P => IM_SVM.Ta, IM_SVM.Tb, IM_SVM.Tc
		
    //--------------------------------------------------------------------------------------------------------------------------
    // PWM Generation Section
    //--------------------------------------------------------------------------------------------------------------------------
        EPWM_Generator();                           // Generate PWM from DSP EPWM module with the PWM reference signals
	//--------------------------------------------------------------------------------------------------------------------------
    }
	else
	{
		if (PMSM_RPE.RotPosFlag == 0)
		{
			VFD_Status.bits.StartFlag	= 0;
			INVERTER_OFF();
			Init_FreqRamp_Vars();       // Initialise Ramp Controller Parameters
            Init_TorqRamp_Vars();       // Initialise Torque Ramp Controller Parameters
			Init_VsqRamp_Vars();        // Initialise Flux Ramp Controller Parameters
			Init_VFD_Data_Vars();       // Initialise VFD Data Variables
			Init_FluxSpeedEst_Vars();   // Set Motor time constants for Flux & Speed Estimation
			Init_PIcntrl_Vars();        // Initialise PI Control
			Debounce_Counter      	= 0;
			PMSM_RPE.InjCnt       	= 0;

			if(PMSM_RPE.RotPosCnt >= PMSM_RPE.RotPosCntLim)
			{	// Limit the Rotor Estimation to RotPosCntLim times
				MOT_RC.RampOut		= MOT_FSE.ThetaFlux;

				PMSM_RPE.RotPosFlag	= 1;	//
			}
			else
				PMSM_RPE.RotPosCnt++;
		}
	}
}
//--------------------------------------------------------------------------------------------------------------------------
// END OF POSITION ESTIMATION
//--------------------------------------------------------------------------------------------------------------------------
