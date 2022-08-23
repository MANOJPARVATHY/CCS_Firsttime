//==================================================================================================================================================
// FILE NAME		: Funcs_3_MotorControl.c
// DATE             : 01-Feb-2018
// Project			: VARIABLE FREQUENCY DRIVE FOR COMPRESSOR CONTROL APPLICATIONS
// Project Code		: PEG 124B
// Author			: Rohit V Thomas[ELGI], Manju R[CDAC] & Prashobh[CDAC]
//==================================================================================================================================================
// Include Header Files....
//==============================================================================================================================
#include "DSP28234_Device.h"								// Includes all header files
#include "Variable.h"										// Include the Variables
#include "DSP28234_GlobalPrototypes.h"
#include "IQmathLib.h"

//--------------------------------------------------------------------------------------------------------------------------
// START OF MOTOR Functions
//--------------------------------------------------------------------------------------------------------------------------

//==============================================================================================================================
// Motor Selector Function
//==============================================================================================================================
void Motor_Selector()
{
//------------------------------------------------------------------------------------------------------
// IND Motor Controller
//------------------------------------------------------------------------------------------------------
    if (VFD_Data.Mot_Type == 1) // INDM
    {
        if (VFD_Status.bits.StartFlag)       // Allow the Motor Control Algorithm only when StartFlag is Set
        {
            Motor_Control();                 // Motor Controller Function
        }
    }

//------------------------------------------------------------------------------------------------------
// PMS Motor Controller
//------------------------------------------------------------------------------------------------------
    if (VFD_Data.Mot_Type == 2) // PMSM
    {
/*        if (VFD_Status.bits.StartFlag == 1 && PMSM_RPE.RotPosFlag == 0)
            Rotor_Position_Estimator();         // Estimating the Rotor Position at Standstill condition

        if (VFD_Status.bits.StartFlag == 0 && PMSM_RPE.RotPosFlag == 1)
        {   // Options to take Theta angle - 1) from D axis, 2) from Q axis ??
            MOT_RC.Angle   =  MOT_FSE.ThetaFluxAtD + _IQ(1/(float32)MOT_Data.Poles);
            // Theta estimated at D axis, so angle + 45 degree to start at S pole
//          MOT_RC.Angle   =  MOT_FSE.ThetaFluxAtQ + _IQ(2/(float32)MOT_Data.Poles);
                                                // Theta estimated at Q axis, so angle + 90 degree to start at next S pole
        }
*/
        if (VFD_Status.bits.StartFlag==1/* && PMSM_RPE.RotPosFlag == 1*/)
            Motor_Control();                    // Allow the Motor Control Algorithm only when StartFlag is Set
    }
}
//==============================================================================================================================
// Motor Control Function
//==============================================================================================================================
void Motor_Control()
{
    // Update User Kp & Ki Variables
    if (User_Data.Data_Srce == 01)
    {
        User_Data.SpeedKp = PI_Spd.Kp;
        User_Data.SpeedKi = PI_Spd.Ki;
        User_Data.CurrKp  = PI_Isd.Kp;
        User_Data.CurrKi  = PI_Isd.Ki;
    }

//##############################################################################################################################
// INDM V/F - Open Loop
//##############################################################################################################################
    if (VFD_Data.Con_Mode == MODE11)
    {
    //----------------------------------------------------------------------------------------------------------------------
    // Frequency Ramp Generator, depending on Speed Ref user ip (pu)
    //----------------------------------------------------------------------------------------------------------------------
        if (VFD_Status.bits.RampMode)            // Speed Rampup when Start Switch is turned on
        {
            MOT_RGI.TargVal = _IQ(0.1);
            MOT_RG.TargVal = VFD_Data.SpeedRef;
        }
        else                                    // Speed Rampdown when Start Switch is turned off
        {
            MOT_RGI.TargVal= 0;
            MOT_RG.EqualFlag = 0;
            MOT_RG.TargVal   = 0;           // Dual Ramp up is introduced to reduce the starting the current..
        }

        Freq_Init_Ramp();

       if(MOT_RGI.EqualFlag== 0)
        {
            MOT_RG.SetValue = _IQ(0.1);
            MOT_RC.Freq     = _IQ(0.1);//MOT_RGI.SetValue;  // I/P => MOT_RG.SetValue:___________________#from Freq_Ramp_Gen ()
        }

        Freq_Ramp_Gen();
        MOT_RC.Freq = MOT_RG.SetValue; // I/P => MOT_RG.SetValue:___________________#from Freq_Ramp_Gen ()

                                                // I/P => VFD_Data.SpeedRef:_________________#from User i/p Speed Ref (pu)

        Angle_Ramp();                           //
                                                // O/P => MOT_RC.RampOut

    //----------------------------------------------------------------------------------------------------------------------
    // V/F Profile Generator
    //----------------------------------------------------------------------------------------------------------------------
        MOT_VF.Freq  = MOT_RC.Freq;             // I/P => MOT_RC.Freq:_______________________#from Freq_Ramp_Gen()
        VbyF_Gen();                             //
                                                // O/P => MOT_VF.VoltOut
        VFD_Data.SpeedRmp = MOT_VF.Freq;        // O/P => MOT_VF.FreqOut

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
                                                // I/P => VFD_Data.Isalpha, VFD_Data.Isbeta:_#from Transform_3to2()
                                                // I/P => VFD_Data.CosTheta:_________________#from from Flux_Speed_Estimator()
        Transform_Forward(VFD_Data.IsAlpha, VFD_Data.IsBeta, VFD_Data.CosTheta, VFD_Data.SineTheta);
                                                // Estimated angle from Flux Speed Estimator to be used
                                                // O/P => VFD_Data.Isd & VFD_Data.Isq

    //----------------------------------------------------------------------------------------------------------------------
    // Transform D-Q User Ref Voltages to Alpha-Beta Ref Voltages
    //----------------------------------------------------------------------------------------------------------------------
        VFD_Data.VsqRef     = _IQsat(MOT_VF.VoltOut, VFD_Data.VsqLim, _IQ(0.0));
                                                // I/P => VFD_Data.VsdRef, VFD_Data.VsqRef:__#from V/F Profile Generator()
        VFD_Data.VsdRef = 0;                    // I/P => MOT_RC.RampOut:____________________#from Freq_Ramp_Gen()
        VFD_Data.CosEpsilon     = _IQcosPU(MOT_RC.RampOut);
        VFD_Data.SineEpsilon    = _IQsinPU(MOT_RC.RampOut);
                                                // Angle from Freq Ramp
        VFD_Data.CosTheta   = MOT_FSE.CosTheta;
        VFD_Data.SineTheta  = MOT_FSE.SineTheta;
                                                // Estimated Flux Angle
        Transform_Backward(VFD_Data.VsdRef, VFD_Data.VsqRef, VFD_Data.CosEpsilon, VFD_Data.SineEpsilon);
                                                // O/P => VFD_Data.VsAlphaRef, VFD_Data.VsBetaRef

    //----------------------------------------------------------------------------------------------------------------------
    // Speed Estimator
    //----------------------------------------------------------------------------------------------------------------------
        MOT_FSE.IsAlpha= VFD_Data.IsAlpha ; // VFD_Data.DAxisFilt ;      // I/P => VFD_Data.Isalpha:__________________#from Transform_3to2()
        MOT_FSE.IsBeta = VFD_Data.IsBeta ; //VFD_Data.VdcFilt;//_IQmpy(_IQ(2.0),VFD_Data.VdcFilt);//VFD_Data.IsBeta;       // I/P => VFD_Data.Isbeta:___________________#from Transform_3to2()
        MOT_FSE.VsAlpha= VFD_Data.VsAlphaRef;   // I/P => MOT_VSI.Valpha:____________________#from Transform_Backward()
        MOT_FSE.VsBeta = VFD_Data.VsBetaRef;    // I/P => MOT_VSI.Vbeta:_____________________#from Transform_Backward()
        Flux_Speed_Estimator();                 //
                                                // O/P => MOT_FSE.ThetaFlux:__________________Estimated Rotor Flux Angle
                                                // O/P => MOT_FSE.WEstFilt:___________________Estimated Rotor Speed
        VFD_Data.SpeedAct = MOT_FSE.WEstFilt;   //

    //----------------------------------------------------------------------------------------------------------------------
    // Space Vector PWM Generator, from Vsalpha & Vsbeta
    //----------------------------------------------------------------------------------------------------------------------
        MOT_SVM.Alpha = VFD_Data.VsAlphaRef;    // I/P => VFD_Data.VsAlphaRef:_______________#from Transform_Backward()
        MOT_SVM.Beta  = VFD_Data.VsBetaRef;     // I/P => VFD_Data.VsBetaRef:________________#from Transform_Backward()
        THIPWM_Gen();                           //
                                                // O/P => MOT_SVM.Ta, MOT_SVM.Tb, MOT_SVM.Tc

    }

//##########################################################################################################################
// INDM V/F - Closed Loop
//##########################################################################################################################
    if (VFD_Data.Con_Mode == MODE12)
    {
    //----------------------------------------------------------------------------------------------------------------------
    // Frequency Ramp Generator, depending on Speed Ref user ip (pu)
    //----------------------------------------------------------------------------------------------------------------------
        if (VFD_Status.bits.RampMode)            // Speed Rampup when Start Switch is turned on
               {
                   MOT_RGI.TargVal = _IQ(0.1);
                   MOT_RG.TargVal = VFD_Data.SpeedRef;
               }
               else                                    // Speed Rampdown when Start Switch is turned off
               {
                   MOT_RGI.TargVal= 0;
                   MOT_RG.EqualFlag = 0;
                   MOT_RG.TargVal   = 0;           // Dual Ramp up is introduced to reduce the starting the current..
               }

               Freq_Init_Ramp();

        if(MOT_RGI.EqualFlag== 0)
       {
           MOT_RG.SetValue = _IQ(0.1);
           MOT_RC.Freq     = _IQ(0.1);//MOT_RGI.SetValue;  // I/P => MOT_RG.SetValue:___________________#from Freq_Ramp_Gen ()
       }

        Freq_Ramp_Gen();                        //
        MOT_RC.Freq  = MOT_RG.SetValue;         // O/P => MOT_RC.Freq


    //----------------------------------------------------------------------------------------------------------------------
    // Speed PI Controller, generate Freq Reference for closed loop V/F
    //----------------------------------------------------------------------------------------------------------------------
  //      if (PI_Spd.LoopCnt == PI_Spd.PreScaler)
  //      {                                       // Execute Speed PI Control every Prescaler*Ts time only
            PI_Spd.Ref  = MOT_RC.Freq;          // I/P => MOT_RC.Freq:_______________________#from VbyF_Gen()
            PI_Spd.Fbk  = MOT_FSE.WEstFilt;     // I/P => MOT_FSE.WEstFilt:__________________#from Flux_Speed_Estimator()
            PI_Spd.Kp   = _IQ(MOT_Data.SigmaR);
                                                // Kp = SigmaR
            if (MOT_RG.EqualFlag != 0x1FFF)     // PI Speed only after Full Speed reached
                PI_Spd.Ki   = 0;
            else
                PI_Spd.Ki   = _IQmpy(VFD_Data.Ts,_IQ(10.0));
                                                // Ki = Ts*10
            PI_Spd.Umax = _IQ(0.025);           // Umax = 0.025 Freq for 35rpm
            PI_Spd.Umin = _IQ(-0.05);           // Umin = -0.05 Freq for 75V
            PI_Speed();                         //
  //          PI_Spd.LoopCnt = 1;                 // O/P => PI_Spd.Out
  //      }
   //     else    PI_Spd.LoopCnt ++;

    //----------------------------------------------------------------------------------------------------------------------
    // V/F Profile Generator
    //----------------------------------------------------------------------------------------------------------------------
        if (MOT_RG.EqualFlag != 0x1FFF)         // PI Speed only after Full Speed reached // Hided for Redesign
            MOT_VF.Freq  = MOT_RC.Freq;
        else
            MOT_VF.Freq  = MOT_RC.Freq + PI_Spd.Out;

                                                // I/P => MOT_RC.Freq:_______________________#from Freq_Ramp_Gen()
                                                // I/P => PI_Spd.Out:________________________#from PI_Speed()
        VbyF_Gen();                             //
                                                // O/P => MOT_VF.VoltOut
        VFD_Data.SpeedRmp = MOT_VF.FreqOut;     // O/P => MOT_VF.FreqOut
        MOT_RC.Freq  = MOT_VF.FreqOut;          // I/P => MOT_VF.FreqOut:____________________#from VbyF_Gen()
        Angle_Ramp();                           //
                                                // O/P => MOT_RC.RampOut

    //----------------------------------------------------------------------------------------------------------------------
    // Transform measured ABC currents to Alpha-Beta currents
    //----------------------------------------------------------------------------------------------------------------------
                                                // I/P => VFD_Data.Isa, VFD_Data.Isb:________#from EXT ADC
        Transform_3to2(VFD_Data.Isa, VFD_Data.Isb, VFD_Data.Isc);
                                                //
                                                // O/P => VFD_Data.IsAlpha & VFD_Data.IsBeta

    //----------------------------------------------------------------------------------------------------------------------
    // Transform measured Alpha-Beta currents to D-Q currents
    //----------------------------------------------------------------------------------------------------------------------
                                                // I/P => VFD_Data.Isalpha, VFD_Data.Isbeta:_#from Transform_3to2()
                                                // I/P => VFD_Data.CosTheta:_________________#from from Flux_Speed_Estimator()
        Transform_Forward(VFD_Data.IsAlpha, VFD_Data.IsBeta, VFD_Data.CosTheta, VFD_Data.SineTheta);
                                                // Estimated angle from Flux Speed Estimator to be used
                                                // O/P => VFD_Data.Isd & VFD_Data.Isq

    //----------------------------------------------------------------------------------------------------------------------
    // VDC P Controller
    //----------------------------------------------------------------------------------------------------------------------
        // P Controller for Vdc                                          // VFD_Data.Vdc // To be changed for new design..
   //     VFD_Data.VsqRefDelta = _IQmpy(VFD_Data.VdcKp, (_IQ(1.0) - _IQ(0.90)/*VFD_Data.VdcFilt*/));    // Proportional Controller // Hided for Redesign


        VFD_Data.VsqRefDelta = _IQmpy(VFD_Data.VdcKp, (_IQ(1.0) - VFD_Data.VdcFilt));    // Proportional Controller
        VFD_Data.VsqRefDelta = _IQsat(VFD_Data.VsqRefDelta, _IQ(0.25), _IQ(-0.25));        // Saturation

    //----------------------------------------------------------------------------------------------------------------------
    // Delta Vsq Ramp Generator
    //----------------------------------------------------------------------------------------------------------------------
        if (MOT_RG.EqualFlag == 0x1FFF)          // Delta Vsq Ramping only after Full Speed reached
            MOT_RGQ.TargVal  = VFD_Data.VsqRefDelta;
        else
        {
            MOT_RGQ.EqualFlag    = 0;
            MOT_RGQ.TargVal      = 0;
        }

        MOT_RGQ.Delay    = _IQtoF(_IQmpy(MOT_Data.Trt, _IQ(10)));
                                                // Ramptime = 0.1*Trt s
                                                // I/P => VFD_Data.VsqRefDelta:______________#from User i/p Speed Ref (pu)
        Vsq_Ramp_Gen();                         //
                                                // O/P => MOT_RGQ.SetVal

    //----------------------------------------------------------------------------------------------------------------------
    // Transform D-Q User Ref Voltages to Alpha-Beta Ref Voltages
    //----------------------------------------------------------------------------------------------------------------------
   /*     if (_IQabs(VFD_Data.SpeedAct) > _IQ(1.0))
        {                                       // If Actual Speed > Rated Speed, Field Weakening Mode
            if (MOT_RG.EqualFlag != 0x1FFF)     // Delta Vsq Ramping only after Full Speed reached
                VFD_Data.VsqRef = _IQsat((MOT_VF.VoltOut), _IQmpy(VFD_Data.VsqLim, _IQabs(_IQ(2.0) - VFD_Data.SpeedAct)), _IQ(0.0));
            else
                VFD_Data.VsqRef = _IQsat((MOT_VF.VoltOut + MOT_RGQ.SetValue), _IQmpy(VFD_Data.VsqLim, _IQabs(_IQ(2.0) - VFD_Data.SpeedAct)), _IQ(0.0));
                                                // I/P => VFD_Data.VsdRef, VFD_Data.VsqRef:__#from V/F Profile Generator()
        }*/ // Hided for redesign
     //   else
        {
            if (MOT_RG.EqualFlag != 0x1FFF)     // Delta Vsq Ramping only after Full Speed reached
                VFD_Data.VsqRef = _IQsat((MOT_VF.VoltOut), VFD_Data.VsqLim, _IQ(0.0));
            else
                VFD_Data.VsqRef = _IQsat((MOT_VF.VoltOut + MOT_RGQ.SetValue), VFD_Data.VsqLim, _IQ(0.0));
                                                // I/P => VFD_Data.VsdRef, VFD_Data.VsqRef:__#from V/F Profile Generator()
        }


        VFD_Data.VsdRef = 0;                    // I/P => MOT_RC.RampOut:____________________#from Freq_Ramp_Gen()
        VFD_Data.CosEpsilon     = _IQcosPU(MOT_RC.RampOut);
        VFD_Data.SineEpsilon    = _IQsinPU(MOT_RC.RampOut);
                                                // Angle from Freq Ramp
        VFD_Data.CosTheta   = MOT_FSE.CosTheta;
        VFD_Data.SineTheta  = MOT_FSE.SineTheta;
                                                // Estimated Flux Angle
        Transform_Backward(VFD_Data.VsdRef, VFD_Data.VsqRef, VFD_Data.CosEpsilon, VFD_Data.SineEpsilon);
                                                // O/P => VFD_Data.VsAlphaRef, VFD_Data.VsBetaRef

    //----------------------------------------------------------------------------------------------------------------------
    // Flux & Speed Estimator
    //----------------------------------------------------------------------------------------------------------------------
        MOT_FSE.IsAlpha= VFD_Data.IsAlpha;      // I/P => VFD_Data.Isalpha:__________________#from Transform_3to2()
        MOT_FSE.IsBeta = VFD_Data.IsBeta;       // I/P => VFD_Data.Isbeta:___________________#from Transform_3to2()
        MOT_FSE.VsAlpha= VFD_Data.VsAlphaRef;   // I/P => VFD_Data.VsAlphaRef:_______________#from Transform_Backward()
        MOT_FSE.VsBeta = VFD_Data.VsBetaRef;    // I/P => VFD_Data.VsBetaRef:________________#from Transform_Backward()
        Flux_Speed_Estimator();                 //
                                                // O/P => MOT_FSE.ThetaFlux:__________________Estimated Rotor Flux Angle
                                                // O/P => MOT_FSE.WEstFilt:___________________Estimated Rotor Speed
        VFD_Data.SpeedAct = MOT_FSE.WEstFilt;   //

    //----------------------------------------------------------------------------------------------------------------------
    // Space Vector PWM Generator, from Vsalpha & Vsbeta
    //----------------------------------------------------------------------------------------------------------------------
        MOT_SVM.Alpha = VFD_Data.VsAlphaRef;    // I/P => VFD_Data.VsAlphaRef:_______________#from Transform_Backward()
        MOT_SVM.Beta  = VFD_Data.VsBetaRef;     // I/P => VFD_Data.VsBetaRef:________________#from Transform_Backward()
        THIPWM_Gen();
                                                // O/P => MOT_SVM.Ta, MOT_SVM.Tb, MOT_SVM.Tc
    }

//##########################################################################################################################
// INDM FOC - Sensorless Mode
//##########################################################################################################################
    if (VFD_Data.Con_Mode == MODE13)
    {
    //----------------------------------------------------------------------------------------------------------------------
    // Frequency Ramp Generator, depending on Speed Ref user ip (pu)
    //----------------------------------------------------------------------------------------------------------------------
        if (VFD_Status.bits.RampMode)           // Speed Rampup when Start Switch is turned on
        {
            MOT_RGI.TargVal = _IQ(0.1);
            MOT_RG.TargVal = VFD_Data.SpeedRef;
        }
        else                                    // Speed Rampdown when Start Switch is turned off
        {
            MOT_RGI.TargVal  = 0;
            MOT_RG.EqualFlag = 0;
            MOT_RG.TargVal   = 0;
        }

        Freq_Init_Ramp();

        if (MOT_RGI.EqualFlag == 0)
        {
            MOT_RG.SetValue = _IQ(0.1);
            MOT_RC.Freq     = _IQ(0.1); // I/P => MOT_RG.SetValue:___________________#from Freq_Ramp_Gen ()
        }
        else
        {
            Freq_Ramp_Gen();                        //
            MOT_RC.Freq  = MOT_RG.SetValue;
        }

    //----------------------------------------------------------------------------------------------------------------------
    // Mode Changeover Flag Conditions
    //----------------------------------------------------------------------------------------------------------------------
        if ((_IQabs(VFD_Data.SpeedRmp) > _IQ(0.3)) )
        {       // V/F Rampup to 10Hz, Change over from Epsilon(V/F) to Theta(FOC)
            VFD_Status.bits.EpsThetFlag = 1;
            VFD_Status.bits.ThetEpsFlag = 0;
        }
        if ((MOT_RG.TargVal < MOT_RG.SetValue) && (_IQabs(VFD_Data.SpeedRmp) < _IQ(0.2)) && _IQabs(MOT_RC.RampOut - MOT_FSE.ThetaFlux) <= _IQ(0.001388))
        {       // V/F Rampdown from 10Hz, Change over from Theta(FOC) to Epsilon(V/F)
            VFD_Status.bits.ThetEpsFlag = 1;
            VFD_Status.bits.EpsThetFlag = 0;
            MOT_RC.Angle                = MOT_FSE.ThetaFlux;
        }

        MOT_RC.Freq  = MOT_RG.SetValue;         // I/P => MOT_RG.SetValue:___________________#from Freq_Ramp_Gen ()
        Angle_Ramp();                           //
                                                // O/P => MOT_RC.RampOut

    //----------------------------------------------------------------------------------------------------------------------
    // V/F Profile Generator
    //----------------------------------------------------------------------------------------------------------------------
        MOT_VF.Freq  = MOT_RC.Freq;
                                                // I/P => MOT_RC.Freq:_______________________#from Freq_Ramp_Gen()
        VbyF_Gen();                             //
                                                // O/P => MOT_VF.VoltOut
        VFD_Data.SpeedRmp = MOT_VF.FreqOut;     // O/P => MOT_VF.FreqOut

    //----------------------------------------------------------------------------------------------------------------------
    // Speed PI Controller, to generate the Isq Ref
    //----------------------------------------------------------------------------------------------------------------------
    //    if (PI_Spd.LoopCnt == PI_Spd.PreScaler)
    //    {                                       // Execute Speed PI Control every Prescaler*Ts time only
            PI_Spd.Ref  = MOT_VF.FreqOut;       // I/P => MOT_VF.FreqOut:____________________#from VbyF_Gen()
            PI_Spd.Fbk  = MOT_FSE.WEstFilt;     // I/P => MOT_FSE.WEstFilt:__________________#from Flux_Speed_Estimator()

            if (VFD_Status.bits.EpsThetFlag == 1 )          // Ramp up to 10Hz by V/F mode
                if (MOT_RGQ.EqualFlag == 0x1FFF)
                    PI_Spd.Ki = PI_Spd.KiInt;
                 else
                    PI_Spd.Ki = _IQ(0.0);
            else if (VFD_Status.bits.ThetEpsFlag == 1 )     // Ramp down to 10Hz by FOC mode
                if (MOT_RGQ.EqualFlag == 0x1FFF)
                    PI_Spd.Ki = _IQ(0.0);
                else
                    PI_Spd.Ki = PI_Spd.KiInt;
            else
                PI_Spd.Ki = _IQ(0.0);


            PI_Speed();                         //
    //        PI_Spd.LoopCnt = 1;                 //
   //    }
   //     else    PI_Spd.LoopCnt ++;

        VFD_Data.IsqRef = PI_Spd.Out;           // O/P => PI_Spd.Out

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
                                                // I/P => VFD_Data.Isalpha, VFD_Data.Isbeta:_#from Transform_3to2()
                                                // I/P => VFD_Data.CosTheta:_________________#from from Flux_Speed_Estimator()
        Transform_Forward(VFD_Data.IsAlpha, VFD_Data.IsBeta, VFD_Data.CosTheta, VFD_Data.SineTheta);
                                                // Angle from Frequency Ramp can be used for tuning PI loops
                                                // Estimated angle from Flux Speed Estimator to be used
                                                // O/P => VFD_Data.Isd & VFD_Data.Isq

    //----------------------------------------------------------------------------------------------------------------------
    // Field Weakening Correction for ISD
    //----------------------------------------------------------------------------------------------------------------------
        if (_IQabs(VFD_Data.SpeedAct) > _IQ(1.0))
         {
            VFD_Data.Isd = _IQmpy(VFD_Data.Isd, _IQabs(VFD_Data.SpeedAct));// If Actual Speed > Rated Speed
                                                           // Isd = Isd*SpeedAct
         }

    //----------------------------------------------------------------------------------------------------------------------
    // Imr PI Controller, to Generate Isd Reference
    //----------------------------------------------------------------------------------------------------------------------
        if (MOT_RGQ.EqualFlag == 0x1FFF && (VFD_Data.Energy_Efficient==1))
        {
            VFD_Data.ImrRef = _IQmpy(VFD_Data.FluxRef,VFD_Data.K1) + _IQmpy(PI_Spd.Out,VFD_Data.K2) ; // Power Warping
            VFD_Data.ImrRef = _IQsat(VFD_Data.ImrRef,VFD_Data.FluxRef,PI_Imr.Umin) ;
        }
        else
            VFD_Data.ImrRef = VFD_Data.FluxRef;

           PI_Imr.Ref = VFD_Data.ImrRef;
           PI_Imr.Fbk= _IQmpy(MOT_Data.Trt_Imr,VFD_Data.Isd) + _IQmpy(MOT_Data.Trt_Imr_fbk,VFD_Data.Imr);

           VFD_Data.Imr = PI_Imr.Fbk;
           PI_Imrcurrent();

    //----------------------------------------------------------------------------------------------------------------------
    // ISD PI Controller, to generate the Vsd Ref (Vsd*)
    //----------------------------------------------------------------------------------------------------------------------
        VFD_Data.IsdRef = VFD_Data.FluxRef;//PI_Imr.Out; //_IQ(0.4);//PI_Imr.Out ; // VFD_Data.FluxRef; //
        PI_Isd.Ref      = VFD_Data.IsdRef; //      // I/P => VFD_Data.Isd:______________________#from Transform_Forward()

        if (VFD_Status.bits.EpsThetFlag == 1 )          // Ramp up to 10Hz by V/F mode
            if (MOT_RGQ.EqualFlag == 0x1FFF)
            {
                PI_Isd.Ki = PI_Isd.KiInt;
                PI_Imr.Ki = PI_Imr.KiInt;
                PI_Isq.Ki = PI_Isq.KiInt;
            }
             else
             {
                PI_Isd.Ki = _IQ(0.0);
                PI_Imr.Ki = _IQ(0.0);
                PI_Isq.Ki = _IQ(0.0);
             }
        else if (VFD_Status.bits.ThetEpsFlag == 1 )     // Ramp down to 10Hz by FOC mode
            if (MOT_RGQ.EqualFlag == 0x1FFF)
            {
                PI_Isd.Ki = _IQ(0.0);
                PI_Imr.Ki = _IQ(0.0);
                PI_Isq.Ki = _IQ(0.0);
            }
            else
            {
                PI_Isd.Ki = PI_Isd.KiInt;
                PI_Imr.Ki = PI_Imr.KiInt;
                PI_Isq.Ki = PI_Isq.KiInt;
            }
        else
        {
            PI_Isd.Ki = _IQ(0.0);
            PI_Imr.Ki = _IQ(0.0);
            PI_Isq.Ki = _IQ(0.0);
        }

        PI_Isd.Fbk  =  VFD_Data.Isd; // VFD_Data.DAxisFilt ; //   // VFD_Data.DAxisFilt; //           // I/P => VFD_Data.Isq:______________________#from Transform_Forward()
        PI_Dcurrent();                          //
        VFD_Data.VsdRef          = PI_Isd.Out;  // => PI_Isd.Out

    //----------------------------------------------------------------------------------------------------------------------
    // ISQ PI Controller, to generate the Vsq Ref
    //----------------------------------------------------------------------------------------------------------------------
        PI_Isq.Ref  = VFD_Data.IsqRef;          // I/P => VFD_Data.IsqRef:___________________#from PI_Speed()
        PI_Isq.Fbk  = VFD_Data.Isq;         // I/P => VFD_Data.Isq:______________________#from Transform_Forward()

       /* if (VFD_Status.bits.EpsThetFlag == 1 )          // Ramp up to 10Hz by V/F mode
            if (MOT_RGQ.EqualFlag == 0x1FFF)
                PI_Isq.Ki = PI_Isq.KiInt;
             else
                PI_Isq.Ki = _IQ(0.0);
        else if (VFD_Status.bits.ThetEpsFlag == 1 )     // Ramp down to 10Hz by FOC mode
            if (MOT_RGQ.EqualFlag == 0x1FFF)
                PI_Isq.Ki = _IQ(0.0);
            else
                PI_Isq.Ki = PI_Isq.KiInt;
        else
            PI_Isq.Ki = _IQ(0.0);*/

        PI_Qcurrent();                          //
        VFD_Data.VsqRef          = PI_Isq.Out;  // => PI_Isq.Out

    //----------------------------------------------------------------------------------------------------------------------
    // VSD Decoupling Control, to generate the Vsd Ref (Vsd*)
    //----------------------------------------------------------------------------------------------------------------------
                                                // I/P => MOT_FSE.WEstFilt:__________________#from Flux_Speed_Estimator()
                                                // I/P => PI_Isd.Out:________________________#from PI_Dcurrent()
                                                // I/P => VFD_Data.Isd:______________________#from Transform_Forward()
//        Vsd_Decoupling_Control();               // Vsd Decoupling Control Equation => Vsd* = Rspu*Isd + PI_Isd.Out - 1/GI(pu)*Sigma*Lspu*SpeedAct*Isq
                                                // O/P => VFD_Data.VsdRef
        VFD_Data.VsdRef = _IQsat(VFD_Data.VsdRef, VFD_Data.VsdLim, -VFD_Data.VsdLim); // VsdRef Saturation

    //----------------------------------------------------------------------------------------------------------------------
    // VSQ Decoupling Control, to generate the Vsq Ref (Vsq*)
    //----------------------------------------------------------------------------------------------------------------------
     //   VFD_Data.Imr    = 0;                    // I/P => MOT_FSE.WEstFilt:__________________#from Flux_Speed_Estimator()
                                                // I/P => PI_Isq.Out:________________________#from PI_Qcurrent()
                                                // I/P => VFD_Data.Isq:______________________#from Transform_Forward()
//        Vsq_Decoupling_Control();               // Vsq Decoupling Control Equation => Vsq* = Rspu*Isq + PI_Isq.Out + 1/GI(pu)*Lspu*Wmrpu*[Sigma*Isd - (1-Sigma)*Imr]
                                                // O/P => VFD_Data.VsqRef
        VFD_Data.VsqRef = _IQsat(VFD_Data.VsqRef, VFD_Data.VsqLim, - VFD_Data.VsqLim); // VsqRef Saturation

    //----------------------------------------------------------------------------------------------------------------------
    // VSQ & VSD Ramp Generator, to ramp Vsq & Vsd when changing from V/F to FOC
    //----------------------------------------------------------------------------------------------------------------------
       if (VFD_Status.bits.EpsThetFlag == 1)    // Initial Ramp upto 10Hz by V/F mode
           MOT_RGQ.TargVal  = _IQ(1.0);         // Ramp up from VoltOut to VsqRef

       else if (VFD_Status.bits.ThetEpsFlag == 1)
       {
           if ( ( _IQ(0.2) - _IQabs(VFD_Data.SpeedRmp))  < _IQ(0.001) && (MOT_RGQ.EqualFlag == 0x1FFF) )
               MOT_RGQ.EqualFlag    = 0;        // Reset flag before FOC to V/F changeover

           MOT_RGQ.TargVal  = _IQ(0.0);         // Ramp down from VsqRef to VoltOut
       }
       else
       {
           MOT_RGQ.EqualFlag    = 0;
           MOT_RGQ.TargVal      = 0;
       }

       MOT_RGQ.Delay            = _IQtoF(MOT_Data.Trt) * 10 + 1;
                                                // Ramptime = 0.1*Trt + 1 s
                                                // I/P => (1.0):______________#Timer set for Trt seconds
       Vsq_Ramp_Gen();                          //
                                                // O/P => MOT_RGQ.SetVal

       if (VFD_Status.bits.EpsThetFlag == 0 && VFD_Status.bits.ThetEpsFlag == 0)
           MOT_RGQ.EqualFlag    = 0;            // Reset Flag if set before changeover

    //----------------------------------------------------------------------------------------------------------------------
    // Transform D-Q User Ref Voltages to Alpha-Beta Ref Voltages
    //----------------------------------------------------------------------------------------------------------------------
        VFD_Data.VsqRefTmp = _IQsat(MOT_VF.VoltOut + _IQmpy(MOT_RGQ.SetValue, (VFD_Data.VsqRef - MOT_VF.VoltOut)), VFD_Data.VsqLim, -VFD_Data.VsqLim);
                                                // VsqRef transition => V/F Volt + (VsqRef - V/F Volt) * (0 to 1) ramp for Trt seconds
        VFD_Data.VsdRefTmp = _IQsat(_IQmpy(MOT_RGQ.SetValue, VFD_Data.VsdRef), VFD_Data.VsdLim, -VFD_Data.VsdLim);
                                                // VsdRef transition => 0 Volt + (VsdRef) * (0 to 1) ramp for Trt seconds

                                                // I/P => VFD_Data.VsdRef, VFD_Data.VsqRef:__#from Vsd_Current_Control() & Vsq_Current_Control()
                                                // I/P => MOT_RC.RampOut:____________________#from Freq_Ramp_Gen()

        VFD_Data.CosEpsilon     = _IQcosPU(MOT_RC.RampOut);
        VFD_Data.SineEpsilon    = _IQsinPU(MOT_RC.RampOut);
                                                // Angle from Freq Ramp
        VFD_Data.CosTheta   = MOT_FSE.CosTheta;
        VFD_Data.SineTheta  = MOT_FSE.SineTheta;
                                                // Estimated Flux Angle

        if (VFD_Status.bits.EpsThetFlag == 1 )          // Ramp up to 10Hz by V/F mode
            if (MOT_RGQ.EqualFlag == 0x1FFF)
                Transform_Backward(VFD_Data.VsdRef, VFD_Data.VsqRef, VFD_Data.CosTheta, VFD_Data.SineTheta);
            else
                Transform_Backward(VFD_Data.VsdRefTmp, VFD_Data.VsqRefTmp, VFD_Data.CosTheta, VFD_Data.SineTheta);

        else if (VFD_Status.bits.ThetEpsFlag == 1 )     // Ramp down to 10Hz by FOC mode
            if (MOT_RGQ.EqualFlag == 0x1FFF)
                Transform_Backward(_IQ(0.0), MOT_VF.VoltOut, VFD_Data.CosEpsilon, VFD_Data.SineEpsilon);
            else
                Transform_Backward(VFD_Data.VsdRefTmp, VFD_Data.VsqRefTmp, VFD_Data.CosEpsilon, VFD_Data.SineEpsilon);
        else
            Transform_Backward(_IQ(0.0), MOT_VF.VoltOut, VFD_Data.CosEpsilon, VFD_Data.SineEpsilon);
                                                // O/P => VFD_Data.VsAlphaRef, VFD_Data.VsBetaRef

    //----------------------------------------------------------------------------------------------------------------------
    // Flux & Speed Estimator
    //----------------------------------------------------------------------------------------------------------------------
        MOT_FSE.IsAlpha=  VFD_Data.IsAlpha;      // I/P => VFD_Data.Isalpha:__________________#from Transform_3to2()
        MOT_FSE.IsBeta =  VFD_Data.IsBeta;       // I/P => VFD_Data.Isbeta:___________________#from Transform_3to2()
        MOT_FSE.VsAlpha= VFD_Data.VsAlphaRef;   // I/P => MOT_VSI.Valpha:____________________#from Transform_Backward()
        MOT_FSE.VsBeta = VFD_Data.VsBetaRef;    // I/P => MOT_VSI.Vbeta:_____________________#from Transform_Backward()
        Flux_Speed_Estimator();                 //
                                                // O/P => MOT_FSE.ThetaFlux:__________________Estimated Rotor Flux Angle
                                                // O/P => MOT_FSE.WEstFilt:___________________Estimated Rotor Speed
        VFD_Data.SpeedAct = MOT_FSE.WEstFilt;   //
    //----------------------------------------------------------------------------------------------------------------------
    // Space Vector PWM Generator, from Vsalpha & Vsbeta
    //----------------------------------------------------------------------------------------------------------------------
        MOT_SVM.Alpha = VFD_Data.VsAlphaRef;    // I/P => VFD_Data.VsAlphaRef:_______________#from Transform_Backward()
        MOT_SVM.Beta  = VFD_Data.VsBetaRef;     // I/P => VFD_Data.VsBetaRef:________________#from Transform_Backward()
        THIPWM_Gen();                           //
                                                // O/P => MOT_SVM.Ta, MOT_SVM.Tb, MOT_SVM.Tc

    }


//##########################################################################################################################
// PMSM FOC - CURRENT MODE
//##########################################################################################################################
    if (VFD_Data.Con_Mode == MODE21)
    {
    //----------------------------------------------------------------------------------------------------------------------
    // Frequency Ramp Generator, depending on Speed Ref user ip (pu)
    //----------------------------------------------------------------------------------------------------------------------
    if (VFD_Status.bits.RampMode)           // Speed Rampup when Start Switch is turned on
      {
          MOT_RGI.TargVal = _IQ(.05);
          MOT_RG.TargVal   = VFD_Data.SpeedRef;
      }
      else                                    // Speed Rampdown when Start Switch is turned off
      {
           MOT_RGI.TargVal= 0;
           MOT_RG.EqualFlag = 0;
           MOT_RG.TargVal   = 0;
      }

       Freq_Init_Ramp();
                                          // I/P => VFD_Data.SpeedRef:_________________#from User i/p Speed Ref (pu)
       if(MOT_RGI.EqualFlag==0)
          {
          MOT_RG.SetValue = _IQ(.05);
          MOT_RC.Freq     = MOT_RGI.SetValue;  // I/P => MOT_RG.SetValue:___________________#from Freq_Ramp_Gen ()
          }
          else
          {
          Freq_Ramp_Gen();
          MOT_RC.Freq = MOT_RG.SetValue; // I/P => MOT_RG.SetValue:___________________#from Freq_Ramp_Gen ()
          }//
                                                        // O/P => MOT_RG.SetValue
        Angle_Ramp();                                   //
        VFD_Data.SpeedRmp   = MOT_RC.Freq;              // O/P => MOT_RC.Freq, MOT_RC.RampOut

    //----------------------------------------------------------------------------------------------------------------------
    // Flux Ramp Generator(pu)
    //----------------------------------------------------------------------------------------------------------------------
        if (VFD_Status.bits.RampMode)                   // Flux Rampup when Start Switch is turned on
        {
            MOT_RGD.TargVal = VFD_Data.FluxRef;         // I/P => VFD_Data.FluxRef:__________________#from User i/p Flux Ref (pu)
            MOT_RGD.Delay   = MOT_RG.Delay*(1);         // Should be <= MOT_RG.Delay)
                                                        //           > 0.1s
        }
        else                                            // Flux Rampdown when Start Switch is turned off
        {
            MOT_RGD.EqualFlag= 0;
            MOT_RGD.TargVal  = 0;
        }

        Vsd_Ramp_Gen();                                 //
        VFD_Data.IsdRef      = MOT_RGD.SetValue;        // O/P => MOT_RGD.SetVal

    //----------------------------------------------------------------------------------------------------------------------
    // Torq Ramp Generator(pu)
    //----------------------------------------------------------------------------------------------------------------------
        if (VFD_Status.bits.RampMode)                   // Torq Rampup when Start Switch is turned on
        {
            MOT_RGQ.TargVal = VFD_Data.TorqRef;         // I/P => VFD_Data.TorqRef:__________________#from User i/p Torq Ref (pu)
            MOT_RGQ.Delay   = MOT_RG.Delay*(1);         // Should be <= MOT_RG.Delay)
                                                        //           > 0.1s
        }
        else                                            // Flux Rampdown when Start Switch is turned off
        {
            MOT_RGQ.EqualFlag= 0;
            MOT_RGQ.TargVal  = 0;
        }

    //    Vsq_Ramp_Gen();
        VFD_Data.IsqRef      =   MOT_RGQ.TargVal;//MOT_RGQ.SetValue;
//        VFD_Data.IsqRef      = _IQ(0.0);                // Torque Ref = 0.0

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
       VFD_Data.CosEpsilon      = _IQcosPU(MOT_RC.RampOut);
       VFD_Data.SineEpsilon     = _IQsinPU(MOT_RC.RampOut);
                                                    // I/P => VFD_Data.Isalpha, VFD_Data.Isbeta:_#from Transform_3to2()
                                                    // I/P => VFD_Data.CosEpsilon:_______________#from from Flux_Speed_Estimator()
       Transform_Forward(VFD_Data.IsAlpha, VFD_Data.IsBeta, VFD_Data.CosEpsilon, VFD_Data.SineEpsilon);
                                                    // O/P => VFD_Data.Isd & VFD_Data.Isq

    //----------------------------------------------------------------------------------------------------------------------
    // ISD PI Controller, to generate the Vsd Ref (Vsd*)
    //----------------------------------------------------------------------------------------------------------------------
       PI_Isd.Ref               = VFD_Data.IsdRef;  // I/P => VFD_Data.FluxRef:__________________#from User i/p Flux Ref (pu)
       PI_Isd.Fbk               = VFD_Data.Isd;     // I/P => VFD_Data.Isd:______________________#from Transform_Forward()
       PI_Dcurrent();                               //
       VFD_Data.VsdRef          = PI_Isd.Out;       // => PI_Isd.Out

    //----------------------------------------------------------------------------------------------------------------------
    // ISQ PI Controller, to generate the Vsq Ref (Vsq*)
    //----------------------------------------------------------------------------------------------------------------------
       PI_Isq.Ref               = VFD_Data.IsqRef;  // I/P => VFD_Data.IsqRef:___________________#from PI_Speed()
       PI_Isq.Fbk               = VFD_Data.Isq;     // I/P => VFD_Data.Isq:______________________#from Transform_Forward()
       PI_Qcurrent();                               //
       VFD_Data.VsqRef          = PI_Isq.Out;       // => PI_Isq.Out

   //----------------------------------------------------------------------------------------------------------------------
   // VSD Decoupling Control, to generate the Vsd Ref (Vsd*)
   //----------------------------------------------------------------------------------------------------------------------
                                           // I/P => MOT_FSE.WEstFilt:__________________#from Flux_Speed_Estimator()
                                           // I/P => PI_Isd.Out:________________________#from PI_Dcurrent()
                                           // I/P => VFD_Data.Isd:______________________#from Transform_Forward()
//      Vsd_Decoupling_Control();            // Vsd Decoupling Control Equation => Vsd* = Rspu*Isd + PI_Isd.Out - 1/GI(pu)*Ldpu*Wmrpu*Isq
                                           // O/P => VFD_Data.VsdRef

   //----------------------------------------------------------------------------------------------------------------------
   // VSQ Decoupling Control, to generate the Vsq Ref (Vsq*)
   //----------------------------------------------------------------------------------------------------------------------
                                           // I/P => MOT_FSE.WEstFilt:__________________#from Flux_Speed_Estimator()
                                           // I/P => PI_Isq.Out:________________________#from PI_Qcurrent()
                                           // I/P => VFD_Data.Isq:______________________#from Transform_Forward()
// 	    Vsq_Decoupling_Control(); 		     // Vsq Decoupling Control Equation => Vsq* = Rspu*Isq + PI_Isq.Out + 1/GI(pu)*Ldpu*Wmrpu*Isd + Wmrpu*Lmdf
                                           // O/P => VFD_Data.VsqRef

   //----------------------------------------------------------------------------------------------------------------------
   // Transform D-Q User Ref Voltages to Alpha-Beta Ref Voltages
   //----------------------------------------------------------------------------------------------------------------------
       Transform_Backward(VFD_Data.VsdRef, VFD_Data.VsqRef, VFD_Data.CosEpsilon, VFD_Data.SineEpsilon);

   //----------------------------------------------------------------------------------------------------------------------
   // Flux & Speed Estimator
   //----------------------------------------------------------------------------------------------------------------------
       MOT_FSE.IsAlpha     = VFD_Data.IsAlpha;      // I/P => VFD_Data.Isalpha:__________________#from Transform_3to2()
       MOT_FSE.IsBeta      = VFD_Data.IsBeta;       // I/P => VFD_Data.Isbeta:___________________#from Transform_3to2()
       MOT_FSE.VsAlpha     = VFD_Data.VsAlphaRef;   // I/P => MOT_VSI.Valpha:____________________#from Transform_Backward()
       MOT_FSE.VsBeta      = VFD_Data.VsBetaRef;    // I/P => MOT_VSI.Vbeta:_____________________#from Transform_Backward()

       Flux_Speed_Estimator();                      //
                                                    // O/P => MOT_FSE.ThetaFlux:__________________Estimated Rotor Flux Angle
                                                    // O/P => MOT_FSE.WEstFilt:___________________Estimated Rotor Speed

       VFD_Data.SpeedAct    = MOT_FSE.WEstFilt;
       VFD_Data.CosTheta    = MOT_FSE.CosTheta;
       VFD_Data.SineTheta   = MOT_FSE.SineTheta;

    //----------------------------------------------------------------------------------------------------------------------
    // Space Vector PWM Generator, from Vsalpha & Vsbeta
    //----------------------------------------------------------------------------------------------------------------------
       MOT_SVM.Alpha   = MOT_FSE.VsAlpha;  // I/P => VFD_Data.VsAlphaRef:_______________#from Flux_Speed_Estimator_PMSM()
       MOT_SVM.Beta    = MOT_FSE.VsBeta;   // I/P => VFD_Data.VsBetaRef:________________#from Flux_Speed_Estimator_PMSM()

       THIPWM_Gen();                       // PWM Generation
                                           // O/P => MOT_SVM.Ta, MOT_SVM.Tb, MOT_SVM.Tc
    }
//##########################################################################################################################
// PMSM FOC - SENSORLESS MODE
//##########################################################################################################################
    if (VFD_Data.Con_Mode == MODE22)
    {
    //----------------------------------------------------------------------------------------------------------------------
    // Frequency Ramp Generator, depending on Speed Ref user ip (pu)
    //----------------------------------------------------------------------------------------------------------------------
       if (VFD_Status.bits.RampMode)           // Speed Rampup when Start Switch is turned on
       {
           MOT_RGI.TargVal = _IQ(0.05);
           MOT_RG.TargVal  = VFD_Data.SpeedRef;
       }
       else                                    // Speed Rampdown when Start Switch is turned off
       {
            MOT_RGI.TargVal  = 0;
            MOT_RG.EqualFlag = 0;
            MOT_RG.TargVal   = 0;
       }

       Freq_Init_Ramp();
                                           // I/P => VFD_Data.SpeedRef:_________________#from User i/p Speed Ref (pu)
       if (MOT_RGI.EqualFlag == 0)
       {
           MOT_RG.SetValue = _IQ(0.05);
           MOT_RC.Freq     = MOT_RGI.SetValue;  // I/P => MOT_RG.SetValue:___________________#from Freq_Ramp_Gen ()
       }
       else
       {
           Freq_Ramp_Gen();
           MOT_RC.Freq = MOT_RG.SetValue;
       }

                                               // O/P => MOT_RG.SetValue
       Angle_Ramp();                           //
       VFD_Data.SpeedRmp   = MOT_RC.Freq;      // O/P => MOT_RC.Freq, MOT_RC.RampOut

    //----------------------------------------------------------------------------------------------------------------------
    // Flux Ramp Generator(pu)
    //----------------------------------------------------------------------------------------------------------------------
       if (VFD_Status.bits.RampMode)            // Flux Rampup when Start Switch is turned on
       {
            MOT_RGD.TargVal = VFD_Data.FluxRef; // I/P => VFD_Data.FluxRef:__________________#from User i/p Flux Ref (pu)
            MOT_RGD.Delay   = MOT_RG.Delay*(1); // Should be <= MOT_RG.Delay)
                                                //           > 0.1s
       }
       else                                    // Flux Rampdown when Start Switch is turned off
       {
            MOT_RGD.EqualFlag= 0;
            MOT_RGD.TargVal  = 0;
       }

   //     Vsd_Ramp_Gen();                        //
       VFD_Data.IsdRef      = MOT_RGD.TargVal;// O/P => MOT_RGD.SetValue

    //----------------------------------------------------------------------------------------------------------------------
    // Torq Ramp Generator(pu)
    //----------------------------------------------------------------------------------------------------------------------
        if (VFD_Status.bits.RampMode)                   // Torq Rampup when Start Switch is turned on
        {
               // IVFD_Data.TorqRef;/P => VFD_Data.TorqRef:__________________#from User i/p Torq Ref (pu)
            if(PI_Spd.Ref > _IQ(0.2) && (VFD_Status.bits.EpsThetFlag==0))
            {
            if(Q_Decrement_Flag ==0)
            MOT_RGQ.SetValue += _IQmpy(VFD_Data.Ts, _IQ(.25));
            if((VFD_Data.Torqlower) > _IQ(0.15))
            Q_Decrement_Flag =1;
            VFD_Data.TorqRef = _IQ(User_Data.TorqStrt)- MOT_RGQ.SetValue;
            }

            MOT_RGQ.TargVal = VFD_Data.TorqRef;

            MOT_RGQ.Delay   = MOT_RG.Delay*(1);         // Should be <= MOT_RG.Delay)
        }                                         //           > 0.1s
        else                                            // Flux Rampdown when Start Switch is turned off
        {
            MOT_RGQ.EqualFlag= 0;
            MOT_RGQ.TargVal  = 0;
        }

   //     Vsq_Ramp_Gen();

    //----------------------------------------------------------------------------------------------------------------------
    // Mode Changeover Flag Conditions
    //----------------------------------------------------------------------------------------------------------------------
        if ( (MOT_RG.TargVal < MOT_RG.SetValue) && (VFD_Data.SpeedAct < _IQ(0.02)) )
            VFD_Status.bits.ThetEpsFlag = 1;    // Change over from Theta to Epsilon during Rampdown and Speed < 0.02
// 0.002776
        if  (((VFD_Data.SpeedAct >= _IQ(0.2))) && _IQabs(MOT_FSE.ThetaFlux - MOT_RC.RampOut) <= _IQ(0.000694)) // 0.001388
            VFD_Status.bits.EpsThetFlag = 1;    // Angle change from Epsilon to Theta condition: - // Actual Speed >= 0.1   // Sync Window of 0.5 degree => 0.5/360, _IQ(0.001388)

    //----------------------------------------------------------------------------------------------------------------------
    // Transform measured ABC currents to Alpha-Beta currents
    //----------------------------------------------------------------------------------------------------------------------
                                            // I/P => VFD_Data.Isa, VFD_Data.Isb:________#from EXT ADC
        Transform_3to2(VFD_Data.Isa, VFD_Data.Isb, VFD_Data.Isc);
                                            //
                                            // O/P => VFD_Data.Isalpha & VFD_Data.Isbeta
        //----------------------------------------------------------------------------------------------------------------------

        if (VFD_Status.bits.EpsThetFlag == 0)
        {   // Motor to run wit Epsilon Angle, wit IsQ & IsD Current & w/o PI Speed Controllers
             PI_Spd.i1       = _IQ(0.0);         // RESET Speed I-CONTROLLER
             PI_Spd.Ki       = _IQ(0.0);

     /*        if(VFD_Data.SpeedAct < _IQ(0.2))// Introduced for smooth turn off during Stop condition
             VFD_Data.IsqRef = MOT_RGQ.TargVal;//PI_Spd.Out;
             else
             VFD_Data.IsqRef = MOT_RGQ.SetValue;*/
             VFD_Data.IsqRef = MOT_RGQ.TargVal;//MOT_RGQ.SetValue; // PMSM Torq Ref is Zero
             VFD_Data.IsdRef = MOT_RGD.TargVal; // PMSM Flux Ref is Min

             PI_Spd.Ref      = MOT_RC.Freq;      // I/P => MOT_RC.Freq:________________________#from Freq_Ramp_Gen()
             PI_Spd.Fbk      = MOT_FSE.WEstFilt; // I/P => MOT_FSE.WEstFilt:___________________#from Flux_Speed_Estimator()
             PI_Speed();                         //
                                                 // O/P => PI_Spd.Out

        //----------------------------------------------------------------------------------------------------------------------
        // Transform measured Alpha-Beta currents to D-Q currents
        //----------------------------------------------------------------------------------------------------------------------
             VFD_Data.CosEpsilon     = _IQcosPU(MOT_RC.RampOut);
             VFD_Data.SineEpsilon    = _IQsinPU(MOT_RC.RampOut);
             Transform_Forward(VFD_Data.IsAlpha, VFD_Data.IsBeta, VFD_Data.CosEpsilon, VFD_Data.SineEpsilon);
        }
        else
        {
        //----------------------------------------------------------------------------------------------------------------------
        // Transform measured Alpha-Beta currents to D-Q currents
        //----------------------------------------------------------------------------------------------------------------------
           if (VFD_Status.bits.ThetEpsFlag == 1)
           {   // Motor to run wit Epsilon Angle
               VFD_Data.CosEpsilon = _IQcosPU(MOT_RC.RampOut);
               VFD_Data.SineEpsilon= _IQsinPU(MOT_RC.RampOut);
                                                   // I/P => VFD_Data.Isalpha, VFD_Data.Isbeta:_#from Transform_3to2()
                                                   // I/P => VFD_Data.CosEpsilon:_______________#from from from Freq_Ramp_Gen()
               Transform_Forward(VFD_Data.IsAlpha, VFD_Data.IsBeta, VFD_Data.CosEpsilon, VFD_Data.SineEpsilon);
                                                   // O/P => VFD_Data.Isd & VFD_Data.Isq
           }
           else
           {   // Motor to run wit Theta Angle
               VFD_Data.CosTheta    = _IQcosPU(MOT_FSE.ThetaFlux);
               VFD_Data.SineTheta   = _IQsinPU(MOT_FSE.ThetaFlux);
                                                    // I/P => VFD_Data.Isalpha, VFD_Data.Isbeta:_#from Transform_3to2()
                                                    // I/P => VFD_Data.CosTheta:_________________#from from Flux_Speed_Estimator()
               Transform_Forward(VFD_Data.IsAlpha, VFD_Data.IsBeta, VFD_Data.CosTheta, VFD_Data.SineTheta);
                                                    // O/P => VFD_Data.Isd & VFD_Data.Isq
            }

       //----------------------------------------------------------------------------------------------------------------------
       // Speed PI Controller, to generate the Isq Ref
       //----------------------------------------------------------------------------------------------------------------------
           PI_Spd.Kp       = PI_Spd.KpInt;
           PI_Spd.Ki       = PI_Spd.KiInt;

           PI_Spd.Ref      = MOT_RC.Freq;          // I/P => MOT_RC.Freq:________________________#from Freq_Ramp_Gen()
           PI_Spd.Fbk      = MOT_FSE.WEstFilt;     // I/P => MOT_FSE.WEstFilt:___________________#from Flux_Speed_Estimator()
           PI_Speed();

        //   MOT_RGD.TargVal = 0;
        //   MOT_RGD.Delay = 20;
        //   Vsd_Ramp_Gen();                        //
           VFD_Data.IsdRef      = MOT_RGD.TargVal;// O/P => MOT_RGD.SetValue
     //      VFD_Data.IsdRef      = _IQ(0.0);



           if (VFD_Status.bits.ThetEpsFlag == 1)
               VFD_Data.IsqRef = MOT_RGQ.SetValue; // Motor to run wit Epsilon Angle, wit IsQ & IsD Current & w/o PI Speed Controllers
           else
               VFD_Data.IsqRef = PI_Spd.Out;



           if (_IQabs(MOT_FSE.ThetaFlux-MOT_RC.RampOut)<=_IQ(0.0013888) && (VFD_Status.bits.ThetEpsFlag == 0))    //Sync. Window is 0.5 degree And Not on Deccelaration
              MOT_RC.Angle     = MOT_FSE.ThetaFlux;    // Synchronisation with estimated Rotor flux
        }

    //----------------------------------------------------------------------------------------------------------------------
    // ISD PI Controller, to generate the Vsd Ref (Vsd*)
    //----------------------------------------------------------------------------------------------------------------------
        PI_Isd.Ref  = VFD_Data.IsdRef;  // I/P => VFD_Data.FluxRef:__________________#from User i/p Flux Ref (pu)
        PI_Isd.Fbk  = VFD_Data.Isd;     // I/P => VFD_Data.Isq:______________________#from Transform_Forward()
        PI_Dcurrent();                  //
        VFD_Data.VsdRef = PI_Isd.Out;   // => PI_Isd.Out

    //----------------------------------------------------------------------------------------------------------------------
    // ISQ PI Controller, to generate the Vsq Ref
    //----------------------------------------------------------------------------------------------------------------------
        PI_Isq.Ref  = VFD_Data.IsqRef;  // I/P => VFD_Data.IsqRef:___________________#from PI_Speed()
        PI_Isq.Fbk  = VFD_Data.Isq;     // I/P => VFD_Data.Isq:______________________#from Transform_Forward()
        PI_Qcurrent();                  //
        VFD_Data.VsqRef = PI_Isq.Out;   // => PI_Isq.Out

    //----------------------------------------------------------------------------------------------------------------------
    // VSD Decoupling Control, to generate the Vsd Ref (Vsd*)
    //----------------------------------------------------------------------------------------------------------------------
                                            // I/P => MOT_FSE.WEstFilt:__________________#from Flux_Speed_Estimator()
                                            // I/P => PI_Isd.Out:________________________#from PI_Dcurrent()
                                            // I/P => VFD_Data.Isd:______________________#from Transform_Forward()
 //      Vsd_Decoupling_Control();            // Vsd Decoupling Control Equation => Vsd* = Rspu*Isd + PI_Isd.Out - 1/GI(pu)*Ldpu*Wmrpu*Isq
                                            // O/P => VFD_Data.VsdRef

    //----------------------------------------------------------------------------------------------------------------------
    // VSQ Decoupling Control, to generate the Vsq Ref (Vsq*)
    //----------------------------------------------------------------------------------------------------------------------
                                            // I/P => MOT_FSE.WEstFilt:__________________#from Flux_Speed_Estimator()
                                            // I/P => PI_Isq.Out:________________________#from PI_Qcurrent()
                                            // I/P => VFD_Data.Isq:______________________#from Transform_Forward()
 //      Vsq_Decoupling_Control();            // Vsq Decoupling Control Equation => Vsq* = Rspu*Isq + PI_Isq.Out + 1/GI(pu)*Ldpu*Wmrpu*Isd + Wmrpu*Lmdf
                                            // O/P => VFD_Data.VsqRef


    //----------------------------------------------------------------------------------------------------------------------
    // Transform D-Q User Ref Voltages to Alpha-Beta Ref Voltages
    //----------------------------------------------------------------------------------------------------------------------
        if ((VFD_Status.bits.EpsThetFlag == 1 ) &&  ( VFD_Status.bits.ThetEpsFlag == 0))
            Transform_Backward(VFD_Data.VsdRef, VFD_Data.VsqRef, VFD_Data.CosTheta, VFD_Data.SineTheta);
        else
            Transform_Backward(VFD_Data.VsdRef, VFD_Data.VsqRef, VFD_Data.CosEpsilon, VFD_Data.SineEpsilon);

    //----------------------------------------------------------------------------------------------------------------------
    // Flux & Speed Estimator
    //----------------------------------------------------------------------------------------------------------------------
        MOT_FSE.IsAlpha     = VFD_Data.IsAlpha;     // I/P => VFD_Data.Isalpha:__________________#from Transform_3to2()
        MOT_FSE.IsBeta      = VFD_Data.IsBeta;      // I/P => VFD_Data.Isbeta:___________________#from Transform_3to2()
        MOT_FSE.VsAlpha     = VFD_Data.VsAlphaRef;  // I/P => MOT_VSI.Valpha:____________________#from Transform_Backward()
        MOT_FSE.VsBeta      = VFD_Data.VsBetaRef;   // I/P => MOT_VSI.Vbeta:_____________________#from Transform_Backward()

        Flux_Speed_Estimator();                     //
                                                    // O/P => MOT_FSE.ThetaFlux:__________________Estimated Rotor Flux Angle
        VFD_Data.SpeedAct    = MOT_FSE.WEstFilt;    // O/P => MOT_FSE.WEstFilt:___________________Estimated Rotor Speed
        VFD_Data.CosTheta    = MOT_FSE.CosTheta;
        VFD_Data.SineTheta   = MOT_FSE.SineTheta;

    //----------------------------------------------------------------------------------------------------------------------
    // Space Vector PWM Generator, from Vsalpha & Vsbeta
    //----------------------------------------------------------------------------------------------------------------------
        MOT_SVM.Alpha   = MOT_FSE.VsAlpha;  // I/P => VFD_Data.VsAlphaRef:_______________#from Flux_Speed_Estimator_PMSM()
        MOT_SVM.Beta    = MOT_FSE.VsBeta;   // I/P => VFD_Data.VsBetaRef:________________#from Flux_Speed_Estimator_PMSM()

        THIPWM_Gen();                       // PWM Generation
                                            // O/P => MOT_SVM.Ta, MOT_SVM.Tb, MOT_SVM.Tc

    }

//--------------------------------------------------------------------------------------------------------------------------
// PWM Generation Section
//--------------------------------------------------------------------------------------------------------------------------
    EPWM_Generator();   // Generate PWM from DSP EPWM module with the PWM reference signals

}

//--------------------------------------------------------------------------------------------------------------------------
// END OF MOTOR Functions
//--------------------------------------------------------------------------------------------------------------------------

