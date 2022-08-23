//==================================================================================================================================================
// FILE NAME        : Funcs_4_MotorFunctions.c
// DATE             : 01-Feb-2018
// Project          : VARIABLE FREQUENCY DRIVE FOR COMPRESSOR CONTROL APPLICATIONS
// Project Code     : PEG 124B
// Author           : Rohit V Thomas[ELGI], Manju R[CDAC] & Prashobh[CDAC]
//==================================================================================================================================================
// Include Header Files....
//==============================================================================================================================
#include "DSP28234_Device.h"                                // Includes all header files
#include "Variable.h"                                       // Include the Variables
#include "DSP28234_GlobalPrototypes.h"
#include "IQmathLib.h"

//==============================================================================================================================
// Dynamic Brake Control Function
//==============================================================================================================================

void Dyn_Brake_Con()
{
// Compute the angle rate
    Dyn_Con.Angle    += _IQmpy(Dyn_Con.StepAng, Dyn_Con.Freq);

    // Saturate the angle rate within (-1,1)
    if (Dyn_Con.Angle > _IQ(1.0))
        Dyn_Con.Angle -= _IQ(1.0);
    else if (Dyn_Con.Angle < _IQ(-1.0))
        Dyn_Con.Angle += _IQ(1.0);
//--------------------------------------------------------------------------------------------------------------------------
    Dyn_Con.RampOut              = Dyn_Con.Angle;     // Ramp Output at the Set Frequency
//--------------------------------------------------------------------------------------------------------------------------

    if (Dyn_Con.RampOut >= _IQ(0.0) && Dyn_Con.RampOut <= VFD_Data.DynBrkDuty)
    {
        DYN_BRK_Ctrl     = 0;       // Turning On Dyn Brake PWM at DynBrakeDuty, pulse count limited by 6 for 10Hz

        VFD_Limits.DB_Pulse_Cnt++;
    }
    else
        DYN_BRK_Ctrl     = 1;       // Turning Off Dyn Brake PWM

}
//=========================================================================================================================
// Initial Frequency Ramp Generation
//=========================================================================================================================
void Freq_Init_Ramp()
{
    MOT_RGI.Tmp = MOT_RGI.TargVal - MOT_RGI.SetValue;
    if (_IQabs(MOT_RGI.Tmp) >= _IQmpy(VFD_Data.Ts, _IQ(1.0))) // Min Ramp time = 1s, dt = Ts
    {
        MOT_RGI.DelayCnt++;
        if (MOT_RGI.DelayCnt >= MOT_RGI.Delay)    // Delay value sets No of times min ramp time; eg - 10*1ms = 10ms
        {
            if (MOT_RGI.TargVal >= MOT_RGI.SetValue)
                MOT_RGI.SetValue +=_IQmpy(VFD_Data.Ts, _IQ(1.0));    //VFD_Data.Ts;    // Min Ramp time = 1s, dt = Ts
            else
                MOT_RGI.SetValue -= _IQmpy(VFD_Data.Ts, _IQ(1.0));   //VFD_Data.Ts;    // Min Ramp time = 1s, dt = Ts
            MOT_RGI.SetValue      = _IQsat(MOT_RGI.SetValue,MOT_RGI.HighLim,MOT_RGI.LowLim);
            MOT_RGI.DelayCnt     = 0;
        }
    }
    else MOT_RGI.EqualFlag   = 0x1FFF;
}

//=========================================================================================================================
// Voltage Ramp Generation Function
//=========================================================================================================================
void Freq_Ramp_Gen()
{
    MOT_RG.Tmp = MOT_RG.TargVal - MOT_RG.SetValue;
    if (_IQabs(MOT_RG.Tmp) >= _IQmpy(VFD_Data.Ts, _IQ(1.0))) // Min Ramp time = 1s, dt = Ts
    {
        MOT_RG.DelayCnt++;
        if (MOT_RG.DelayCnt >= MOT_RG.Delay)    // Delay value sets No of times min ramp time; eg - 10*1ms = 10ms
        {
            if (MOT_RG.TargVal >= MOT_RG.SetValue)
                MOT_RG.SetValue +=_IQmpy(VFD_Data.Ts, _IQ(1.0));    //VFD_Data.Ts;    // Min Ramp time = 1s, dt = Ts
            else
                MOT_RG.SetValue -= _IQmpy(VFD_Data.Ts, _IQ(1.0));   //VFD_Data.Ts;    // Min Ramp time = 1s, dt = Ts
            MOT_RG.SetValue      = _IQsat(MOT_RG.SetValue,MOT_RG.HighLim,MOT_RG.LowLim);
            MOT_RG.DelayCnt     = 0;
        }
    }
    else MOT_RG.EqualFlag   = 0x1FFF;
}

//=========================================================================================================================
// Ramped Angle Generation Function
//=========================================================================================================================
void Angle_Ramp()
{
    // Compute the angle rate
    MOT_RC.Angle    += _IQmpy(MOT_RC.StepAng,MOT_RC.Freq);

    // Saturate the angle rate within (-1,1)
    if (MOT_RC.Angle > _IQ(1.0))
        MOT_RC.Angle -= _IQ(1.0);
    else if (MOT_RC.Angle < _IQ(-1.0))
        MOT_RC.Angle += _IQ(1.0);
//--------------------------------------------------------------------------------------------------------------------------
    MOT_RC.RampOut              = MOT_RC.Angle;     // Ramp Output at the Set Frequency
//--------------------------------------------------------------------------------------------------------------------------
}
//=========================================================================================================================
// VSD Ramp Generation Function
//=========================================================================================================================
void Vsd_Ramp_Gen()
{
    MOT_RGD.Tmp = MOT_RGD.TargVal - MOT_RGD.SetValue;
    if (_IQabs(MOT_RGD.Tmp) >= _IQmpy(VFD_Data.Ts, _IQ(10.0))) // Min Ramp time = 0.1s, dt = Ts
    {
        MOT_RGD.DelayCnt++;
        if (MOT_RGD.DelayCnt >= MOT_RGD.Delay)  // Delay value sets No of times min ramp time; eg - 10*0.1s = 1s
        {
            if (MOT_RGD.TargVal >= MOT_RGD.SetValue)
                MOT_RGD.SetValue += _IQmpy(VFD_Data.Ts, _IQ(10.0));   // Min Ramp time = 0.1s, dt = Ts
            else
                MOT_RGD.SetValue -= _IQmpy(VFD_Data.Ts, _IQ(10.0));   // Min Ramp time = 0.1s, dt = Ts

            MOT_RGD.SetValue     = _IQsat(MOT_RGD.SetValue,MOT_RGD.HighLim,MOT_RGD.LowLim);
            MOT_RGD.DelayCnt     = 0;
        }
    }
    else MOT_RGD.EqualFlag   = 0x1FFF;
}
//=========================================================================================================================
// VSQ Ramp Generation Function
//=========================================================================================================================
void Vsq_Ramp_Gen()
{
    MOT_RGQ.Tmp = MOT_RGQ.TargVal - MOT_RGQ.SetValue;
    if (_IQabs(MOT_RGQ.Tmp) >= _IQmpy(VFD_Data.Ts, _IQ(10.0))) // Min Ramp time = 0.1s, dt = Ts
    {
        MOT_RGQ.DelayCnt++;
        if (MOT_RGQ.DelayCnt >= MOT_RGQ.Delay)  // Delay value sets No of times min ramp time; eg - 10*0.1s = 1s
        {
            if (MOT_RGQ.TargVal >= MOT_RGQ.SetValue)
                MOT_RGQ.SetValue += _IQmpy(VFD_Data.Ts, _IQ(10.0));   // Min Ramp time = 0.1s, dt = Ts
            else
                MOT_RGQ.SetValue -= _IQmpy(VFD_Data.Ts, _IQ(10.0));   // Min Ramp time = 0.1s, dt = Ts

            MOT_RGQ.SetValue     = _IQsat(MOT_RGQ.SetValue,MOT_RGQ.HighLim,MOT_RGQ.LowLim);
            MOT_RGQ.DelayCnt     = 0;
        }
    }
    else MOT_RGQ.EqualFlag   = 0x1FFF;
}
//=========================================================================================================================
// Voltage / Frequency Profile Function
//=========================================================================================================================
void VbyF_Gen()
{
    MOT_VF.AbsFreq = labs(MOT_VF.Freq);       // Taking absolute frequency to allow the operation of both rotational directions
    if (MOT_VF.AbsFreq <= MOT_VF.FreqMin)                                             // Compute output voltage in profile #1
    {
        MOT_VF.VoltOut = MOT_VF.VoltMin;
        MOT_VF.FreqOut = MOT_VF.FreqMin;
    }
    else if ((MOT_VF.AbsFreq > MOT_VF.FreqMin) && (MOT_VF.AbsFreq <= MOT_VF.FreqRated)) // Compute output voltage in profile #2
    {
        MOT_VF.VoltOut = MOT_VF.VoltMin + _IQmpy(MOT_VF.VfSlope, (MOT_VF.AbsFreq - MOT_VF.FreqMin));
        MOT_VF.FreqOut = MOT_VF.Freq;
    }
    else if ((MOT_VF.AbsFreq > MOT_VF.FreqRated) && (MOT_VF.AbsFreq < MOT_VF.FreqMax))  // Compute output voltage in profile #3
    {
        MOT_VF.VoltOut = MOT_VF.VoltRated;  // Limit o/p Voltage to Rated
        MOT_VF.FreqOut = MOT_VF.Freq;
    }
    else if (MOT_VF.AbsFreq > MOT_VF.FreqMax)                                           // Compute output voltage in profile #4
    {
        MOT_VF.VoltOut = MOT_VF.VoltRated;  // Limit o/p Voltage to Rated, Freq to Max allowed
        MOT_VF.FreqOut = MOT_VF.FreqMax;
    }
}
//=========================================================================================================================
// Flux & Speed Estimator Function
//=========================================================================================================================
void Flux_Speed_Estimator()
{
    //--------------------------------------------------------------------------------------------------------------------------
    //  Motor Back EMF Calculation
    //--------------------------------------------------------------------------------------------------------------------------

 //   VFD_Data.fop        = _IQmpy(_IQ(User_Data.Frated),VFD_Data.SpeedRmp);
 //   VFD_Data.FluxEstFc  = _IQmpy(VFD_Data.fop, MOT_FSE.K);
 //   MOT_FSE.FluxEstWc   = _IQmpy(_IQ(PIx2), VFD_Data.FluxEstFc);
 //   MOT_FSE.K5          = _IQmpy(MOT_FSE.K3 , VFD_Data.SpeedRmp);
 //   MOT_FSE.K4          = _IQ(1.0) - _IQmpy(VFD_Data.Ts, MOT_FSE.FluxEstWc); // //

    MOT_FSE.EsAlpha = MOT_FSE.VsAlpha - _IQmpy(MOT_Data.Rspu, MOT_FSE.IsAlpha);     // EsAlpha = VsAlpha - Rspu*IsAlpha
    MOT_FSE.EsBeta  = MOT_FSE.VsBeta  - _IQmpy(MOT_Data.Rspu, MOT_FSE.IsBeta);      // EsBeta = VsBeta - Rspu*IsBeta

    MOT_FSE.Psis_constant =  _IQdiv(MOT_FSE.K, VFD_Data.SpeedRmp );
    if (VFD_Data.Mot_Type == 1) // INDM
    {
    //--------------------------------------------------------------------------------------------------------------------------
    // Transformation from Alpha Beta to D-Q
    //--------------------------------------------------------------------------------------------------------------------------
        MOT_FSE.Isd = _IQmpy(MOT_FSE.IsAlpha, MOT_FSE.CosTheta);
        MOT_FSE.Isd = MOT_FSE.Isd + _IQmpy(MOT_FSE.IsBeta, MOT_FSE.SineTheta);
        MOT_FSE.Isq = _IQmpy(MOT_FSE.IsBeta, MOT_FSE.CosTheta);
        MOT_FSE.Isq = MOT_FSE.Isq - _IQmpy(MOT_FSE.IsAlpha, MOT_FSE.SineTheta);
    //--------------------------------------------------------------------------------------------------------------------------
    // Stator Flux Calculation in Rotating frame
    //--------------------------------------------------------------------------------------------------------------------------
        MOT_FSE.PsisAlpha    = _IQmpy(MOT_FSE.EsAlpha, MOT_FSE.K3);            // PsisAlpha[k] = K3*EsAlpha + K4*PsisAlpha[k-1]
        MOT_FSE.PsisAlpha    = MOT_FSE.PsisAlpha + _IQmpy(MOT_FSE.PsisAlphaOld, MOT_FSE.K4);
        MOT_FSE.PsisAlphaOld = MOT_FSE.PsisAlpha;

        MOT_FSE.PsisBeta     = _IQmpy(MOT_FSE.EsBeta, MOT_FSE.K3);             // PsisBeta[k]  = K3*EsBeta + K4*PsisBeta[k-1]
        MOT_FSE.PsisBeta     = MOT_FSE.PsisBeta + _IQmpy(MOT_FSE.PsisBetaOld, MOT_FSE.K4);
        MOT_FSE.PsisBetaOld  = MOT_FSE.PsisBeta;
                                                    // NOTE: The a*b + c*d intentionally written as 2 statements due to improper calculation observed....
                                                    // NOTE: a*b*c intentionally written as 2 statements due to improper calculation observed....
    //--------------------------------------------------------------------------------------------------------------------------


       MOT_FSE.PsisAlpha_comp = MOT_FSE.PsisAlpha +  _IQmpy(MOT_FSE.Psis_constant,MOT_FSE.PsisBeta) ;
       MOT_FSE.PsisBeta_comp =  MOT_FSE.PsisBeta   -  _IQmpy(MOT_FSE.Psis_constant,MOT_FSE.PsisAlpha) ;

    //    MOT_FSE.PsisAlpha_comp =  _IQmpy(MOT_FSE.K, MOT_FSE.PsisAlpha );
    //    MOT_FSE.PsisBeta_comp  =   MOT_FSE.PsisBeta - _IQdiv( MOT_FSE.PsisAlpha_comp , MOT_FSE.PsisAlpha );


     //   MOT_FSE.PsisAlpha_comp =  MOT_FSE.PsisAlpha + _IQmpy(MOT_FSE.K, MOT_FSE.PsisBeta );
     //   MOT_FSE.PsisBeta_comp  =  MOT_FSE.PsisBeta  - _IQmpy(MOT_FSE.K, MOT_FSE.PsisAlpha );

        // Rotor Flux Calculation in Rotating frame
    //--------------------------------------------------------------------------------------------------------------------------
        MOT_FSE.PsirAlpha    = _IQmpy(_IQ(MOT_Data.Sigma), MOT_Data.Lspu);     // PsirAlpha = Lr/Lm*[PsisAlpha - Sigma*Lspu*IsAlpha]
        MOT_FSE.PsirAlpha    = _IQmpy(MOT_FSE.PsirAlpha, MOT_FSE.IsAlpha);     // Sigma*Lspu*IsAlpha

        MOT_FSE.PsirBeta     = _IQmpy(_IQ(MOT_Data.Sigma), MOT_Data.Lspu);      // PsirBeta = Lr/Lm*[PsisBeta - Sigma*Lspu*IsBeta]
        MOT_FSE.PsirBeta     = _IQmpy(MOT_FSE.PsirBeta, MOT_FSE.IsBeta);        // Sigma*Lspu*IsBeta



        MOT_FSE.PsirAlpha    = MOT_FSE.PsisAlpha_comp - MOT_FSE.PsirAlpha;          // [PsisAlpha - Sigma*Lspu*IsAlpha]
    //    MOT_FSE.PsirAlphatemp    = _IQ21mpy(MOT_FSE.PsirAlphatemp, _IQ21(MOT_Data.Lr)); // As (Lr/Lm) Ratio comes in both Beta and Alpha components these steps are hided
    //    MOT_FSE.PsirAlphatemp    = _IQ21mpy(MOT_FSE.PsirAlphatemp, _IQ21(MOT_Data.Lm_1));


        MOT_FSE.PsirBeta     = MOT_FSE.PsisBeta_comp - MOT_FSE.PsirBeta;             // [PsisBeta - Sigma*Lspu*IsBeta]
     //   MOT_FSE.PsirBetatemp     = _IQ21mpy(MOT_FSE.PsirBetatemp, _IQ21(MOT_Data.Lr));
     //   MOT_FSE.PsirBetatemp     = _IQ21mpy(MOT_FSE.PsirBetatemp, _IQ21(MOT_Data.Lm_1));
    //--------------------------------------------------------------------------------------------------------------------------
    // Rotor Flux angle estimation
    //--------------------------------------------------------------------------------------------------------------------------

        MOT_FSE.ThetaFlux    =  _IQatan2PU(MOT_FSE.PsirBeta,MOT_FSE.PsirAlpha); //  //
        MOT_FSE.CosTheta     = _IQcosPU(MOT_FSE.ThetaFlux);
        MOT_FSE.SineTheta    = _IQsinPU(MOT_FSE.ThetaFlux);
    //--------------------------------------------------------------------------------------------------------------------------
    //  Synchronous Speed Calculation
    //--------------------------------------------------------------------------------------------------------------------------
        MOT_FSE.WSyn         = _IQmpy(MOT_FSE.CosTheta, (MOT_FSE.SineTheta - MOT_FSE.SineThetaOld));
        MOT_FSE.WSyntemp     = _IQmpy(MOT_FSE.SineTheta, (MOT_FSE.CosTheta - MOT_FSE.CosThetaOld));
        MOT_FSE.WSyn         = MOT_FSE.WSyn - MOT_FSE.WSyntemp;
        MOT_FSE.WSyn         = _IQmpy(MOT_FSE.WSyn,_IQ(MOT_Data.Wb_1));
        MOT_FSE.WSyn         = _IQmpyIQX(MOT_FSE.WSyn, GLOBAL_Q, VFD_Data.Ts_1, 15);
                                                                                // WSyn = CosTheta*[SinTheta - SineThetaOld] - SineTheta*[CosTheta - CosThetaOld]
        MOT_FSE.CosThetaOld  = MOT_FSE.CosTheta;
        MOT_FSE.SineThetaOld = MOT_FSE.SineTheta;
    //--------------------------------------------------------------------------------------------------------------------------
    // Slip Calculation                                                         // WSlip = Isq/(Wb*Trt*Imr)
    //--------------------------------------------------------------------------------------------------------------------------
        MOT_FSE.WSlip        = _IQmpyIQX(_IQ21(MOT_Data.Wb), 21, MOT_Data.Trt, GLOBAL_Q);
        MOT_FSE.WSlip        = _IQmpy(MOT_FSE.WSlip, MOT_FSE.Isd);              // Imr = Isd
        MOT_FSE.WSlip        = _IQdiv(MOT_FSE.Isq, MOT_FSE.WSlip);
    //--------------------------------------------------------------------------------------------------------------------------
    // Rotor Speed Estimation
    //--------------------------------------------------------------------------------------------------------------------------
        MOT_FSE.WEst         = MOT_FSE.WSyn - MOT_FSE.WSlip;                    // West = WSyn - WSlip

    }

    if (VFD_Data.Mot_Type == 2) // PMSM
    {
    //--------------------------------------------------------------------------------------------------------------------------
    // Stator Flux Calculation in Rotating frame
    //--------------------------------------------------------------------------------------------------------------------------
        MOT_FSE.PsisAlpha      = _IQmpy(MOT_FSE.EsAlpha,MOT_FSE.K3);            // PsisAlpha[k] = K3*EsAlpha + K4*PsisAlpha[k-1]
        MOT_FSE.PsisAlpha      = MOT_FSE.PsisAlpha + _IQmpy(MOT_FSE.PsisAlphaOld, MOT_FSE.K4);
        MOT_FSE.PsisAlphaOld   = MOT_FSE.PsisAlpha;

        MOT_FSE.PsisBeta       = _IQmpy(MOT_FSE.EsBeta,MOT_FSE.K3);             // PsisBeta[k]  = K3*EsBeta + K4*PsisBeta[k-1]
        MOT_FSE.PsisBeta       = MOT_FSE.PsisBeta + _IQmpy(MOT_FSE.PsisBetaOld, MOT_FSE.K4);
        MOT_FSE.PsisBetaOld    = MOT_FSE.PsisBeta;
    //--------------------------------------------------------------------------------------------------------------------------
    // Rotor Flux Calculation in Rotating frame
    //--------------------------------------------------------------------------------------------------------------------------
        MOT_FSE.PsirAlpha      = _IQmpy(MOT_FSE.IsAlpha, MOT_Data.Ldpu);
        MOT_FSE.PsirAlpha      = MOT_FSE.PsisAlpha-MOT_FSE.PsirAlpha;           // MOT_FSE.PsisAlpha - MOT_FSE.PsirAlpha;    // PsirAlpha = PsisAlpha - Ib*Ldpu*IsAlpha

        MOT_FSE.PsirBeta       = _IQmpy(MOT_FSE.IsBeta, MOT_Data.Ldpu);
        MOT_FSE.PsirBeta       = MOT_FSE.PsisBeta- MOT_FSE.PsirBeta;            // MOT_FSE.PsisBeta - MOT_FSE.PsirBeta;      // PsirBeta = PsisBeta - Ib*Ldpu*IsBeta
    //--------------------------------------------------------------------------------------------------------------------------
    // Rotor Flux angle estimation
    //--------------------------------------------------------------------------------------------------------------------------
        MOT_FSE.ThetaFlux      = _IQatan2PU(MOT_FSE.PsirBeta, MOT_FSE.PsirAlpha);   // ThetaFlux = atan(PsirBeta/PsirAlpha)
        MOT_FSE.CosTheta       = _IQcosPU(MOT_FSE.ThetaFlux);
        MOT_FSE.SineTheta      = _IQsinPU(MOT_FSE.ThetaFlux);
    //--------------------------------------------------------------------------------------------------------------------------
    // Synchronous Speed Calculation
    //--------------------------------------------------------------------------------------------------------------------------
        MOT_FSE.WSyn           = MOT_FSE.SineTheta - MOT_FSE.SineThetaOld;      // WSyn = CosTheta*[SinTheta - SineThetaOld] - SineTheta*[CosTheta - CosThetaOld]
        MOT_FSE.WSyn           = _IQmpy(MOT_FSE.WSyn, MOT_FSE.CosTheta);
        MOT_FSE.WSyntemp       = MOT_FSE.CosTheta - MOT_FSE.CosThetaOld;
        MOT_FSE.WSyntemp       = _IQmpy(MOT_FSE.WSyntemp, MOT_FSE.SineTheta);
        MOT_FSE.WSyn           = MOT_FSE.WSyn - MOT_FSE.WSyntemp;
        MOT_FSE.WSyn           = _IQmpy(MOT_FSE.WSyn,_IQ(MOT_Data.Wb_1));
        MOT_FSE.WSyn           = _IQmpyIQX(MOT_FSE.WSyn, GLOBAL_Q, VFD_Data.Ts_1, 15);

        MOT_FSE.CosThetaOld    = MOT_FSE.CosTheta;
        MOT_FSE.SineThetaOld   = MOT_FSE.SineTheta;
    //--------------------------------------------------------------------------------------------------------------------------
    // Rotor Speed Estimation
    //--------------------------------------------------------------------------------------------------------------------------
        MOT_FSE.WEst           = MOT_FSE.WSyn;                                  // West = WSyn //PMSM

    }

    //--------------------------------------------------------------------------------------------------------------------------
    // Low Pass Filter for Estimated Speed
    //--------------------------------------------------------------------------------------------------------------------------
        MOT_FSE.WEstFilt       = MOT_FSE.WEstFiltOld + _IQmpy(MOT_FSE.SpeedEstTc, (MOT_FSE.WEst- MOT_FSE.WEstFilt));
        MOT_FSE.WEstFiltOld    = MOT_FSE.WEstFilt;                              // WestFilt = WestFiltOld + SpeedEstTc*(West - WestFilt)

 }
//=========================================================================================================================
// Speed PI Controller
//=========================================================================================================================
void PI_Speed()
{
   // Trapezoidal PI Controller

    // proportional term            // Error term: up = Ref - Fbk
    PI_Spd.up = PI_Spd.Ref - PI_Spd.Fbk;

    // integral term                // if (PI_Spd.Out == PI_Spd.v1), then ui = Ki*(up+u1) + i1, else ui = i1
    PI_Spd.ut = PI_Spd.up + PI_Spd.u1;
    PI_Spd.ui = (PI_Spd.Out == PI_Spd.v1) ? (_IQmpy(PI_Spd.Ki, PI_Spd.ut)+ PI_Spd.i1) : PI_Spd.i1;

    // control output               // v1 = (Kp*up) + ui, saturate b/n Umin to Umax
    PI_Spd.v1 = _IQmpy(PI_Spd.Kp, PI_Spd.up) + PI_Spd.ui;
    PI_Spd.Out= _IQsat(PI_Spd.v1, PI_Spd.Umax, PI_Spd.Umin);

    // memory storage
    PI_Spd.u1 = PI_Spd.up;          // up[k-1]
    PI_Spd.i1 = PI_Spd.ui;          // ui[k-1]


/*  // Backward Euler PI Controller

    // proportional term
    PI_Spd.up = PI_Spd.Ref - PI_Spd.Fbk;

    // integral term
    PI_Spd.ui = (PI_Spd.Out == PI_Spd.v1) ? (_IQmpy(PI_Spd.Ki, PI_Spd.up)+ PI_Spd.i1) : PI_Spd.i1;
    PI_Spd.i1 = PI_Spd.ui;

    // control output
    PI_Spd.v1 = _IQmpy(PI_Spd.Kp, PI_Spd.up) + PI_Spd.ui;
    PI_Spd.Out= _IQsat(PI_Spd.v1, PI_Spd.Umax, PI_Spd.Umin);
*/

}
//=========================================================================================================================
// IsD Current PI Controller
//=========================================================================================================================
void PI_Dcurrent()
{
   // Trapezoidal PI Controller

    // proportional term            // Error term: up = Ref - Fbk
    PI_Isd.up = PI_Isd.Ref - PI_Isd.Fbk;

    // integral term                // if (PI_Isd.Out == PI_Isd.v1), then ui = Ki*(up+u1) + i1, else ui = i1
    PI_Isd.ut = PI_Isd.up + PI_Isd.u1;
    PI_Isd.ui = (PI_Isd.Out == PI_Isd.v1) ? (_IQmpy(PI_Isd.Ki, PI_Isd.ut)+ PI_Isd.i1) : PI_Isd.i1;

    // control output               // v1 = (Kp*up) + ui, saturate b/n Umin to Umax
    PI_Isd.v1 = _IQmpy(PI_Isd.Kp, PI_Isd.up) + PI_Isd.ui;
    PI_Isd.Out= _IQsat(PI_Isd.v1, PI_Isd.Umax, PI_Isd.Umin);

    // memory storage
    PI_Isd.u1 = PI_Isd.up;          // up[k-1]
    PI_Isd.i1 = PI_Isd.ui;          // ui[k-1]


/*  // Backward Euler PI Controller

    // proportional term
    PI_Isd.up = PI_Isd.Ref - PI_Isd.Fbk;

    // integral term
    PI_Isd.ui = (PI_Isd.Out == PI_Isd.v1) ? (_IQmpy(PI_Isd.Ki, PI_Isd.up)+ PI_Isd.i1) : PI_Isd.i1;
    PI_Isd.i1 = PI_Isd.ui;

    // control output
    PI_Isd.v1 = _IQmpy(PI_Isd.Kp, PI_Isd.up) + PI_Isd.ui;
    PI_Isd.Out= _IQsat(PI_Isd.v1, PI_Isd.Umax, PI_Isd.Umin);
*/

}
//=========================================================================================================================
// IsQ Current PI Controller
//=========================================================================================================================
void PI_Qcurrent()
{
    // Trapezoidal PI Controller

    // proportional term            // Error term: up = Ref - Fbk
    PI_Isq.up = PI_Isq.Ref - PI_Isq.Fbk;

    // integral term                // if (PI_Isq.Out == PI_Isq.v1), then ui = Ki*(up+u1) + i1, else ui = i1
    PI_Isq.ut = PI_Isq.up + PI_Isq.u1;
    PI_Isq.ui = (PI_Isq.Out == PI_Isq.v1) ? (_IQmpy(PI_Isq.Ki, PI_Isq.ut)+ PI_Isq.i1) : PI_Isq.i1;

    // control output               // v1 = (Kp*up) + ui, saturate b/n Umin to Umax
    PI_Isq.v1 = _IQmpy(PI_Isq.Kp, PI_Isq.up) + PI_Isq.ui;
    PI_Isq.Out= _IQsat(PI_Isq.v1, PI_Isq.Umax, PI_Isq.Umin);

    // memory storage
    PI_Isq.u1 = PI_Isq.up;          // up[k-1]
    PI_Isq.i1 = PI_Isq.ui;          // ui[k-1]


/*  // Backward Euler PI Controller

    // proportional term
    PI_Isq.up = PI_Isq.Ref - PI_Isq.Fbk;

    // integral term
    PI_Isq.ui = (PI_Isq.Out == PI_Isq.v1) ? (_IQmpy(PI_Isq.Ki, PI_Isq.up)+ PI_Isq.i1) : PI_Isq.i1;
    PI_Isq.i1 = PI_Isq.ui;

    // control output
    PI_Isq.v1 = _IQmpy(PI_Isq.Kp, PI_Isq.up) + PI_Isq.ui;
    PI_Isq.Out= _IQsat(PI_Isq.v1, PI_Isq.Umax, PI_Isq.Umin);
*/

}
//=========================================================================================================================
// VsD Decoupling Control, to generate the Vsd Ref (Vsd*)
//=========================================================================================================================
void Vsd_Decoupling_Control()
{
    if(VFD_Data.Mot_Type == 1)  // INDM
    {
        // Vsd Decoupling Control Equation => Vsd* = Rspu*Isd + PI_Isd.Out - 1/GI(pu)*Sigma*Lspu*Wmrpu*Isq
        VFD_Data.VsdRef = _IQmpy(_IQ(MOT_Data.Sigma), MOT_Data.Lspu);
        VFD_Data.VsdRef = _IQmpy(VFD_Data.VsdRef, VFD_Data.SpeedAct);   // Check the 1/GIpu part
        VFD_Data.VsdRef = _IQmpy(VFD_Data.VsdRef, VFD_Data.Isq);        // VsdRef = 1/GI(pu)*Sigma*Lspu*Wmrpu*Isq

        VFD_Data.VsdRef = _IQmpy(MOT_Data.Rspu, VFD_Data.Isd) + PI_Isd.Out - VFD_Data.VsdRef;
                                                                        // VsdRef CALCULATED RESULT
    }

    if(VFD_Data.Mot_Type == 2)  // PMSM
    {
        // Vsd Decoupling Control Equation => Vsd* = Rspu*Isd + PI_Isd.Out - 1/GI(pu)*Ldpu*Wmrpu*Isq
        VFD_Data.VsdRef = _IQmpy(MOT_Data.Ldpu, VFD_Data.SpeedAct);     // Check the 1/GIpu part
        VFD_Data.VsdRef = _IQmpy(VFD_Data.VsdRef, VFD_Data.Isq);        // VsdRef = 1/GI(pu)*Sigma*Lspu*Wmrpu*Isq
        VFD_Data.VsdRef = _IQmpy(MOT_Data.Rspu, VFD_Data.Isd) + PI_Isd.Out - VFD_Data.VsdRef;
                                                                        // VsdRef CALCULATED RESULT
    }

    VFD_Data.VsdRef = _IQsat(VFD_Data.VsdRef, VFD_Data.VsdLim, -VFD_Data.VsdLim); // VsdRef Saturation
}
//=========================================================================================================================
// VsQ Decoupling Control, to generate the Vsq Ref (Vsq*)
//=========================================================================================================================
void Vsq_Decoupling_Control()
{
    if(VFD_Data.Mot_Type == 1)  // INDM
    {
        // Vsd Decoupling Control Equation => Vsq* = Rspu*Isq + PI_Isq.Out + 1/GI(pu)*Lspu*Wmrpu*[Sigma*Isd - (1-Sigma)*Imr]
        VFD_Data.VsqRef = _IQmpy(_IQ(MOT_Data.Sigma), VFD_Data.Isd);     // Sigma*Isd
        VFD_Data.VsqRef = VFD_Data.VsqRef - _IQmpy((1-MOT_Data.Sigma), VFD_Data.Imr);
                                                                        // [Sigma*Isd - (1-Sigma)*Imr]
        VFD_Data.VsqRef = _IQmpy(VFD_Data.VsqRef, VFD_Data.SpeedAct);
        VFD_Data.VsqRef = _IQmpy(VFD_Data.VsqRef, MOT_Data.Lspu);        // VsqRef = 1/GI(pu)*Lspu*Wmrpu*[Sigma*Isd + (1-Sigma)*Imr]
        VFD_Data.VsqRef = _IQmpy(MOT_Data.Rspu, VFD_Data.Isq) + PI_Isq.Out + VFD_Data.VsqRef;
                                                                        // VsqRef CALCULATED RESULT
    }

    if(VFD_Data.Mot_Type == 2)  // PMSM
    {
        // Vsd Decoupling Control Equation => Vsq* = Rspu*Isq + PI_Isq.Out + 1/GI(pu)*Ldpu*Wmrpu*Isd + Wmrpu*Lmdf
        VFD_Data.VsqRef = _IQmpy(MOT_Data.Ldpu, VFD_Data.Isd);          // Ldpu*Isd
        VFD_Data.VsqRef = _IQmpy(VFD_Data.VsqRef, VFD_Data.SpeedAct);   // Ldpu*Wmrpu*Isd
        VFD_Data.VsqRef = VFD_Data.VsqRef + _IQmpy(VFD_Data.SpeedAct, MOT_Data.Lmdfpu);
                                                                        // [Ldpu*Wmrpu*Isd + Wmrpu*Lmdf]
        VFD_Data.VsqRef = _IQmpy(MOT_Data.Rspu, VFD_Data.Isq) + PI_Isq.Out + VFD_Data.VsqRef;
                                                                        // VsqRef CALCULATED RESULT
    }

    VFD_Data.VsqRef = _IQsat(VFD_Data.VsqRef, VFD_Data.VsqLim, - VFD_Data.VsqLim); // VsqRef Saturation
}
//=========================================================================================================================
// A,B,C to ALPHA-BETA generation
//=========================================================================================================================
void Transform_3to2(_iq Ia,_iq Ib,_iq Ic)
{
    // Modified Clark's Transformation
    VFD_Data.IsAlpha    = Ia - _IQmpy(_IQ(0.5), Ib) - _IQmpy(_IQ(0.5), Ic); // Ia - _IQmpy(_IQ(0.5), Ib) - _IQmpy(_IQ(0.5), Ic);
    VFD_Data.IsAlpha    = _IQmpy(VFD_Data.IsAlpha,_IQ(0.666666667));        // _IQmpy(VFD_Data.IsAlpha,_IQ(0.666666667));
                                                                            // IsAlpha = 2/3*[Ia - 0.5*Ib - 0.5*Ic]
    VFD_Data.IsBeta     =   _IQmpy((Ib - Ic),_IQ(Sqrt3_1));                 //_IQmpy((Ib - Ic),_IQ(Sqrt3_1));
                                                                            // IsBeta =2/3*[(sqrt(3)/2)*[Ib - Ic]]
}
//=========================================================================================================================
// ALPHA-BETA to D-Q generation
//=========================================================================================================================
void Transform_Forward(_iq Alpha,_iq Beta,_iq cosAngle,_iq sinAngle)
{
    VFD_Data.Isd = _IQmpy(Alpha,cosAngle) + _IQmpy(Beta,sinAngle);
    VFD_Data.Isq = _IQmpy(Beta,cosAngle) - _IQmpy(Alpha,sinAngle);
}
//=========================================================================================================================
// D-Q to ALPHA-BETA generation
//=========================================================================================================================
void Transform_Backward (_iq Ds,_iq Qs,_iq cosAngle,_iq sinAngle)
{
    VFD_Data.VsAlphaRef = _IQmpy(Ds, cosAngle) - _IQmpy(Qs, sinAngle);
    VFD_Data.VsBetaRef  = _IQmpy(Qs, cosAngle) + _IQmpy(Ds, sinAngle);
}
//=========================================================================================================================
// ALPHA-BETA to A,B,C generation
//=========================================================================================================================
void Transform_2to3(_iq Alpha, _iq Beta)
{
    // Inverse Clark's Transformation
    VFD_Data.VrRef      = Alpha;                    // Vr = Valpha
    VFD_Data.VyRef      = _IQmpy(Beta, _IQ(Sqrt3by2)) - _IQmpy(Alpha, _IQ(0.5));
                                                    // Vy = sqrt(3)/2*Vbeta - 1/2*Valpha
    VFD_Data.VbRef      = -(_IQmpy(Beta, _IQ(Sqrt3by2)) + _IQmpy(Alpha, _IQ(0.5)));
                                                    // Vb = -[sqrt(3)/2*Vbeta + 1/2*Valpha]
}
//=========================================================================================================================
// Third Harmonic PWM Generator for Field Oriented Control
//=========================================================================================================================
void THIPWM_Gen()
{
    Transform_2to3(MOT_SVM.Alpha, MOT_SVM.Beta);

    MOT_SVM.VaRef   = -VFD_Data.VrRef;  // Negative due to inversion in Gate Driver PCB _IQ(0.0);
    MOT_SVM.VbRef   = -VFD_Data.VyRef;
    MOT_SVM.VcRef   = -VFD_Data.VbRef;

    if (MOT_SVM.VaRef > MOT_SVM.VbRef)  // Find Maximum and Minimum Phase
    {
        MOT_SVM.tmp1 = MOT_SVM.VaRef;   // If yes, R Phase ref is the maximum
        MOT_SVM.tmp2 = MOT_SVM.VbRef;   // If yes, Y Phase ref is the minimum
    }
    else
    {
        MOT_SVM.tmp1 = MOT_SVM.VbRef;   // If yes, Y Phase ref is the maximum
        MOT_SVM.tmp2 = MOT_SVM.VaRef;   // If yes, R Phase ref is the minimum
    }
    if (MOT_SVM.VcRef > MOT_SVM.tmp1) MOT_SVM.tmp1 = MOT_SVM.VcRef; // If yes, B phase ref is maximum
    if (MOT_SVM.VcRef < MOT_SVM.tmp2) MOT_SVM.tmp2 = MOT_SVM.VcRef; // If yes, B phase ref is minimum

    MOT_SVM.Ta  = MOT_SVM.VaRef - _IQmpy((MOT_SVM.tmp1+MOT_SVM.tmp2), _IQ(0.5));    // Reference + Half (min + max)
    MOT_SVM.Tb  = MOT_SVM.VbRef - _IQmpy((MOT_SVM.tmp1+MOT_SVM.tmp2), _IQ(0.5));    // Reference + Half (min + max)
    MOT_SVM.Tc  = MOT_SVM.VcRef - _IQmpy((MOT_SVM.tmp1+MOT_SVM.tmp2), _IQ(0.5));    // Reference + Half (min + max)
}
//=========================================================================================================================
// EPWM Generator to generate the PWM from the DSP
//=========================================================================================================================
void EPWM_Generator()
{
    MOT_SVM.Ta  = _IQsat(MOT_SVM.Ta, _IQ(1.0), _IQ(-1.0));  // Saturating to (1.0) for overmodulation
    MOT_SVM.Tb  = _IQsat(MOT_SVM.Tb, _IQ(1.0), _IQ(-1.0));
    MOT_SVM.Tc  = _IQsat(MOT_SVM.Tc, _IQ(1.0), _IQ(-1.0));

    if (VFD_Data.Mot_Rot == 0)    // AntiClockwise Motor Rotation (default)
    {
        EPwm1Regs.CMPA.half.CMPA    = _IQmpy(MOT_SVM.HalfPerMax,MOT_SVM.Ta) + MOT_SVM.HalfPerMax;
        EPwm2Regs.CMPA.half.CMPA    = _IQmpy(MOT_SVM.HalfPerMax,MOT_SVM.Tb) + MOT_SVM.HalfPerMax;
        EPwm3Regs.CMPA.half.CMPA    = _IQmpy(MOT_SVM.HalfPerMax,MOT_SVM.Tc) + MOT_SVM.HalfPerMax;
    }

    if (VFD_Data.Mot_Rot == 1)    // Clockwise Motor Rotation
    {
        EPwm1Regs.CMPA.half.CMPA    = _IQmpy(MOT_SVM.HalfPerMax,MOT_SVM.Tb) + MOT_SVM.HalfPerMax;
        EPwm2Regs.CMPA.half.CMPA    = _IQmpy(MOT_SVM.HalfPerMax,MOT_SVM.Ta) + MOT_SVM.HalfPerMax;
        EPwm3Regs.CMPA.half.CMPA    = _IQmpy(MOT_SVM.HalfPerMax,MOT_SVM.Tc) + MOT_SVM.HalfPerMax;
    }
}

//=========================================================================================================================
// RELEASE THE PWM PULSES FOR THE INVERTER
//=========================================================================================================================
void INVERTER_ON()
{
    EALLOW;

// PWM 1 Control Functions
//-------------------------------------------------------------------------------------------------------------------
// Action Control Register [AQCTLA]
//-------------------------------------------------------------------------------------------------------------------
    EPwm1Regs.AQCTLA.bit.CAD      = AQ_CLEAR;               // Action when CTR = CMPA on Down Count,    [Clear]
    EPwm1Regs.AQCTLA.bit.CAU      = AQ_SET;                 // Action when CTR = CMPA on Up Count,      [Set]
//-------------------------------------------------------------------------------------------------------------------
// Action Control Register [AQCTLB]
//-------------------------------------------------------------------------------------------------------------------
//    EPwm1Regs.AQCTLB.bit.CAD      = AQ_SET;               // Action when CTR = CMPB on Down Count,    [Clear]
//    EPwm1Regs.AQCTLB.bit.CAU      = AQ_CLEAR;                 // Action when CTR = CMPB on Up Count,      [Set]

    EPwm1Regs.AQCTLB.bit.CBD      = AQ_CLEAR;               // Action when CTR = CMPB on Down Count,    [Clear]
    EPwm1Regs.AQCTLB.bit.CBU      = AQ_SET;                 // Action when CTR = CMPB on Up Count,      [Set]
//-------------------------------------------------------------------------------------------------------------------
//  Action-Qualifier Continuous Software Force Register [AQCSFRC]
//-------------------------------------------------------------------------------------------------------------------
    EPwm1Regs.AQCSFRC.bit.CSFB    = AQ_FRC_DISABLED;        // Continuous S/W Force on Output B     [Disabled]
    EPwm1Regs.AQCSFRC.bit.CSFA    = AQ_FRC_DISABLED;        // Continuous S/W Force on Output A     [Disabled]
//-------------------------------------------------------------------------------------------------------------------
//  Dead-Band Generator Control Register[DBCTL]
//-------------------------------------------------------------------------------------------------------------------

    EPwm1Regs.DBCTL.bit.POLSEL    = DB_ACTV_HIC ;
//    EPwm1Regs.DBCTL.bit.POLSEL    = DB_ACTV_LOC;            // Polarity Select Control, [Active Low Complementary]
    EPwm1Regs.DBCTL.bit.OUT_MODE  = DB_FULL_ENABLE;         // Out Mode Control,        [FED and RED]

// PWM 2 Control Functions
//-------------------------------------------------------------------------------------------------------------------
// Action Control Register [AQCTLA]
//-------------------------------------------------------------------------------------------------------------------
    EPwm2Regs.AQCTLA.bit.CAD      = AQ_CLEAR;               // Action when CTR = CMPA on Down Count,    [Clear]
    EPwm2Regs.AQCTLA.bit.CAU      = AQ_SET;                 // Action when CTR = CMPA on Up Count,      [Set]
//-------------------------------------------------------------------------------------------------------------------
// Action Control Register [AQCTLB]
//-------------------------------------------------------------------------------------------------------------------
    //    EPwm2Regs.AQCTLB.bit.CAD      = AQ_SET;               // Action when CTR = CMPB on Down Count,    [Clear]
    //    EPwm2Regs.AQCTLB.bit.CAU      = AQ_CLEAR;                 // Action when CTR = CMPB on Up Count,      [Set]


    EPwm2Regs.AQCTLB.bit.CBD      = AQ_CLEAR;               // Action when CTR = CMPB on Down Count,    [Clear]
    EPwm2Regs.AQCTLB.bit.CBU      = AQ_SET;                 // Action when CTR = CMPB on Up Count,      [Set]
//-------------------------------------------------------------------------------------------------------------------
//  Action-Qualifier Continuous Software Force Register [AQCSFRC]
//-------------------------------------------------------------------------------------------------------------------
    EPwm2Regs.AQCSFRC.bit.CSFB    = AQ_FRC_DISABLED;        // Continuous S/W Force on Output B     [Disabled]
    EPwm2Regs.AQCSFRC.bit.CSFA    = AQ_FRC_DISABLED;        // Continuous S/W Force on Output A     [Disabled]
//-------------------------------------------------------------------------------------------------------------------
//  Dead-Band Generator Control Register[DBCTL]
//-------------------------------------------------------------------------------------------------------------------
    EPwm2Regs.DBCTL.bit.POLSEL    = DB_ACTV_HIC ;
 //   EPwm2Regs.DBCTL.bit.POLSEL    = DB_ACTV_LOC;            // Polarity Select Control, [Active Low Complementary]
    EPwm2Regs.DBCTL.bit.OUT_MODE  = DB_FULL_ENABLE;         // Out Mode Control,        [FED and RED]

// PWM 3 Control Functions
//-------------------------------------------------------------------------------------------------------------------
// Action Control Register [AQCTLA]
//-------------------------------------------------------------------------------------------------------------------
    EPwm3Regs.AQCTLA.bit.CAD      = AQ_CLEAR;               // Action when CTR = CMPA on Down Count,    [Clear]
    EPwm3Regs.AQCTLA.bit.CAU      = AQ_SET;                 // Action when CTR = CMPA on Up Count,      [Set]
//-------------------------------------------------------------------------------------------------------------------
// Action Control Register [AQCTLB]
//-------------------------------------------------------------------------------------------------------------------
    //    EPwm3Regs.AQCTLB.bit.CAD      = AQ_SET;               // Action when CTR = CMPB on Down Count,    [Clear]
    //    EPwm3Regs.AQCTLB.bit.CAU      = AQ_CLEAR;                 // Action when CTR = CMPB on Up Count,      [Set]


    EPwm3Regs.AQCTLB.bit.CBD      = AQ_CLEAR;               // Action when CTR = CMPB on Down Count,    [Clear]
    EPwm3Regs.AQCTLB.bit.CBU      = AQ_SET;                 // Action when CTR = CMPB on Up Count,      [Set]
//-------------------------------------------------------------------------------------------------------------------
//  Action-Qualifier Continuous Software Force Register [AQCSFRC]
//-------------------------------------------------------------------------------------------------------------------
    EPwm3Regs.AQCSFRC.bit.CSFB    = AQ_FRC_DISABLED;        // Continuous S/W Force on Output B     [Disabled]
    EPwm3Regs.AQCSFRC.bit.CSFA    = AQ_FRC_DISABLED;        // Continuous S/W Force on Output A     [Disabled]
//-------------------------------------------------------------------------------------------------------------------
//  Dead-Band Generator Control Register[DBCTL]
//-------------------------------------------------------------------------------------------------------------------
    EPwm3Regs.DBCTL.bit.POLSEL    = DB_ACTV_HIC ;
//    EPwm3Regs.DBCTL.bit.POLSEL    = DB_ACTV_LOC;            // Polarity Select Control, [Active Low Complementary]
    EPwm3Regs.DBCTL.bit.OUT_MODE  = DB_FULL_ENABLE;         // Out Mode Control,        [FED and RED]

}
//=========================================================================================================================
// BLOCK THE PWM PULSES FOR THE INVERTER
//=========================================================================================================================
void INVERTER_OFF()
{
    EALLOW;
// PWM 1 Control Functions
//-------------------------------------------------------------------------------------------------------------------
// Action Control Register [AQCTLA]
//-------------------------------------------------------------------------------------------------------------------
    EPwm1Regs.AQCTLA.bit.CAD      = AQ_NO_ACTION;           // Action when CTR = CMPA on Down Count,    [No Action]
    EPwm1Regs.AQCTLA.bit.CAU      = AQ_NO_ACTION;           // Action when CTR = CMPA on Up Count,      [No Action]
//-------------------------------------------------------------------------------------------------------------------
// Action Control Register [AQCTLB]
//-------------------------------------------------------------------------------------------------------------------
    EPwm1Regs.AQCTLB.bit.CBD      = AQ_NO_ACTION;           // Action when CTR = CMPB on Down Count,    [No Action]
    EPwm1Regs.AQCTLB.bit.CBU      = AQ_NO_ACTION;           // Action when CTR = CMPB on Up Count,      [No Action]
//-------------------------------------------------------------------------------------------------------------------
//  Action-Qualifier Continuous Software Force Register [AQCSFRC]
//-------------------------------------------------------------------------------------------------------------------
    EPwm1Regs.AQCSFRC.bit.CSFB    = AQ_FRC_CONT_LO;         // Continuous S/W Force on Output B     [Forcing High]
    EPwm1Regs.AQCSFRC.bit.CSFA    = AQ_FRC_CONT_LO; // No Inversion for New Redesigned Unit
//    EPwm1Regs.AQCSFRC.bit.CSFB    = AQ_FRC_CONT_HI;         // Continuous S/W Force on Output B     [Forcing High]
//    EPwm1Regs.AQCSFRC.bit.CSFA    = AQ_FRC_CONT_HI;         // Continuous S/W Force on Output A     [Forcing High]
//-------------------------------------------------------------------------------------------------------------------
//  Dead-Band Generator Control Register[DBCTL]
//-------------------------------------------------------------------------------------------------------------------
    EPwm1Regs.DBCTL.bit.POLSEL    = DB_ACTV_HI;             // Polarity Select Control, [Active Low Complementary]
    EPwm1Regs.DBCTL.bit.OUT_MODE  = DB_DISABLE;             // Out Mode Control,        [FED and RED]

// PWM 2 Control Functions
//-------------------------------------------------------------------------------------------------------------------
// Action Control Register [AQCTLA]
//-------------------------------------------------------------------------------------------------------------------
    EPwm2Regs.AQCTLA.bit.CAD      = AQ_NO_ACTION;           // Action when CTR = CMPA on Down Count,    [No Action]
    EPwm2Regs.AQCTLA.bit.CAU      = AQ_NO_ACTION;           // Action when CTR = CMPA on Up Count,      [No Action]
//-------------------------------------------------------------------------------------------------------------------
// Action Control Register [AQCTLB]
//-------------------------------------------------------------------------------------------------------------------
    EPwm2Regs.AQCTLB.bit.CBD      = AQ_NO_ACTION;           // Action when CTR = CMPB on Down Count,    [No Action]
    EPwm2Regs.AQCTLB.bit.CBU      = AQ_NO_ACTION;           // Action when CTR = CMPB on Up Count,      [No Action]
//-------------------------------------------------------------------------------------------------------------------
//  Action-Qualifier Continuous Software Force Register [AQCSFRC]
//-------------------------------------------------------------------------------------------------------------------
        EPwm2Regs.AQCSFRC.bit.CSFB    = AQ_FRC_CONT_LO;         // Continuous S/W Force on Output B     [Forcing High]
        EPwm2Regs.AQCSFRC.bit.CSFA  =   AQ_FRC_CONT_LO; //  No Inversion for New Redesigned Unit

 //   EPwm2Regs.AQCSFRC.bit.CSFB    = AQ_FRC_CONT_HI;         // Continuous S/W Force on Output B     [Forcing High]
 //   EPwm2Regs.AQCSFRC.bit.CSFA    = AQ_FRC_CONT_HI;         // Continuous S/W Force on Output A     [Forcing High]
//-------------------------------------------------------------------------------------------------------------------
//  Dead-Band Generator Control Register[DBCTL]
//-------------------------------------------------------------------------------------------------------------------
    EPwm2Regs.DBCTL.bit.POLSEL    = DB_ACTV_HI;             // Polarity Select Control, [Active Low Complementary]
    EPwm2Regs.DBCTL.bit.OUT_MODE  = DB_DISABLE;             // Out Mode Control,        [FED and RED]

// PWM 3 Control Functions
//-------------------------------------------------------------------------------------------------------------------
// Action Control Register [AQCTLA]
//-------------------------------------------------------------------------------------------------------------------
    EPwm3Regs.AQCTLA.bit.CAD      = AQ_NO_ACTION;           // Action when CTR = CMPA on Down Count,    [No Action]
    EPwm3Regs.AQCTLA.bit.CAU      = AQ_NO_ACTION;           // Action when CTR = CMPA on Up Count,      [No Action]
//-------------------------------------------------------------------------------------------------------------------
// Action Control Register [AQCTLB]
//-------------------------------------------------------------------------------------------------------------------
    EPwm3Regs.AQCTLB.bit.CBD      = AQ_NO_ACTION;           // Action when CTR = CMPB on Down Count,    [No Action]
    EPwm3Regs.AQCTLB.bit.CBU      = AQ_NO_ACTION;           // Action when CTR = CMPB on Up Count,      [No Action]
//-------------------------------------------------------------------------------------------------------------------
//  Action-Qualifier Continuous Software Force Register [AQCSFRC]
//-------------------------------------------------------------------------------------------------------------------
    EPwm3Regs.AQCSFRC.bit.CSFB    = AQ_FRC_CONT_LO;         // Continuous S/W Force on Output B     [Forcing High]
    EPwm3Regs.AQCSFRC.bit.CSFA    = AQ_FRC_CONT_LO; //  No Inversion for New Redesigned Unit

   // EPwm3Regs.AQCSFRC.bit.CSFB    = AQ_FRC_CONT_HI;         // Continuous S/W Force on Output B     [Forcing High]
   // EPwm3Regs.AQCSFRC.bit.CSFA    = AQ_FRC_CONT_HI;         // Continuous S/W Force on Output A     [Forcing High]
//-------------------------------------------------------------------------------------------------------------------
//  Dead-Band Generator Control Register[DBCTL]
//-------------------------------------------------------------------------------------------------------------------
    EPwm3Regs.DBCTL.bit.POLSEL    = DB_ACTV_HI;             // Polarity Select Control, [Active Low Complementary]
    EPwm3Regs.DBCTL.bit.OUT_MODE  = DB_DISABLE;             // Out Mode Control,        [FED and RED]

}


void PI_Imrcurrent()
{
    // Trapezoidal PI Controller

    // proportional term            // Error term: up = Ref - Fbk
    PI_Imr.up = PI_Imr.Ref - PI_Imr.Fbk;

    // integral term                // if (PI_Isq.Out == PI_Isq.v1), then ui = Ki*(up+u1) + i1, else ui = i1
    PI_Imr.ut = PI_Imr.up + PI_Imr.u1;
    PI_Imr.ui = (PI_Imr.Out == PI_Imr.v1) ? (_IQmpy(PI_Imr.Ki, PI_Imr.ut)+ PI_Imr.i1) : PI_Imr.i1;

    // control output               // v1 = (Kp*up) + ui, saturate b/n Umin to Umax
    PI_Imr.v1 = _IQmpy(PI_Imr.Kp, PI_Imr.up) + PI_Imr.ui;
    PI_Imr.Out= _IQsat(PI_Imr.v1, PI_Imr.Umax, PI_Imr.Umin);

    // memory storage
    PI_Imr.u1 = PI_Imr.up;          // up[k-1]
    PI_Imr.i1 = PI_Imr.ui;          // ui[k-1]
}

// This function is to implement Theta Change over automatically
