//==================================================================================================================================================
// FILE NAME        : Funcs_5_Faults&Warnings.c
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




//=========================================================================================================================
// DRIVE FAULT CONFIGURATION
//=========================================================================================================================
void System_Faults()
{
//-------------------------------------------------------------------------------------------------------------------
// VARIOUS SOFTWARE GENERATED FAULTS Fault Flag 1
//-------------------------------------------------------------------------------------------------------------------
// AMPLIFIER ON LINE STATUS SCREEN - FAULT INDICATIONS
// 01.POWER BASE FAULT                 = 0001h  Vce[sat]FROM GATE DRIVER
// 02.ABC CURRENT FAULT                = 0002h
// 03.Internal R Phase Missing         = 0004h
// 04.Internal Y Phase Missing         = 0008h
// 05.Internal B Phase Missing         = 0010h
// 06.Single Phasing R Phase Missing   = 0020h
// 07.Single Phasing Y Phase Missing   = 0040h
// 08.Single Phasing B Phase Missing   = 0080h
// 09.Zero Crossing R Phase Mismatch   = 0100h
// 10.Zero Crossing Y Phase Mismatch   = 0200h
// 11.Zero Crossing B Phase Mismatch   = 0400h
// 12.Current Sensing Mismatch         = 0800h
// 13.Grid Sensing low voltage         = 1000h
// 14.Grid Sensing Unbalance_Voltage   = 2000h
// 15.IGBT Over Temperature            = 4000h
// 16. DC Link Undervoltage            = 8000h
// 17. DC Link Overvoltage             = 10000h
// 18. Heat Sink Fan not working       = 20000h
// 19. SMPS Fan not working            = 40000h
// 19. Enclosure Over Temeperature     = 80000h
// 20. Speed Error                     = 100000h

//-------------------------------------------------------------------------------------------------------------------
// 01 IGBT Fault from Gate Driver
//-------------------------------------------------------------------------------------------------------------------

    if (PULSE_BLK_INV == 0)
    {
        INVERTER_OFF();                                     // Block the Inverter PWM Pulses
        VFD_Status.bits.StartFlag = 0;
        VFD_Status.bits.FaultFlag = VFD_Status.bits.FaultFlag | 0x0001;  // Set the Fault Flag for Power Base Fault
    }

//-------------------------------------------------------------------------------------------------------------------
// 02 ABC pk Over Current Fault Protection // The logic is in a 20ms window , whenever the counter exceeds the count within 20ms window
//-------------------------------------------------------------------------------------------------------------------
 //   if (VFD_Data.SpeedRmp >  _IQ(0.2))


    Software_Current_Protection();

//-------------------------------------------------------------------------------------------------------------------
// If the over current exceeds the limit for 5ms within 20ms the trip activates.
//-------------------------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------------------------
// Inverter Voltage Sensing
//----------------------------------------------------------------------------------------------------------------------

 //  Inverter_Voltage_Sensing();

// --------------------------------------------------------------------------------------------------------------------
// Checks the state of the switch , if TOP or Bottom not triggers the fault occurs
//--------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
// These Faults are based on Current Decision
//-----------------------------------------------------------------------------------------------------------------

   if(VFD_Status.bits.StartFlag==1) // if User_Data.Run there is a delay in system.
      {
        Single_Phasing();
        One_PWM_Missing();
        Unbalance_Current();    // If current unbalance ocuurs this trip activates
      }



//-------------------------------------------------------------------------------------------------------------------
// 04 Heat Sink Over Temperature Protection
//-------------------------------------------------------------------------------------------------------------------

  //  Grid_Voltage_Sensing();

    Temperature_Sensing();

 //    DC_Link_Voltage_Sensing();

 //   HeatSinkFan_Sensing();

//   SMPSFan_Sensing();

 //   Enclosure_Temperature();

 //   Motor_Speed_Sensing();

}


void Software_Current_Protection()
{
    VFD_Data.Isa_abs = _IQabs(VFD_Data.Isa);                // Absolute value of Current
    VFD_Data.Isb_abs = _IQabs(VFD_Data.Isb);
    VFD_Data.Isc_abs = _IQabs(VFD_Data.Isc);

    VFD_Limits.AC_Current_window++;

    if(VFD_Data.Isa_abs > VFD_Limits.ABC_OC_Limit)         // If the R Phase Stator Current is above the Peak Current limit
      {
         VFD_Limits.AC_Current_Counter_Isa ++ ;
         if(VFD_Limits.AC_Current_Counter_Isa > VFD_Data.Sw_Freq * 15 )
         {
             INVERTER_OFF();
             VFD_Status.bits.FaultFlag = VFD_Status.bits.FaultFlag | 0x0002;
             VFD_Status.bits.StartFlag = 0;
             VFD_Limits.AC_Current_Counter_Isa = 0;
     //        GpioDataRegs.GPADAT.bit.GPIO26 =  GPIO_CLEAR;
          }
      }

     if(VFD_Data.Isb_abs > VFD_Limits.ABC_OC_Limit)         // If the R Phase Stator Current is above the Peak Current limit
      {
        VFD_Limits.AC_Current_Counter_Isb ++ ;
         if(VFD_Limits.AC_Current_Counter_Isb > VFD_Data.Sw_Freq * 15 )
         {
            INVERTER_OFF();
            VFD_Status.bits.FaultFlag = VFD_Status.bits.FaultFlag | 0x0002;
            VFD_Status.bits.StartFlag = 0;
            VFD_Limits.AC_Current_Counter_Isb = 0;
     //       GpioDataRegs.GPADAT.bit.GPIO26 =  GPIO_CLEAR;
         }
       }
       if(VFD_Data.Isc_abs > VFD_Limits.ABC_OC_Limit)         // If the R Phase Stator Current is above the Peak Current limit
       {
          VFD_Limits.AC_Current_Counter_Isc ++ ;
          if(VFD_Limits.AC_Current_Counter_Isc > VFD_Data.Sw_Freq * 15 )
          {
              INVERTER_OFF();
              VFD_Status.bits.FaultFlag = VFD_Status.bits.FaultFlag | 0x0002;
              VFD_Status.bits.StartFlag = 0;
              VFD_Limits.AC_Current_Counter_Isc = 0;
     //         GpioDataRegs.GPADAT.bit.GPIO26 =  GPIO_CLEAR;
           }
       }

        if( VFD_Limits.AC_Current_window >   VFD_Data.Sw_Freq * 20 )
         {
          VFD_Limits.AC_Current_window = 0;
          VFD_Limits.AC_Current_Counter_Isa = 0;
          VFD_Limits.AC_Current_Counter_Isb = 0;
          VFD_Limits.AC_Current_Counter_Isc = 0;
          VFD_Limits.Unbalance_current = 0;

         }
}


void Inverter_Voltage_Sensing()
{
    if((VFD_Status.bits.StartFlag ==1) && (VFD_Data.Vdc > _IQ(0.75)))
        {
           VFD_Limits.Vmot_Bot_Switch  = _IQmpy(VFD_Data.Vdc , _IQ(0.95)) ;
           VFD_Limits.Vmot_Top_Switch  = _IQ(0.09);
           if((VFD_Data.Vmotr  >  VFD_Limits.Vmot_Bot_Switch) || (VFD_Data.Vmotr  < VFD_Limits.Vmot_Top_Switch))
            {
             VFD_Limits.Vmot_Rph_Counter++ ;
             if(VFD_Limits.Vmot_Rph_Counter > VFD_Data.Sw_Freq * 20 )  // Verified this fault detects from 2ms to 100ms
                {
                 INVERTER_OFF();
                 VFD_Status.bits.FaultFlag = VFD_Status.bits.FaultFlag | 0x0004;
                 VFD_Limits.Vmot_Rph_Counter = 0;

                }
            }
             else
            VFD_Limits.Vmot_Rph_Counter = 0;
           if((VFD_Data.Vmoty  >  VFD_Limits.Vmot_Bot_Switch) || (VFD_Data.Vmoty  < VFD_Limits.Vmot_Top_Switch))
             {
              VFD_Limits.Vmot_Yph_Counter++ ;
              if(VFD_Limits.Vmot_Yph_Counter > VFD_Data.Sw_Freq * 20 )  // Verified this fault detects from 2ms to 100ms
                 {
                   INVERTER_OFF();
                   VFD_Status.bits.FaultFlag = VFD_Status.bits.FaultFlag | 0x0008;
                   VFD_Limits.Vmot_Yph_Counter = 0;
               //    GpioDataRegs.GPADAT.bit.GPIO26 = 0;
                }
             }
               else
                VFD_Limits.Vmot_Yph_Counter = 0;
           if((VFD_Data.Vmotb  >  VFD_Limits.Vmot_Bot_Switch) || (VFD_Data.Vmotb  < VFD_Limits.Vmot_Top_Switch))
              {
               VFD_Limits.Vmot_Bph_Counter++ ;
               if(VFD_Limits.Vmot_Bph_Counter > VFD_Data.Sw_Freq * 20 )  // Verified this fault detects from 2ms to 100ms
                 {
                    INVERTER_OFF();
                    VFD_Status.bits.FaultFlag = VFD_Status.bits.FaultFlag | 0x0010;
                    VFD_Limits.Vmot_Bph_Counter = 0;
               //     GpioDataRegs.GPADAT.bit.GPIO26 = 0;
                 }
              }
                else
                VFD_Limits.Vmot_Bph_Counter = 0;
       }

}

// --------------------------------------------------------------------------------------
// Single Phasing Fault
// -------------------------------------------------------------------------------------
void Single_Phasing()
{
//    VFD_Limits.zero_cross_counter++;
    VFD_Limits.AC_Zero_Crossing_Window++;
 //  if(VFD_Data.SpeedRmp > _IQ(0.1))
   {

    VFD_Limits.Isab_mirror = VFD_Data.Isa + VFD_Data.Isb ;

    VFD_Limits.Isac_mirror = VFD_Data.Isa + VFD_Data.Isc ;

    VFD_Limits.Isbc_mirror = VFD_Data.Isb + VFD_Data.Isc ;



    if((_IQ(-0.04) < VFD_Limits.Isbc_mirror) && (VFD_Limits.Isbc_mirror < _IQ(0.04)))         // 56A = 1 pu .04*56 2.24A to 2.24A 3*.04 // make the scale as .00212 .001 working
    {
    VFD_Limits.A_Phase_Missing++;
    if(VFD_Limits.A_Phase_Missing > VFD_Data.Sw_Freq * 50)  // if 5ms A phase is missing.
    {
    INVERTER_OFF();
    if(VFD_Data.Mot_Rot==0)
    VFD_Status.bits.FaultFlag = VFD_Status.bits.FaultFlag | 0x0020;
    else
    VFD_Status.bits.FaultFlag = VFD_Status.bits.FaultFlag | 0x0040;
    VFD_Status.bits.StartFlag = 0;
    VFD_Limits.A_Phase_Missing = 0;
 //   GpioDataRegs.GPADAT.bit.GPIO26 = 0;
    }
    }
 //   else
 //   VFD_Limits.A_Phase_Missing = 0;


    if((_IQ(-0.04) < VFD_Limits.Isac_mirror) && (VFD_Limits.Isac_mirror < _IQ(0.04)))
       {
       VFD_Limits.B_Phase_Missing++;
       if(VFD_Limits.B_Phase_Missing > VFD_Data.Sw_Freq * 50)
       {
       INVERTER_OFF();
       if(VFD_Data.Mot_Rot==0)
       VFD_Status.bits.FaultFlag = VFD_Status.bits.FaultFlag | 0x0040;
       else
       VFD_Status.bits.FaultFlag = VFD_Status.bits.FaultFlag | 0x0020;

       VFD_Status.bits.StartFlag = 0;
       VFD_Limits.B_Phase_Missing = 0;
   //    GpioDataRegs.GPADAT.bit.GPIO26 = 0;
       }
       }
    //   else
    //   VFD_Limits.B_Phase_Missing = 0;

    if((_IQ(-0.04) < VFD_Limits.Isab_mirror) && (VFD_Limits.Isab_mirror < _IQ(0.04)))
          {
          VFD_Limits.C_Phase_Missing++;
          if(VFD_Limits.C_Phase_Missing > VFD_Data.Sw_Freq * 50)
          {
          INVERTER_OFF();
          VFD_Status.bits.FaultFlag = VFD_Status.bits.FaultFlag | 0x0080;
          VFD_Status.bits.StartFlag = 0;
          VFD_Limits.C_Phase_Missing = 0;
     //     GpioDataRegs.GPADAT.bit.GPIO26 = 0;
          }
          }
    if(VFD_Limits.AC_Zero_Crossing_Window > VFD_Data.Sw_Freq * 60)
    {
            VFD_Limits.AC_Zero_Crossing_Window = 0;
            VFD_Limits.A_Phase_Missing = 0;
            VFD_Limits.B_Phase_Missing = 0;
            VFD_Limits.C_Phase_Missing = 0;
    }
     //     else
     //     VFD_Limits.C_Phase_Missing = 0;
   }

}

// -----------------------------------------------------------------------------------------------
// Multiple Zero Crossing
//-----------------------------------------------------------------------------------------------

void One_PWM_Missing()
{

    if(VFD_Data.SpeedRmp < _IQ(0.2))
    {
    VFD_Limits.Low_freq = _IQmpy(_IQ(0.75),  VFD_Data.SpeedRmp) ;
    VFD_Limits.Hig_freq   = _IQmpy(_IQ(1.25), VFD_Data.SpeedRmp);
    }
    else
    {
     VFD_Limits.Low_freq = _IQmpy(_IQ(0.9),  VFD_Data.SpeedRmp) ;
     VFD_Limits.Hig_freq   = _IQmpy(_IQ(1.1), VFD_Data.SpeedRmp);
    }

    if((VFD_Data.Isa) > _IQ(0.0))
    {
    GpioDataRegs.GPADAT.bit.GPIO27 =  GPIO_CLEAR ;
    VFD_Limits.A_Phase_Flag = 1;
    }

    if(VFD_Limits.A_Phase_Flag ==1 )
    VFD_Limits.A_Phase_counter++;

    if((VFD_Data.Isa < _IQ(0.0)) && VFD_Limits.A_Phase_Flag ==1)
    {
        VFD_Limits.A_phase_temp = VFD_Limits.A_Phase_counter ;
        VFD_Limits.A_Phase_counter = 0;
        if(VFD_Limits.A_Phase_Flag == 1)
        VFD_Limits.A_Phase_Flag = 0;
    VFD_Limits.freq_A_Init_Flag_zero = 1;
    VFD_Limits.A_Phase_freq =  _IQ(( VFD_Data.Sw_Freq *1000 / (VFD_Limits.A_phase_temp * 2 * User_Data.Frated )));
    }

    if((VFD_Limits.A_Phase_freq >  VFD_Limits.Low_freq) && (VFD_Limits.A_Phase_freq < VFD_Limits.Hig_freq))
    VFD_Limits.A_Phase_Freq_Miss = 0;
    else
    {
        VFD_Limits.A_Phase_Freq_Miss++;
   //    VFD_Limits.A_Phase_Debug = 0;
        if(VFD_Limits.A_Phase_Freq_Miss > VFD_Data.Sw_Freq * 500)
        {
         VFD_Limits.A_Phase_Missing = 0;
         if(VFD_Data.Mot_Rot==0)
         VFD_Status.bits.FaultFlag = VFD_Status.bits.FaultFlag | 0x0100;
         else
         VFD_Status.bits.FaultFlag = VFD_Status.bits.FaultFlag | 0x0200;
         INVERTER_OFF();
        }
    }


   if((VFD_Data.Isb) > _IQ(0.0))
   VFD_Limits.B_Phase_Flag = 1;

   if(VFD_Limits.B_Phase_Flag ==1 )
   VFD_Limits.B_Phase_counter++;

   if((VFD_Data.Isb < _IQ(0.0)) && VFD_Limits.B_Phase_Flag ==1)
   {
       VFD_Limits.B_phase_temp = VFD_Limits.B_Phase_counter ;
       VFD_Limits.B_Phase_counter = 0;
       if(VFD_Limits.B_Phase_Flag == 1)
       VFD_Limits.B_Phase_Flag = 0;
       VFD_Limits.freq_B_Init_Flag_zero = 1;
   VFD_Limits.B_Phase_freq =  _IQ(( VFD_Data.Sw_Freq *1000 / (VFD_Limits.B_phase_temp * 2 *User_Data.Frated )));
   }


   if((VFD_Limits.B_Phase_freq >  VFD_Limits.Low_freq) && (VFD_Limits.B_Phase_freq < VFD_Limits.Hig_freq))
   VFD_Limits.B_Phase_Freq_Miss = 0;
   else
   {
       VFD_Limits.B_Phase_Freq_Miss++;
//        VFD_Limits.A_Phase_Debug = 0;
       if(VFD_Limits.B_Phase_Freq_Miss > VFD_Data.Sw_Freq * 500)
       {
        VFD_Limits.B_Phase_Missing = 0;
  //      if(VFD_Data.Mot_Rot==0)
        VFD_Status.bits.FaultFlag = VFD_Status.bits.FaultFlag | 0x0200;
  //      else
  //      VFD_Status.bits.FaultFlag = VFD_Status.bits.FaultFlag | 0x0100;
        INVERTER_OFF();
       }
   }

   if((VFD_Data.Isc) > _IQ(0.0))
   VFD_Limits.C_Phase_Flag = 1;

   if(VFD_Limits.C_Phase_Flag ==1 )
   VFD_Limits.C_Phase_counter++;

   if((VFD_Data.Isc < _IQ(0.0)) && VFD_Limits.C_Phase_Flag ==1)
   {
       VFD_Limits.C_phase_temp = VFD_Limits.C_Phase_counter ;
       VFD_Limits.C_Phase_counter = 0;
       if(VFD_Limits.C_Phase_Flag == 1)
       VFD_Limits.C_Phase_Flag = 0;
   VFD_Limits.freq_C_Init_Flag_zero = 1;
   VFD_Limits.C_Phase_freq =  _IQ(( VFD_Data.Sw_Freq *1000 / (VFD_Limits.C_phase_temp * 2 * User_Data.Frated )));
   }


   if((VFD_Limits.C_Phase_freq > VFD_Limits.Low_freq) && (VFD_Limits.C_Phase_freq < VFD_Limits.Hig_freq))
   {
   VFD_Limits.C_Phase_Freq_Miss = 0;
   VFD_Limits.A_Phase_Debug = 1;
   }
   else
   {
       VFD_Limits.C_Phase_Freq_Miss++;
       VFD_Limits.A_Phase_Debug = 0;
       if(VFD_Limits.C_Phase_Freq_Miss > VFD_Data.Sw_Freq * 500)
       {
        VFD_Status.bits.FaultFlag = VFD_Status.bits.FaultFlag | 0x0400;
        VFD_Limits.C_Phase_Missing = 0;
        INVERTER_OFF();
       }
   }

}

/*
void Phase_Sequence()
{
 if((VFD_Data.Isa < _IQ(0.0) && VFD_Data.Isc < _IQ(0.0)) && VFD_Limits.Phase_Sequence_Flag ==0)
 VFD_Limits.Phase_sequence++;
// VFD_Status.bits.FaultFlag = VFD_Status.bits.FaultFlag | 0x0800;
}
 */

void Unbalance_Current()
{
  VFD_Data.Resultant_Current_abs = _IQabs(VFD_Data.Resultant_Current);
  if(VFD_Data.Resultant_Current_abs > _IQ (0.2)) // .1 corresponds to 5.56A... .01 .05A
  {
      VFD_Limits.Unbalance_current++;
      if(VFD_Limits.Unbalance_current > VFD_Data.Sw_Freq * 15 )
      {
       INVERTER_OFF();
       VFD_Status.bits.FaultFlag = VFD_Status.bits.FaultFlag | 0x0800;
       VFD_Limits.Unbalance_current  =  0;
      }
  }

}

void Grid_Voltage_Sensing()
{
  VFD_Limits.Vsalphagrid   =  VFD_Data.Vgry ; // - _IQmpy(_IQ(0.5), Vyb) - _IQmpy(_IQ(0.5),Vrb); // Ia - _IQmpy(_IQ(0.5), Ib) - _IQmpy(_IQ(0.5), Ic);
  VFD_Limits.Vsbetagrid    =  _IQmpy((VFD_Data.Vgyb - VFD_Data.Vgrb),_IQ(Sqrt3_1));                 //_IQmpy((Ib - Ic),_IQ(Sqrt3_1));
  VFD_Limits.thetagrid     =  _IQatan2PU(VFD_Limits.Vsbetagrid,VFD_Limits.Vsalphagrid);
  VFD_Limits.CosThetagrid  =  _IQcosPU( VFD_Limits.thetagrid );
  VFD_Limits.SineThetagrid =  _IQsinPU(VFD_Limits.thetagrid );
  VFD_Limits.Vdgrid        =  _IQmpy(VFD_Limits.Vsalphagrid,VFD_Limits.CosThetagrid) + _IQmpy( VFD_Limits.Vsbetagrid,VFD_Limits.SineThetagrid);

  if (VFD_Limits.Vdgrid < _IQ(0.85) )      //.85     // 10 cycles of i/p voltage
  {
   VFD_Limits.InpCyc_cnt++;
   if(VFD_Limits.InpCyc_cnt > VFD_Data.Sw_Freq * 300) // Input missing for 300ms
   {
      VFD_Status.bits.FaultFlag = VFD_Status.bits.FaultFlag | 0x1000;
      INVERTER_OFF();
      VFD_Limits.InpCyc_cnt = 0;

   }
  }
      else
      VFD_Limits.InpCyc_cnt = 0;

  VFD_Data.GridTmp     = VFD_Limits.Vdgrid  - VFD_Data.GridFilt;
  VFD_Data.GridFilt    = VFD_Data.GridFiltOld + _IQmpyIQX(VFD_Data.GridTc, 15, VFD_Data.GridTmp, GLOBAL_Q);
  VFD_Data.GridFiltOld = VFD_Data.GridFilt;


  if(VFD_Data.GridFilt < _IQ(0.8))  //.8
  {
      VFD_Limits.Inp_phase_unbalance_cnt++;
      if(VFD_Limits.Inp_phase_unbalance_cnt > VFD_Data.Sw_Freq * 100) // Input missing for 100 seconds
         {
            VFD_Status.bits.FaultFlag = VFD_Status.bits.FaultFlag | 0x2000;
            INVERTER_OFF();
            VFD_Limits.Inp_phase_unbalance_cnt = 0;
      //      GpioDataRegs.GPADAT.bit.GPIO26 = 0;
         }
  }
  else
  VFD_Limits.Inp_phase_unbalance_cnt = 0;

}

void Temperature_Sensing()

{
    if (VFD_Data.Rhsk < _IQ(1.3))   // .8 for 80C // .7 for 85C // .5 for 100C // 1k 70C
        {
       //   if(MOT_RGI.EqualFlag == 0x1FFF)
          {
            INVERTER_OFF();
              VFD_Limits.Temparature_cnt ++ ;
            if(VFD_Limits.Temparature_cnt >  VFD_Data.Sw_Freq * 300) // 300 mseconds
           {
           INVERTER_OFF();
           VFD_Status.bits.FaultFlag = VFD_Status.bits.FaultFlag | 0x4000;
           VFD_Limits.Temparature_cnt = 0;
           }
          }
        }
        else
            VFD_Limits.Temparature_cnt = 0;
}

void DC_Link_Voltage_Sensing()
{
    if (VFD_Data.Vdc > VFD_Limits.DC_OV_Limit)
       {                           // Check the DC Bus Voltage > DC Over Voltage limit
           INVERTER_OFF();                                     // Block the Inverter PWM Pulses
   //        Dyn_Brake_Con();                                    // Turn on Dynamic Brake
           VFD_Status.bits.StartFlag = 0;
           VFD_Status.bits.FaultFlag = VFD_Status.bits.FaultFlag | 0x8000;  // Update the FaultFlag, online status
       }

      else if (VFD_Data.Vdc < VFD_Limits.DC_UV_Limit)
      {                           // Check the DC Bus voltage < DC Under Voltage limit
          VFD_Limits.DC_UV_counter++;
          if(VFD_Limits.DC_UV_counter > VFD_Data.Sw_Freq * 200) // Input missing for 100 seconds
           {
        //    VFD_Status.bits.FaultFlag = VFD_Status.bits.FaultFlag | 0x10000;
        //    INVERTER_OFF();
            VFD_Limits.DC_UV_counter = 0;
           }
       }
       else
       {                                                       // Reset DC Under Voltage Fault if DC Bus restored
           VFD_Limits.DC_UV_counter  = 0;
       }

}

void HeatSinkFan_Sensing()
{
    if(VFD_Data.HeatSink_Fan < _IQ(0.75))
    {
        VFD_Limits.Heat_Sink_Fan_counter++;
        if(VFD_Limits.Heat_Sink_Fan_counter > VFD_Data.Sw_Freq * 100)   // 100ms
        {
            VFD_Status.bits.FaultFlag = VFD_Status.bits.FaultFlag | 0x20000;
            INVERTER_OFF();
            VFD_Limits.Heat_Sink_Fan_counter = 0;
            GpioDataRegs.GPADAT.bit.GPIO26 = 0;
        }
    }
    else
    VFD_Limits.Heat_Sink_Fan_counter   = 0 ;
}


void SMPSFan_Sensing()
{
    if(VFD_Data.SMPS_Fan < _IQ(0.75))
        {
            VFD_Limits.SMPS_Fan_counter++;
            if(VFD_Limits.SMPS_Fan_counter > VFD_Data.Sw_Freq * 100)   // 100ms
            {
           //     VFD_Status.bits.FaultFlag = VFD_Status.bits.FaultFlag | 0x40000;
           //     INVERTER_OFF();
                VFD_Limits.SMPS_Fan_counter = 0;
            }
        }
        else
        VFD_Limits.SMPS_Fan_counter   = 0 ;
}

void Enclosure_Temperature()
{
    if(VFD_Data.EncTem >  VFD_Limits.Enc_OT_Limit)
       {
           VFD_Limits.Enclosure_Temp_counter++;
           if(VFD_Limits.Enclosure_Temp_counter > VFD_Data.Sw_Freq * 100)   // 100ms
           {
               VFD_Status.bits.FaultFlag = VFD_Status.bits.FaultFlag | 0x80000;
               INVERTER_OFF();
               VFD_Limits.Enclosure_Temp_counter = 0;
           }
       }
       else
       VFD_Limits.Enclosure_Temp_counter   = 0 ;  ;
}


void Motor_Speed_Sensing()
{

    //-------------------------------------------------------------------------------------------------------------------
    // 13 Motor Over Speed Fault
    //-------------------------------------------------------------------------------------------------------------------
       VFD_Data.Speed_Error = VFD_Data.SpeedRmp - VFD_Data.SpeedAct ;
       // VFD_Data.Speed_Error  = _IQabs(VFD_Data.Speed_Error);

       if (VFD_Data.SpeedRmp >  _IQ(0.32))
       {
        if(VFD_Data.Speed_Error > _IQ(0.1))
       {                                                       // Speed Difference b/n Speed Ref & Estimated Speed > 10% of Rated Speed
           VFD_Limits.Speed_cnt ++ ;
           if(VFD_Limits.Speed_cnt > VFD_Data.Sw_Freq * 100)
           {
           INVERTER_OFF();                                     // Block the Inverter PWM Pulses
           VFD_Status.bits.StartFlag = 0;
           VFD_Limits.Speed_cnt = 0 ;
           VFD_Status.bits.FaultFlag = VFD_Status.bits.FaultFlag | 0x100000;  // Update the FaultFlag, online status
           }
       }
        else
        VFD_Limits.Speed_cnt = 0;
       }

     /*  else if (VFD_Data.SpeedAct > _IQ(1.1))                  // Actual Speed > 150% of rated Speed
       {
           INVERTER_OFF();                                     // Block the Inverter PWM Pulses
           VFD_Status.bits.StartFlag = 0;
           VFD_Status.bits.FaultFlag = VFD_Status.bits.FaultFlag | 0x1000;  // Update the FaultFlag, online status
       }
    */
}

/*
// After Closed Loop d-q current protection is active.
//-------------------------------------------------------------------------------------------------------------------
// 03 D-Q Over Current Protection
//-------------------------------------------------------------------------------------------------------------------

   if( VFD_Status.bits.EpsThetFlag==1)
   {
    if(VFD_Data.Isq > VFD_Limits.Qs_OC_Limit)         // If the Q Axis Current is above the Peak Current limit
    {
        INVERTER_OFF();                                     // Block the Inverter PWM Pulses
        VFD_Status.bits.StartFlag = 0;
        VFD_Status.bits.FaultFlag = VFD_Status.bits.FaultFlag | 0x0004;  // Update the FaultFlag, online status

    }
    else if(VFD_Data.Isd > VFD_Limits.Ds_OC_Limit)    // If the D Axis Current is above the Peak Current limit
    {
        INVERTER_OFF();                                     // Block the Inverter PWM Pulses
        VFD_Status.bits.StartFlag = 0;
        VFD_Status.bits.FaultFlag = VFD_Status.bits.FaultFlag | 0x0004;  // Update the FaultFlag, online status

    }
   }
*/
