//==================================================================================================================================================
// FILE NAME		: Funcs_1_Periph.c
// DATE             : 01-Feb-2018
// Project			: VARIABLE FREQUENCY DRIVE FOR COMPRESSOR CONTROL APPLICATIONS
// Project Code		: PEG 124B
// Author           : Rohit V Thomas[ELGI], Manju R[CDAC] & Rinu J Vijyan[CDAC]
//==================================================================================================================================================
// Include Header Files....
//==================================================================================================================================================
#include "DSP28234_Device.h"
#include "variable.h"
#include "DSP28234_GlobalPrototypes.h"
#include "IQmathLib.h"




//--------------------------------------------------------------------------------------------------------------------------------------------------
// Only Line Currents are read in ISR
//--------------------------------------------------------------------------------------------------------------------------------------------------
void ADC_READ()
{

    AdcRegs.ADCTRL2.bit.RST_SEQ1        = 1;                // Reset Seq 1,             0=> No action
                                                                      //                          1=> Immediate Reset Seq 1 to initial state
    AdcRegs.ADCTRL2.bit.SOC_SEQ1        = 1;

  //  delay(50) ;
    while(!AdcRegs.ADCST.bit.SEQ1_BSY==0 ) ;

    VFD_IntAdcData.Vgry = AdcRegs.ADCRESULT3>>4;

    VFD_IntAdcData.Vgyb =  AdcRegs.ADCRESULT4>>4;

    VFD_IntAdcData.Mon_Enc = AdcRegs.ADCRESULT5>>4 ;

    VFD_IntAdcData.Tach_EF  = AdcRegs.ADCRESULT6>>4;

    VFD_IntAdcData.Tach_HF = AdcRegs.ADCRESULT7>>4;

    VFD_IntAdcData.Mon_HST  = AdcRegs.ADCRESULT8>>4;

     VFD_IntAdcData.Mon_Vdc = AdcRegs.ADCRESULT9>>4;

     VFD_IntAdcData.Vmot_Rph = AdcRegs.ADCRESULT10>>4;

     VFD_IntAdcData.Vmot_Yph = AdcRegs.ADCRESULT11>>4;

     VFD_IntAdcData.Vmot_Bph = AdcRegs.ADCRESULT12>>4;

     VFD_IntAdcData.Mon_Pt100     = AdcRegs.ADCRESULT14>>4 ;

     VFD_IntAdcData.SpdRef = AdcRegs.ADCRESULT15>>4; // 1.64V Offset from AMC1311 = 840


    if (VFD_Data.Mot_Rot == 0)    // AntiClockwise Motor Rotation (default)
    {
        VFD_IntAdcData.Isa     =  AdcRegs.ADCRESULT0>>4; // Read the results, 4 bit shift for avoiding sign
	    VFD_IntAdcData.Isb      = AdcRegs.ADCRESULT1>>4;
	    VFD_IntAdcData.Isc      = AdcRegs.ADCRESULT2>>4;
    }
	if (VFD_Data.Mot_Rot == 1)    // Clockwise Motor Rotation
    {
	    VFD_IntAdcData.Isb     =  AdcRegs.ADCRESULT0>>4; // Read the results, 4 bit shift for avoiding sign
	    VFD_IntAdcData.Isa      = AdcRegs.ADCRESULT1>>4;
	    VFD_IntAdcData.Isc      = AdcRegs.ADCRESULT2>>4;
    }

//	 AdcRegs.ADCTRL2.all = 0x4040 ; // Wait till SOC

	 VFD_IntAdcData.offset_counts_Isa =  VFD_IntAdcData.Isa - VFD_Data.offset_Isa ;  // 165 count offset is observed to make it zero
	 VFD_IntAdcData.offset_counts_Isb =  VFD_IntAdcData.Isb - VFD_Data.offset_Isb  ;  // 117mV Offset Voltage is observed
	 VFD_IntAdcData.offset_counts_Isc =  VFD_IntAdcData.Isc - VFD_Data.offset_Isc ;


	// Passing scaled values to VFD_Data
	VFD_Data.Isa            = _IQmpyIQX(_IQ16(VFD_IntAdcData.offset_counts_Isa),16, -(VFD_IntAdcData.Curr_Scal), GLOBAL_Q);	// Scaled to 1pu - 60A
	VFD_Data.Isb            = _IQmpyIQX(_IQ16(VFD_IntAdcData.offset_counts_Isb),16, -(VFD_IntAdcData.Curr_Scal), GLOBAL_Q);	// Scaled to 1pu - 60A
    VFD_Data.Isc            = _IQmpyIQX(_IQ16(VFD_IntAdcData.offset_counts_Isc),16, -(VFD_IntAdcData.Curr_Scal), GLOBAL_Q);  // Scaled to 1pu - 60A
 //   VFD_Data.Isc            = - (VFD_Data.Isa + VFD_Data.Isb ) ;

    VFD_Data.Resultant_Current = VFD_Data.Isa + VFD_Data.Isb ;

    VFD_Data.Resultant_Current = VFD_Data.Resultant_Current + VFD_Data.Isc ;


        adc_init_mot_volt ++;

        VFD_Data.average_Vmot_r = ( AdcRegs.ADCRESULT10>>4) + VFD_Data.average_Vmot_r;

        VFD_Data.average_Vmot_y = (AdcRegs.ADCRESULT11>>4)  + VFD_Data.average_Vmot_y;

        VFD_Data.average_Vmot_b =  (AdcRegs.ADCRESULT12>>4) + VFD_Data.average_Vmot_b;

        if( adc_init_mot_volt > 7)
        {
        adc_init_mot_volt = 0;
        VFD_IntAdcData.Vmot_Rph = VFD_Data.average_Vmot_r >> 3;
        VFD_IntAdcData.Vmot_Yph = VFD_Data.average_Vmot_y >> 3;
        VFD_IntAdcData.Vmot_Bph = VFD_Data.average_Vmot_b >> 3;
        VFD_Data.average_Vmot_r = 0;
        VFD_Data.average_Vmot_y = 0;
        VFD_Data.average_Vmot_b = 0;
        VFD_Data. Vmotr        = _IQmpyIQX(_IQ16(VFD_IntAdcData.Vmot_Rph),16, VFD_IntAdcData.Volt_Scal, GLOBAL_Q);
        VFD_Data. Vmoty        = _IQmpyIQX(_IQ16(VFD_IntAdcData.Vmot_Yph),16, VFD_IntAdcData.Volt_Scal, GLOBAL_Q);
        VFD_Data. Vmotb        = _IQmpyIQX(_IQ16(VFD_IntAdcData.Vmot_Bph),16, VFD_IntAdcData.Volt_Scal, GLOBAL_Q);
        }

}

//--------------------------------------------------------------------------------------------------------------------------------------------------
// DAC Display Function
//--------------------------------------------------------------------------------------------------------------------------------------------------
/*int DAC_Display(int dacdata)
{
	signed int tempA;
	signed int tempB;
	tempA	 =	(int)dacdata >> 3; //  (int)dacdata >> 6
	tempB	 =  tempA;
	tempB	 = 	tempB + 0x1FFF; // tempB + 0x1FF;

	return(tempB);
} */
//--------------------------------------------------------------------------------------------------------------------------------------------------
// EXTERNAL DAC Write
//--------------------------------------------------------------------------------------------------------------------------------------------------
void EXT_DAC_Write(int DAC1_DATA,int DAC2_DATA,int DAC3_DATA,int DAC4_DATA)
{


    int  dummy_dac_data = 0;
    dac_channel++;
 //   SpiaRegs.SPITXBUF = 0x0020 | counter >> 10;
 //   SpiaRegs.SPITXBUF = 0x0000 | counter << 4;//
   if(dac_channel ==1)
   {
    dummy_dac_data  = (DAC1_DATA + 0x3FFF ) ;
    SpiaRegs.SPITXBUF = 0x0200 | dummy_dac_data >> 11;
    SpiaRegs.SPITXBUF = 0x0000 |  dummy_dac_data << 5;//
  //  dac_channel = 0;
    } //
   if(dac_channel ==2)
    {
    dummy_dac_data  = (DAC2_DATA + 0x3FFF ) ;
    SpiaRegs.SPITXBUF = 0x0210 | dummy_dac_data >> 11;
    SpiaRegs.SPITXBUF = 0x0000 |  dummy_dac_data << 5;//
 //   dac_channel = 0;
    }
    if(dac_channel ==3)
    {
     dummy_dac_data  = (DAC3_DATA + 0x3FFF ) ;
     SpiaRegs.SPITXBUF = 0x0220 | dummy_dac_data >> 11;
     SpiaRegs.SPITXBUF = 0x0000 |  dummy_dac_data << 5;//
  //   dac_channel = 0;
    }
   if(dac_channel ==4)
    {
     dummy_dac_data  = (DAC4_DATA + 0x3FFF ) ;
     SpiaRegs.SPITXBUF = 0x0230 | dummy_dac_data >> 11;
     SpiaRegs.SPITXBUF = 0x0000 |  dummy_dac_data << 5;//
     dac_channel = 0;
    }

}
//--------------------------------------------------------------------------------------------------------------------------------------------------
// INTERNAL ADC Read
//--------------------------------------------------------------------------------------------------------------------------------------------------
void ADC_Processing()
{
// INTERNAL ADC Start of Conversion
//

// INTERNAL ADC End of Conversion

								// Sign extension mode


//	VFD_IntAdcData.Mon_HST      = VFD_IntAdcData.Mon_HST & 0xFFF0 ;
	//   VFD_IntAdcData.Mon_HST      = VFD_IntAdcData.Mon_HST *1.6; // Differential Opamp Req


	// DC Link Voltage Scaling to Per Unit
//	VFD_Data.Vdc    = _IQmpyIQX(_IQ16(VFD_IntAdcData.Mon_Vdc), 16, VFD_IntAdcData.VdcVlt_Scal, GLOBAL_Q);

//	VFD_IntAdcData.Vmot_RYph = VFD_IntAdcData.Vmot_Rph - VFD_IntAdcData.Vmot_Yph;
//	VFD_IntAdcData.Vmot_YBph = VFD_IntAdcData.Vmot_Yph - VFD_IntAdcData.Vmot_Bph;
//	VFD_IntAdcData.Vmot_BRph = VFD_IntAdcData.Vmot_Bph - VFD_IntAdcData.Vmot_Rph;


	VFD_IntAdcData.offset_counts_Vgry = VFD_IntAdcData.Vgry - VFD_Data.offset_Isa ;
	VFD_IntAdcData.offset_counts_Vgyb = VFD_IntAdcData.Vgyb - VFD_Data.offset_Isa ;

	VFD_Data.Vgry           = _IQmpyIQX(_IQ16(VFD_IntAdcData.offset_counts_Vgry),16,  VFD_IntAdcData.Volt_grid_Scal, GLOBAL_Q);   // Scaled to 1pu - Vrated
    VFD_Data.Vgyb           = _IQmpyIQX(_IQ16(VFD_IntAdcData.offset_counts_Vgyb),16,  VFD_IntAdcData.Volt_grid_Scal, GLOBAL_Q);   // Scaled to 1pu - Vrated
    VFD_Data.Vgrb           = - (VFD_Data.Vgry + VFD_Data.Vgyb) ;

    VFD_Data.offset_speed    = VFD_IntAdcData.SpdRef - 2831 ;
    VFD_Data.SpdRef         = _IQmpyIQX(_IQ16(VFD_IntAdcData.SpdRef),16, VFD_IntAdcData.SpdRef_Scal, GLOBAL_Q);    // Scaled to 1pu - 2.5V
/*
    VFD_Data. Vmotry        = _IQmpyIQX(_IQ16(VFD_IntAdcData.Vmot_RYph),16, VFD_IntAdcData.Volt_Scal, GLOBAL_Q);
    VFD_Data. Vmotyb        = _IQmpyIQX(_IQ16(VFD_IntAdcData.Vmot_YBph),16, VFD_IntAdcData.Volt_Scal, GLOBAL_Q);
    VFD_Data. Vmotbr       = _IQmpyIQX(_IQ16(VFD_IntAdcData.Vmot_BRph),16, VFD_IntAdcData.Volt_Scal, GLOBAL_Q);
*/
    // Scaling Vdc


    // Low Pass Filter for Vdc              // VdcFilt = VdcFiltOld + VdcTc*(Vdc - VdcFilt)
  /*  VFD_Data.VdcTmp     = VFD_Data.Vdc - VFD_Data.VdcFilt;
    VFD_Data.VdcFilt    = VFD_Data.VdcFiltOld + _IQmpyIQX(VFD_Data.VdcTc, 15, VFD_Data.VdcTmp, GLOBAL_Q);
    VFD_Data.VdcFiltOld = VFD_Data.VdcFilt; */

	// NTC Resistance calculation : Rntc = Rst * Vadc/(5-Vadc)
//	VFD_Data.Renc   = _IQmpyIQX(_IQ16(VFD_IntAdcData.Mon_Enc), 16, VFD_IntAdcData.EncTem_Scal, GLOBAL_Q);   // Scaling to 2.5V
//	VFD_Data.Renc   = _IQdiv(VFD_Data.Renc, (_IQ(5.0) - VFD_Data.Renc));      // Vadc/(5-Vadc), times of Rst

	VFD_Data.Rhsk   = _IQmpyIQX(_IQ16(VFD_IntAdcData.Mon_HST), 16, VFD_IntAdcData.HskTem_Scal, GLOBAL_Q);   // Scaling to 2.5V
	if(VFD_Data.Rhsk > _IQ(4.5))
	VFD_Data.Rhsk = _IQ(4.5) ;
	VFD_Data.Rhsk   = _IQdiv(VFD_Data.Rhsk, (_IQ(5.190) - VFD_Data.Rhsk));
	VFD_Data.Rhsk   = _IQmpy((VFD_Data.Rhsk),_IQ(15));

    VFD_Data.HeatSink_Fan =  _IQmpyIQX(_IQ16(VFD_IntAdcData.Tach_HF), 16, _IQ(0.000366), GLOBAL_Q);
	// NTC Temperature calculation : Tntc = 298.15*B / [298.15*ln(Rntc/Rst) + B], in Kelvin

    VFD_Data.offset_Enclosure_Temp = VFD_IntAdcData.Mon_Enc - 682 ; // offset counts --> 682 corresponds to .500mV
	VFD_Data.EncTem   = _IQmpyIQX(_IQ16(VFD_Data.offset_Enclosure_Temp), 16, _IQ(0.002929), GLOBAL_Q);// Refer Controller Card Analog Calculations

	VFD_Data.SMPS_Fan =  _IQmpyIQX(_IQ16(VFD_IntAdcData.Tach_EF), 16, _IQ(0.000366), GLOBAL_Q);


	VFD_Data.Vdc    = _IQmpyIQX(_IQ16(VFD_IntAdcData.Mon_Vdc), 16, VFD_IntAdcData.VdcVlt_Scal, GLOBAL_Q);
	VFD_Data.VdcTmp     = VFD_Data.Vdc  - VFD_Data.VdcFilt;
	VFD_Data.VdcFilt    = VFD_Data.VdcFiltOld + _IQmpyIQX(VFD_Data.VdcTc, 15, VFD_Data.VdcTmp, GLOBAL_Q);
	VFD_Data.VdcFiltOld = VFD_Data.VdcFilt;

	//	VFD_Data.EncTem   = _IQ15log(VFD_Data.EncTem);
//	VFD_Data.EncTem   = _IQ15mpy(VFD_Data.EncTem, _IQ15(298.15)) + _IQ15(Beta_ETntc);
//	VFD_Data.EncTem   = _IQ15div(_IQ15(Beta_ETntc), VFD_Data.EncTem);
//	VFD_Data.EncTem   = _IQ15mpy(_IQ15(298.15), VFD_Data.EncTem) - _IQ15(273) + _IQ15(00);   // 00K correction


//	if(temperature_counter > 45)
//	{
/*	VFD_Data.RhskTmp     = VFD_Data.Rhsk - VFD_Data.RhskFilt;
	VFD_Data.RhskFilt    = VFD_Data.RhskFiltOld + _IQmpyIQX(VFD_Data.HskTemTc, 15, VFD_Data.RhskTmp, GLOBAL_Q);
	VFD_Data.RhskFiltOld = VFD_Data.RhskFilt;
//	} */
/*	VFD_Data.HskTem   = _IQ15mpyIQX(VFD_Data.RhskFilt, GLOBAL_Q, _IQ(0.2), GLOBAL_Q); // _IQ(.2) for 1/5
	if(VFD_Data.HskTem < _IQ15(0.1))
	VFD_Data.HskTem = _IQ15(0.1) ; // As LOG O is not defined.
	VFD_Data.HskTem   = _IQ15log(VFD_Data.HskTem); // IQ15log(_IQ(3.443451226));//
	VFD_Data.HskTem   = _IQ15mpy(VFD_Data.HskTem , _IQ15(.000293)); //  1/(beta)
	VFD_Data.HskTem   =  _IQ15(0.003354) + VFD_Data.HskTem ;
	VFD_Data.HskTem   = _IQ15div(_IQ15(1.0), VFD_Data.HskTem);
	VFD_Data.HskTem   = VFD_Data.HskTem - _IQ15(273.15);
	temperature_counter = 0;
	}
*/
   	// Scaling Pt100
//	VFD_Data.Rpt100   = _IQmpyIQX(_IQ16(VFD_IntAdcData.Mon_Pt100), 16, VFD_IntAdcData.Pt100_Scal, GLOBAL_Q);
//    VFD_Data.Rpt100   = _IQdiv(VFD_Data.Rpt100, (VFD_Data.Dig3V - VFD_Data.Rpt100));      // Vadc/(3.3-Vadc), times of Rref

    // Pt100 Calculation : Pt100 = 1/Alpha * (Rpt/Rref - 1)
    //VFD_Data.Pt100    = _IQ15mpyIQX(_IQ15(Pt100_Alp_1), 15, (VFD_Data.Rpt100 - 1), GLOBAL_Q );

}

//--------------------------------------------------------------------------------------------------------------------------------------------------
// Delay Function - us
//--------------------------------------------------------------------------------------------------------------------------------------------------
void delay(unsigned int count)
{
	int j;
	for(j=0; j < (VFD_Data.Sw_Freq * count); j++)
	{
		asm(" NOP");
	}
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
// Fault Latch Reset....
//--------------------------------------------------------------------------------------------------------------------------------------------------
void Fault_Latch_Reset()
{
    Latch_Reset_Ctrl = 0; 		// Writing Low-High to LATCH_RST
    delay(3);
    Latch_Reset_Ctrl = 1;		// Writing Low-High to LATCH_RST
    delay(3);
    Latch_Reset_Ctrl = 0;       // Writing Low-High to LATCH_RST
    delay(3);
    Latch_Reset_Ctrl = 1;       // Writing Low-High to LATCH_RST
    delay(3);

  //  RELAY_PTC_Ctrl   = 0;       // Turning On Relay, Inverted
    DYN_BRK_Ctrl     = 1;       // Turning Off Dyn Brake, Inverter
  //  RELAY_PTC_Ctrl   = 0;




}



//--------------------------------------------------------------------------------------------------------------------------------------------------
// Code ends here....
//--------------------------------------------------------------------------------------------------------------------------------------------------
