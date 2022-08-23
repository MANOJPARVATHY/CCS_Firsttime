//==================================================================================================================================================
// FILE NAME        : variable.c
// DATE             : 01-Feb-2018
// Project          : VARIABLE FREQUENCY DRIVE FOR COMPRESSOR CONTROL APPLICATIONS
// Project Code     : PEG 124B
// Author           : Rohit V Thomas[ELGI], Manju R[CDAC], Rinu J Vijyan [CDAC], Prashobh [CDAC], Sapna Ravindran[CDAC]
//==================================================================================================================================================
// Include Header Files....
//==================================================================================================================================================
//--------------------------------------------------------------------------------------------------------------------------------------------
#include "DSP28234_Device.h"
#include "IQmathLib.h"

//############################################################################################################################################
// Define the Motor Control Mode
#define MODE11      11              // 11 = INDM V/F - Open Loop
#define MODE12      12              // 12 = INDM V/F - Closed Loop
#define MODE13      13              // 13 = INDM FOC - Sensorless

#define MODE21      21              // 21 = PMSM FOC - Current Mode
#define MODE22      22              // 22 = PMSM FOC - Sensorless Mode

//--------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------
#define INDMotorSpec   INDM_22k0// IND Motor Selection - INDM_03k7, INDM_22k0
#define PMSMotorSpec   PMSM_22k8    // PMS Motor Selection - PMSM_05k8, PMSM_22k8

#define INDM_03k7   10          // Lab m/c
#define INDM_22k0   11
#define PMSM_05k8   20          // Lab m/c
#define PMSM_22k8   21
//--------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------------------------

// Range Definitions for User Data
#define Swfreq_min  01              // Min value for Switching Frequency
#define Swfreq_max  06              // Max value for Switching Frequency
#define Clock_freq 120
#define Sampling_Time 83.333
#define Sampling_Frequency 1/Sampling_Time

#define DBand_min   01              // Min value for Dead band
#define DBand_max   50              // Max value for Dead band

#define Vrated_min  323             // Min value for Rated Voltage, 323V
#define Vrated_max  528             // Max value for Rated Voltage, 528V

#define PowFac_min  0.7             // Min value for Power Factor, 0.7

#define Frated_min  50              // Min value for Rated Frequency
#define Frated_max  500             // Max value for Rated Frequency

#define Poles_min   02              // Min value for No of Poles
#define Poles_max   12              // Max value for No of Poles

#define ModInd_min  0.8             // Min value for Modulation Index
#define ModInd_max  1.3             // Max value for Modulation Index

#define RmpTm_min   10              // Min value for Ramp time
#define RmpTm_max   120             // Max value for Ramp time

#define Hsk_max     60//95             // Max allowed Heat Sink temperature
#define Enc_max     50              // Max allowed Enclosure temperature
#define Pt_max      80              // Max allowed Pt100 temperature

#define InpPhs_bal  25              // Max allowed % Input Phase Imbalance
#define OutPhs_bal  35              // Max allowed % Output Phase Imbalance

#define MotSpd_diff 10              // Max allowed % Speed Difference b/n Reference & Actual
#define MotSpd_max  150             // Max allowed % Speed Actual
#define MotTmp_max  100             // Max value of Motor Temperature

#define Vdc_min     500             // Min value for DC Bus voltage, 457V
#define Vdc_BrkRel  750             // Release Dynamic Brake at 750V
#define Vdc_BrkAct  780             // Activate Dynamic Brake at 780V
#define Vdc_max     780           // Max value for DC Bus voltage, 800V


// Define the Constants used in the controller calculations
#define PI          3.14159265358979
#define PIx2        6.28318530717958
#define PIby3       1.04719755119660    // PI/3 for 60 degree
#define Sqrt3by2    0.86602540378444
#define Sqrt3       1.73205080756888
#define Sqrt3_1     0.57735026918963
#define Sqrt2       1.41421356237309
#define Sqrt2_1     0.70710678118655
#define Oneby3      0.33333333333333

#define PWMUtilFac  Sqrt3_1         // PWM Utilisation Factor

#define DLOG_SIZE   200+2           // Datalog Size for each Channel, Can view HALF of DLOG_SIZE correctly in Graph (Sampling Theorem)


//############################################################################################################################################
// CRITICAL CALCULATIONS FROM HARDWARE CIRCUITRY - Required for proper functioning of Drive Controls
//############################################################################################################################################

#define ExtADC_R1       100            // Attenuation Res1 (kOhm)
#define ExtADC_R2       100              // Attenuation Res2 (kOhm)
#define ExtADC_Att      1//ExtADC_R2/(ExtADC_R1+ExtADC_R2)
                                        // Ext ADC Ch Attenuation

// INT ADC Specification
//-------------------------------------------------------------------------------------
#define IntADC_Vmax     3.0  /*Const*/  // Int ADC max i/p Voltage, Check Excel Sheet
#define IntDigValmax    4096.0 /*Const*/  // 12 bit ADC max Digital value (+-)
//-------------------------------------------------------------------------------------



// Current Scaling Definitions - EXT ADC
//-------------------------------------------------------------------------------------
#define Curr_Peak                   56.56              // LEM Current (A, rms), Check Excel Sheet
#define R_Shunt                    0.0003// LEM Burden Resistance (Ohm), Check Ckt & Excel Sheet
#define Shunt_Voltage              Curr_Peak*R_Shunt
#define Gain_Stage                 41
#define Curr_Differential          Shunt_Voltage*Gain_Stage
#define RF_DiffCurr                10.0
#define R1_DiffCurr                10.2
#define Curr_Diff_output           Curr_Differential*RF_DiffCurr/R1_DiffCurr
#define offset_Voltage             1.6 // Actual 1.65 as there is a offset of 117mV Offset made to 1.545..
#define Curr_Diff_offset_out       Curr_Diff_output + offset_Voltage
#define offset_voltage_gain        offset_Voltage
#define Actual_Curr_Value          Curr_Diff_offset_out  - offset_voltage_gain
#define offset_Counts              (offset_Voltage)*(IntDigValmax)/ (IntADC_Vmax)//(IntDigValmax*Curr_Diff_output )/ (IntADC_Vmax)
#define Actual_DC_Curr_Value       Curr_Diff_output
#define Enc_offset_counts          (0.5*4095)/3.0




//-------------------------------------------------------------------------------------
//#define LEM_Scal        2000 /*Const*/  // LEM Scaling Ratio, Check Excel Sheet - LA 100-TP

//#define LEM_ADC         LEM_Irms * Sqrt2 * LEM_Brd_R / LEM_Scal * ExtADC_Att
                                        // LEM max o/p Voltage at ADC i/p, Check Excel Sheet
//#define DigVal_cur      LEM_ADC / ExtADC_Vmax * ExtDigValmax
                                        // ADC Digital result, Check Excel Sheet


// DC Voltage Scaling Definitions - INT ADC for Redesigned Unit
//-------------------------------------------------------------------------------------
#define VdcR1           6000.00       // ACtually 6Mohm to compenstae for 25mV for calculations 6.3 is made      // V divider Res1 @ DC Bus (kOhm), Check Ckt & Excel Sheet
#define VdcR2           10.0             // V divider Res2 @ DC Bus (kOhm), Check Ckt & Excel Sheet
//-------------------------------------------------------------------------------------
#define Vdc_Nom         565.0  /*Const*/   // ADC max i/p Voltage, Check Excel Sheet - ISO124

#define VdcMeas       Vdc_Nom*VdcR2 / (VdcR1+VdcR2) //Vdc_Nom  /  (VdcR1+VdcR2)
                                        // Measurable max DC Voltage
//-------------------------------------------------------------------------------------
#define VdcOpAmpR1      9.4              // V divider Res1 @ Op Amp (kOhm), Check Ckt & Excel Sheet
#define VdcOpAmpR2      15.0            // V divider Res2 @ Op Amp (kOhm), Check Ckt & Excel Sheet
//-------------------------------------------------------------------------------------
#define Vdc_ADC         VdcMeas * VdcOpAmpR2 / VdcOpAmpR1
                                        // Vdc max o/p Voltage @ ADC i/p, Check Excel Sheet
#define DigVal_vlt       Vdc_ADC* IntDigValmax/IntADC_Vmax
                                        // ADC Digital result, Check Excel Sheet

#define VoltageSensing     6000.00       // ACtually 6Mohm to compenstae for 25mV for calculations 6.3 is made      // V divider Res1 @ DC Bus (kOhm), Check Ckt & Excel Sheet
#define VoltageSensingR2    10.0

#define Voltage_Nom         600.0
#define VoltageMeas       Voltage_Nom*VoltageSensingR2 / (VoltageSensing+VoltageSensingR2)

#define VoltageOpamp_Rf      7.5              // V divider Res1 @ Op Amp (kOhm), Check Ckt & Excel Sheet
#define VoltageOpamp_R1      10.2            // V divider Res2 @ Op Amp (kOhm), Check Ckt & Excel Sheet
//-------------------------------------------------------------------------------------
#define Voltage_ADC         VoltageMeas * VoltageOpamp_Rf / VoltageOpamp_R1


     // ACtually 6Mohm to compenstae for 25mV for calculations 6.3 is made      // V divider Res1 @ DC Bus (kOhm), Check Ckt & Excel Sheet
#define Grid_Voltage_Nom         415

#define VoltageSensingR2_grid    2.0


#define VoltageMeas_Grid      Grid_Voltage_Nom*VoltageSensingR2_grid / (VoltageSensing+VoltageSensingR2_grid)

#define VoltageOpamp_Rf_Grid      7.5              // V divider Res1 @ Op Amp (kOhm), Check Ckt & Excel Sheet
#define VoltageOpamp_R1_Grid      10.2            // V divider Res2 @ Op Amp (kOhm), Check Ckt & Excel Sheet

#define AMC_1300_Gain             8.1
//-------------------------------------------------------------------------------------
#define Voltage_ADC_Grid         VoltageMeas_Grid * VoltageOpamp_Rf_Grid * AMC_1300_Gain / VoltageOpamp_R1_Grid




#define ET_Rdiv         ET_Rst/ET_Rntc
#define ET_ADC          2.5             // After attenuation (V)
                                        // 2.5V max o/p Voltage @ ADC i/p, Check Excel Sheet
#define DigVal_ET       IntDigValmax / IntADC_Vmax
                                         // ADC Digital result, Check Excel Sheet


// Heat Sink Temp Scaling Definitions - INT ADC
//-------------------------------------------------------------------------------------
#define Beta_HTntc      3411            // Beta value of NTC (25/80) from IGBT datasheet
#define HT_Rst          15             // Rst (kOhm)
#define HT_Rntc         5               // Rntc @ 25 degC (kOhm)
//-------------------------------------------------------------------------------------
#define HT_OpAmpR1      15             // V divider Res1 @ Op Amp (kOhm), Check Ckt & Excel Sheet
#define HT_OpAmpRf      9.4             // V divider Res2 @ Op Amp (kOhm), Check Ckt & Excel Sheet
//-------------------------------------------------------------------------------------
#define HT_Rdiv         HT_Rntc/HT_Rst* HT_OpAmpRf/ (HT_OpAmpR1)
#define HT_ADC          1.99          // After attenuation (V)
                                        // 2.5V max o/p Voltage @ ADC i/p, Check Excel Sheet
#define DigVal_HT       (IntADC_Vmax * HT_OpAmpRf) / (HT_OpAmpR1 * IntDigValmax)
                                        // ADC Digital result, Check Excel Sheet

#define Pt100_Vs        3.3             // Supply voltage for Pt100 circuit (V)
#define Pt100_Ref       1.2             // Reference Resistance of Pt100 in Circuit (kOhm)
#define Pt100_max       2.0             // Max Resistance of Pt100 (kOhm)
#define Pt100_Alpha     0.003729        // Thermal Coefficient Alpha
#define Pt100_Alp_1     1/Pt100_Alpha   // Inverse of Alpha
//-------------------------------------------------------------------------------------
#define Pt100_Vptr      Pt100_Vs * Pt100_max / (Pt100_Ref+Pt100_max)
                                        // Pt100 max o/p Voltage @ Opamp i/p, Check Excel Sheet
//-------------------------------------------------------------------------------------
#define Pt100_Ra        1.24            // Opamp Gain Res A (kOhm)
#define Pt100_Rb        2.49            // Opamp Gain Res B (kOhm)
//-------------------------------------------------------------------------------------
#define Pt100_ADC       Pt100_Vptr * (1+Pt100_Ra/Pt100_Rb)
                                        // SPD max o/p Voltage @ ADC i/p, Check Excel Sheet
#define DigVal_Pt100    Pt100_ADC / IntADC_Vmax * IntDigValmax
                                        // ADC Digital result, Check Excel Sheet

// Speed Reference Scaling Definitions - INT ADC
//-------------------------------------------------------------------------------------
// Pressure i/p from Compressor unit serves as Speed Ref for the Motor Control, to be retransmitted in DAC
#define SPD_Vsp         2.0             // SPD max o/p Voltage @ Opamp i/p, Check Excel Sheet
#define SPD_Rpm         1470            // Speed equivalent of SPD_Vsp (rpm)
//-------------------------------------------------------------------------------------
#define SPD_Ra          1.24            // Opamp Gain Res A (kOhm)
#define SPD_Rb          2.49            // Opamp Gain Res B (kOhm)
//-------------------------------------------------------------------------------------
#define SPD_ADC         SPD_Vsp * (1+SPD_Ra/SPD_Rb) * ExtADC_Att
                                        // SPD max o/p Voltage @ ADC i/p, Check Excel Sheet
#define DigVal_SP       SPD_ADC / IntADC_Vmax  * IntDigValmax
                                        // ADC Digital result, Check Excel Sheet


//############################################################################################################################################


//--------------------------------------------------------------------------------------------------------------------------------------------
//#define EXT_ADC_Addr        *(volatile unsigned int*)0x4000 // External ADC Address
#define EXT_DAC_Op1_Addr    *(volatile unsigned int*)0x4001 // External DAC Output 1 Address
#define EXT_DAC_Op2_Addr    *(volatile unsigned int*)0x4005 // External DAC Output 2 Address
#define Latch_Addr          *(volatile unsigned int*)0x4002 // External Latch Address
#define SRAMBaseAdddr 0x200000
#define SRAMStartAddr 0x200000
//-------------------------------------------------------------------------------------------------------------------------------------------
#define SUCCESS             1
#define FAILED              0

#define Mode_TX             1
#define Mode_RX             0

#define OnlineStatusUpate           0xB7
#define RxControllerParamDSPtoGUI   0xB8
#define SendControllerParamGUItoDSP 0xB9
#define TrajTimeUpdate              0xC8
#define DataSampling                0xC9
#define BulkDataTransfer            0xCA
//-------------------------------------------------------------------------------------------------------------------------------------------
#define RELAY_1_Ctrl        GpioDataRegs.GPADAT.bit.GPIO14  // Relay control Port
#define RELAY_2_Ctrl        GpioDataRegs.GPADAT.bit.GPIO15  // Relay control Port
#define RELAY_PTC_Ctrl      GpioDataRegs.GPADAT.bit.GPIO30  // PTC Relay control Port
#define DYN_BRK_Ctrl        GpioDataRegs.GPADAT.bit.GPIO13 //GpioDataRegs.GPADAT.bit.GPIO26  // Dynamic Brake Control Port, Pin swapped in new design

#define PULSE_BLK_INV       GpioDataRegs.GPADAT.bit.GPIO12  // Inverter Fault
#define PULSE_BLK_AFE       GpioDataRegs.GPADAT.bit.GPIO13  // Active Front End Fault
 
#define Latch_Reset_Ctrl    GpioDataRegs.GPADAT.bit.GPIO9 //GpioDataRegs.GPBDAT.bit.GPIO61  // Reset the Fault Latch Cntrol Port
#define DAC_Write_En        GpioDataRegs.GPBDAT.bit.GPIO38  // Write Enable of External DAC
#define EPROM_Write_Protect GpioDataRegs.GPADAT.bit.GPIO25

#define DO_OP1              GpioDataRegs.GPADAT.bit.GPIO26//GpioDataRegs.GPBDAT.bit.GPIO59  // Digital Output Port 1
#define DO_OP2              GpioDataRegs.GPADAT.bit.GPIO27//GpioDataRegs.GPBDAT.bit.GPIO60  // Digital Output Port 2
#define DO_OP3              GpioDataRegs.GPBDAT.bit.GPIO59 //GpioDataRegs.GPADAT.bit.GPIO31  // Digital Output Port 3
#define DO_OP4              GpioDataRegs.GPBDAT.bit.GPIO60 // DIgital output 4 is given in redesign.

#define SCIA_RxTx_En        GpioDataRegs.GPADAT.bit.GPIO20  // Rx Tx Enable of SCI A
#define SCIB_RxTx_En        GpioDataRegs.GPADAT.bit.GPIO21  // Rx Tx Enable of SCI B
#define SCIC_RxTx_En        GpioDataRegs.GPADAT.bit.GPIO24  // Rx Tx Enable of SCI C

//#define EXT_ADC_Reset       GpioDataRegs.GPBDAT.bit.GPIO35  // Reset Control Port of External ADC
//#define EXT_ADC_Start       GpioDataRegs.GPADAT.bit.GPIO30  // Start Control Port of External ADC
//#define EXT_ADC_Busy        GpioDataRegs.GPBDAT.bit.GPIO34  // Busy Control Port of External ADC

#define DI_IP1              GpioDataRegs.GPBDAT.bit.GPIO48  // Digital Input Port 1
#define DI_IP2              GpioDataRegs.GPBDAT.bit.GPIO49  // Digital Input Port 2
#define DI_IP3              GpioDataRegs.GPBDAT.bit.GPIO50  // Digital Input Port 3
#define DI_IP4              GpioDataRegs.GPBDAT.bit.GPIO51  // Digital Input Port 4
#define DI_IP5              GpioDataRegs.GPBDAT.bit.GPIO52  // Digital Input Port 5
#define DI_IP6              GpioDataRegs.GPBDAT.bit.GPIO53  // Digital Input Port 6
#define DI_IP7              GpioDataRegs.GPBDAT.bit.GPIO54  // Digital Input Port 7
#define DI_IP8              GpioDataRegs.GPBDAT.bit.GPIO55  // Digital Input Port 8
#define DI_IP9              GpioDataRegs.GPBDAT.bit.GPIO56  // Digital Input Port 9
#define DI_IP10             GpioDataRegs.GPBDAT.bit.GPIO57  // Digital Input Port 10

#define SystemOn            0   // Inversion in Hardware Circuit
#define SystemOff           1   // Inversion in Hardware Circuit

#define DI_FailStop        GpioDataRegs.GPADAT.bit.GPIO6  // DI Fail stop is included in the new design
#define DI_GNDFault        GpioDataRegs.GPADAT.bit.GPIO7  // DI GND fault is included in the new design
#define DI_OvrCurr         GpioDataRegs.GPADAT.bit.GPIO8  // Digital Input overcurrent in the new design
#define RFI_Filter         GpioDataRegs.GPADAT.bit.GPIO31 // RFI Filter to be made ON
//--------------------------------------------------------------------------------------------------------------------------
// Structure variables declared here...
//--------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------
// Structure for Flags for System Status...
//--------------------------------------------------------------------------------------------------------------------------
union SystemFlags
{
    unsigned long int all;
    struct StatusFlags
    {
    //--------------------------------------------------------------------------------------------------------------------------
    // Structure for Status Flags
    //--------------------------------------------------------------------------------------------------------------------------
        Uint32  FaultFlag:32;           // System Status                : System Fault Status
    //    Uint16  SpareBits1:16;

        Uint16  StartFlag:1;            // System Status                : System On Off Status
        Uint16  IntrptFlag:1;           // System Status                : Interrupt Status
        Uint16  RampMode:1;             // Ramp Control                 : 0 = RampDown, 1 = RampUp
        Uint16  EpsThetFlag:1;          // Epsilon to Theta Change over : 0 = Epsilon,  1 = Theta
        Uint16  ThetEpsFlag:1;          // Theta to Epsilon Change over : 0 = Theta,    1 = Epsilon
        Uint16  DynBrkFlag:1;           // Dynamic Brake Active Status  : 0 = Disabled, 1 = Enabled
        Uint16  UpdateVars:1;           // VFD Variables update status  : 0 = Updated,  1 = To be updated
        Uint16  ReInitParm:1;           // Parmeters to be Reinited     : 0 = Reinited, 1 = To be Reinited
        Uint16  SpareBits2:8;
        Uint16  SpareBits3:16;
    }bits;

};


//--------------------------------------------------------------------------------------------------------------------------
// Structure for Internal ADC Data...
//--------------------------------------------------------------------------------------------------------------------------
struct IntAdcData
{
    // ADC Scaling Factors
    _iq EncTem_Scal;
    _iq HskTem_Scal;

    _iq VdcVlt_Scal;
    _iq Curr_Scal;
    _iq Volt_Scal;
    _iq Volt_grid_Scal;
    _iq SpdRef_Scal;
    _iq Idc_Curr_Scal;

    _iq Pt100_Scal;

    // Internal ADC
    int     Isa;
    int     Isb;
    int     Isc;

    int     Vmot_Rph;   // Mon_Enc
    int     Vmot_Yph;  //  Mon_HST
    int     Vmot_Bph;
    int     Vmot_RYph;
    int     Vmot_YBph;
    int     Vmot_BRph;

    int     Mon_Vdc;
    int     Idc_Sns;
    int     offset_counts_Isa;
    int     offset_counts_Isb;
    int     offset_counts_Isc;
    int     offset_counts_Idc;
    int     offset_counts_Vgry;
    int     offset_counts_Vgyb;

    int     Vgry;
    int     Vgyb;
    int     SpdRef;
    int     Mon_Pt100;
    int     Tach_HF;
    int     Tach_EF;
    int     Mon_Enc;
    int     Mon_HST;
    int     Isa_aver;
    int     Isb_aver;
    int     Isc_aver;
};

//--------------------------------------------------------------------------------------------------------------------------
// Structure for User Data...
//--------------------------------------------------------------------------------------------------------------------------
struct UserData
{
    // Motor Nameplate
    float32 Prated;         // Rated Power      (kW)
    float32 Vrated;         // Rated Voltage    (V)
    float32 Irated;         // Rated Current    (A)
    float32 Frated;         // Rated Frequency  (Hz)

    // Motor Parameters
    float32 Rs;             // INDM Stator Resistance
    float32 Rr;             // INDM Rotor Resistance
    float32 Ls;             // INDM Stator Inductance
    float32 Lr;             // INDM Rotor inductance
    float32 Lm;             // Magnetising Inductance

    float32 Rdc;            // PMSM Stator Resistance
    float32 Ld;             // PMSM Stator Inductance
    float32 Lq;             // PMSM Rotor inductance
    float32 Ebc;            // PMSM Back EMF Constant

    int16 Nrated;           // Rated Speed (rpm)
    int16 Spare_1;

    int16 Data_Srce;        // 00 => From Code
                            // 01 => From GUI
    int16 Spare_2;

    int16 Motor_Type;       // 01 => INDM, Induction Motor
                            // 02 => PMSM, Permanent Magnet Synchronous Motor
    int16 Spare_3;

    int16 Control_Mode;     // 11 => INDM V/F - Open Loop
                            // 12 => INDM V/F - Closed Loop
                            // 13 => INDM FOC - Sensorless Mode
                            // 21 => PMSM FOC - Current Mode
                            // 22 => PMSM FOC - Sensorless Mode
    int16 Spare_4;

    int16 Mot_Rot;          // Direction of Motor Rotation
    int16 Spare_5;


// Motor Run Settings
    float32 Mod_Ind;        // Max Modulation Index
    int16   Sw_Freq;        // Switching Frequency (kHz)
    int16 Spare_6;

    int16 TorqStrt;         // Starting Flux Reference for Current Control Mode (pu)
    int16 Spare_7;
//    int16 TorqStrt;         // Starting Torq Reference for Current Control Mode (pu)

    int16 ReInitParm;       //Parmeters to be Reinitialised
                            // 0 = Reinitialised
                            // 1 = To be Reinitialised"
    int16 Spare_8;

    int16 Refip_Srce;       // 00 => From Code
                            // 01 => From GUI
                            // 02 => From Analog i/p
    int16 Spare_9;

    float32 Op_Freq;        // Operating Frequency, Speed Ref (Hz)
    float32 Accel_Rate;     // Ramp Acceleration (Hz/s)

    int16 Run;              // Run system from GUI
    int16 Spare_10;

    _iq SpeedKp;            // Proportional Gain of Speed PI Controller
    _iq SpeedKi;            // Integral Gain of Speed PI Controller

    _iq CurrKp;             // Proportional Gain of Speed PI Controller
    _iq CurrKi;             // Integral Gain of Current PI Controller

    int16 Energy_Efficient; // Energy EFficient Mode
    int16 Spare_11;

    float32 AnaVoltMin;           // Minimum Analog Voltage
    float32 AnaVoltMax;           // Maximum Analog Voltage

    int16 SpdAnaMin;      // Speed Analog Minimum
    int16 Spare_12;

    int16 SpdAnaMax;      // Speed Analog Maximum
    int16 Spare_13;

};

//--------------------------------------------------------------------------------------------------------------------------
// Structure for System Data...
//--------------------------------------------------------------------------------------------------------------------------
struct SystemData
{
//--------------------------------------------------------------------------------------------
// Measured from Analog i/p
    _iq     Isa;            // Rph Current, measured from ADC (pu)
    _iq     Isb;            // Yph Current, measured from ADC (pu)
    _iq     Isc;            // Bph Current, measured from ADC (pu)

    _iq     Vgry;           // RY LL Voltage, from ADC (pu)
    _iq     Vgyb;           // YB LL Voltage, from ADC (pu)
    _iq     Vgrb;
    _iq     SpdRef;         // Speed Reference from ADC (pu)
    _iq     Vmotr;
    _iq     Vmoty;
    _iq     Vmotb;

    _iq     Vdc;            // DC Bus voltage measured from ADC (pu)
    _iq15   EncTem;         // Enclosure Temperature
    _iq15   HskTem;         // Heat Sink Temperature


    _iq15   Pt100;          // Pt100 Temperature

    _iq     Isd;            // Transformed from Isalpha, Isbeta (pu)
    _iq     Isq;
    _iq     Imr;

    _iq     ImrRef;
    _iq     FluxRef;        // Reference for Flux Reference (pu)
    _iq     TorqRef;        // Reference for Torq Reference (pu) for PMSM starting
    _iq     Torqlower;      // Lower Limit of Isq for Change over
    _iq     IsdRef;         //
    _iq     IsqRef;         //

    _iq     IsdLim;         // Isd Max Limit (pu)
    _iq     IsqLim;         // Isq Max Limit (pu)

    _iq     SpeedRef;       // Reference for Speed Reference (pu)
    _iq     SpeedRmp;       // Ramping Speed Reference (pu)
    _iq     SpeedAct;       // Actual Speed (pu)

    _iq     VsdRef;         // Reference for Inverter (pu)
    _iq     VsqRef;
    _iq     VsdLim;         // Vsd Max Limit (pu)
    _iq     VsqLim;         // Vsq Max Limit (pu)
    int16   Dead_Band;      // VFD PWM Deadband
    int16   Spare_11;

//--------------------------------------------------------------------------------------------
    int16   Con_Mode;       // Control_Mode
    int16   Mot_Type;       // Motor type

    int16   DynBrkFreq;     // Frequency of Dynamic Braking PWM
    _iq     DynBrkDuty;     // Duty cycle of Dynamic Braking PWM

    _iq     Isa_abs;        // Absolute value of Isa
    _iq     Isb_abs;
    _iq     Isc_abs;



      _iq     Idc;         // Dig 15V measured from ADC (pu)



    _iq     Renc;           // NTC Resistance for Ambient temp from ADC (pu)
    _iq     Rhsk;           // NTC Resistance for Heat Sink temp from ADC (pu)
    _iq     Rpt100;         // Pt100 Resistance from ADC (pu)




    _iq     IsAlpha;        // Transformed from Iabc (pu)
    _iq     IsBeta;



    float32 VdqMax;         // Actual Maximum Value for Vd & Vq

    float32 IsdMax;         // Actual Maximum Value for Isd
    float32 IsdNorm;
    float32 IsqMax;
    float32 IsqNorm;

    float32 Mod_Ind;        // Modulation Index
    int16   Mot_Rot;        // Direction of Motor Rotation
    float32 VdcRef;         // Reference DC Bus Voltage
    float32 InvGain_1;      // Inverse of Inverter Gain

    _iq     VrRef;          // Transformed from Vsalpha, Vsbeta (pu)
    _iq     VyRef;
    _iq     VbRef;
    int16   VrRef_;         // Reference for PWM Generation (pu)
    int16   VyRef_;
    int16   VbRef_;

    float32 VsdMax;         //
    float32 VsqMax;

    _iq     VsdRefTmp;      // Temp Reference for Inverter (pu)
    _iq     VsqRefTmp;

    _iq     VsAlphaRef;
    _iq     VsBetaRef;

    _iq     SineEpsilon;    // Generated Sine angle for V/F operation (pu)
    _iq     CosEpsilon;     // Generated Cosine angle for V/F operation (pu)
    _iq     SineTheta;      // Estimated Sine angle for Sensorless FOC operation (pu)
    _iq     CosTheta;       // Estimated Cosine angle for Sensorless FOC operation (pu)

    int16   Sys_Freq;       // System Clock Frequency
    int16   Sw_Freq;        // VFD PWM Switching Frequency

    _iq     Ts;             // System Sampling Time
    _iq15   Ts_1;           // Inverse of System Sampling Time
    _iq     Tsdc;
    _iq20   FbwIs;          // Current Controller Bandwidth
    _iq     TbwIs;          // Current Controller Time Constant
    _iq   FbwWm;          // Speed Controller Bandwidth
    _iq     TbwWm;          // Speed Controller Time Constant
    _iq20   FbwImr;         // Imr Bandwidth
    _iq     TbwImr;         // Imr Controller Time Constant
    _iq     K1;             // Constants for Imr Flux Component
    _iq     K2;             // Constant for Isq Component for Power Warping



    _iq     FluxEstFc;      // LowPass Filter Corner Frequency for Flux Estimation
    _iq     fop      ;      // Operating frequency
    _iq     SpeedEstFc;     // LowPass Filter Corner Frequency for Speed Estimation
    _iq     DAxisFc ;
    _iq     GridFc ;

    _iq20   VdcFc;          // LowPass Filter Corner Frequency for Vds Estimation
    _iq     VdcTmp;
    _iq15   VdcTc;          // Parameter: Low Pass Filter Time constant
    _iq     VdcFilt;
    _iq     VdcFiltOld;
    _iq     VdcKp;          // Proportional Controller for DC Voltage
    _iq     VsqRefDelta;
    int16     Energy_Efficient;


    _iq20 HskTemFc;          // LowPass Filter Corner Frequency for Vds Estimation
    _iq   RhskTmp;
    _iq15 HskTemTc;          // Parameter: Low Pass Filter Time constant
    _iq   RhskFilt;
    _iq   RhskFiltOld;

    _iq     AnaMin;
    _iq     AnaMax;

    _iq     SpdMin;
    _iq     SpdMax;
    _iq     SpdAnalConstant;

    _iq     DAxisTmp;
    _iq15   DAxisTc;          // Parameter: Low Pass Filter Time constant
    _iq     DAxisFilt;
    _iq     DAxisFiltOld;


    _iq    GridTmp;
   _iq15   GridTc;          // Parameter: Low Pass Filter Time constant
   _iq     GridFilt;
   _iq     GridFiltOld;


    int ADC_Flag ;
    int32  offset_Isa ;
    int32  offset_Isb;
    int32  offset_Isc;
    int32  offset_speed ;
    int32  offset_Enclosure_Temp;
    int32  average_Vmot_r;
    int32  average_Vmot_y;
    int32  average_Vmot_b;
    _iq Speed_Error ;

    _iq Resultant_Current;
    _iq Resultant_Current_abs;

    _iq HeatSink_Fan;
    _iq SMPS_Fan;
};

//--------------------------------------------------------------------------------------------------------------------------
// Structure for Motor Data...
//--------------------------------------------------------------------------------------------------------------------------
struct MotorData
{
    float32 Rs;             // Stator Resistance
    float32 Rr;             // Rotor Resistance
    float32 Ls;             // Stator Inductance
    float32 Lr;             // Rotor inductance

    float32 Ld;             // Stator Inductance
    float32 Lq;             // Rotor inductance
    float32 Lmdf;           // Flux Linkage

    float32 Lm;             // Magnetising Inductance
    float32 Lm_1;           // Inverse of Base Inductance

    float32 Kt;             // Torque Constant
    float32 Jm;             // Motor Inertia

    float32 Vb;             // Base Voltage
    float32 Vb_1;           // Inverse of Base Voltage
    float32 Ib;             // Base Current
    float32 Ib_1;           // Inverse of Base Current
    float32 Zb;             // Base Impedance
    float32 Zb_1;           // Inverse of Base Impedance
    float32 Wb;             // Base Angular Speed
    float32 Wb_1;           // Inverse of Angular Speed
    float32 Lb;             // Base Inductance
    float32 Lb_1;           // Inverse of Base Inductance
    float32 Sigma;          // Leakage Inductance
    float32 SigmaR;         // Rotor Leakage factor
    float32 SigmaS;         // Stator Leakage factor

    float32 Nsynch;         // Synchronous Speed
    float32 Nrated;         // Rated Speed
    float32 Trated;         // Rated Torque
    int16   Poles;          // No of Poles

    _iq Rspu;               // Stator Resistance (pu)
    _iq Rrpu;               // Rotor Resistance (pu)
    _iq Lspu;               // Stator Inductance (pu)
    _iq Lrpu;               // Rotor inductance (pu)
    _iq Ldpu;               // Daxis Inductance (pu)
    _iq Lqpu;               // Qaxis inductance (pu)
    _iq Lmpu;               // Magnetising Inductance (pu)
    _iq Lmdfpu;             // Flux Linkage (pu)

    _iq Tst;                // Stator Time Constant
    _iq Trt;                // Rotor Time Constant
    _iq Trt_Imr;              // Imr Constant
    _iq Trt_Imr_fbk;        // Imr_Feedback constant
};

//--------------------------------------------------------------------------------------------------------------------------
// Structure for Ramping Function
//-------------------------------------------------------------------------------------------------------------------------
struct RampGen
{
    _iq     TargVal;        // Input    : Target input (pu)
     Uint32 Delay;          // Parameter: Delay rate (Q0)
     _iq    LowLim;         // Parameter: Minimum limit (pu)
     _iq    HighLim;        // Parameter: Maximum limit (pu)
     Uint32 DelayCnt;       // Variable : Incremental delay (Q0)
     _iq    Tmp;            // Variable : Temp variable
     _iq    SetValue;       // Output   : Target output (pu)
     Uint16 EqualFlag;      // Output   : Status output (Q0)
};

//--------------------------------------------------------------------------------------------------------------------------
// Structure for Ramp Controller
//-------------------------------------------------------------------------------------------------------------------------
struct RampCntrl
{
     _iq    Freq;           // Input    : Ramp frequency (pu)
     _iq    Angle;          // Variable : Step angle (pu)
     _iq    StepAng;        // Parameter: Maximum step angle (pu)
     _iq    RampOut;        // Output   : Ramp signal (pu)
};

//--------------------------------------------------------------------------------------------------------------------------
// Structure for V/F Profile
//-------------------------------------------------------------------------------------------------------------------------
struct VbyFprofile
{
    _iq  Freq;              // Input    : Input Frequency (pu)
    _iq  FreqMin;           // Parameter: Low Frequency (pu)
    _iq  FreqRated;         // Parameter: High Frequency at rated voltage (pu)
    _iq  FreqMax;           // Parameter: Maximum Frequency (pu)
    _iq  VoltMin;           // Parameter: Voltage at low Frequency range (pu)
    _iq  VoltRated;         // Parameter: Rated voltage (pu)
    _iq  VfSlope;           // Variable : Calculated (Voltage/Frequency) Slope
    _iq  AbsFreq;           // Variable :
    _iq  VoltOut;           // Output   : Output voltage (pu)
    _iq  FreqOut;           // Output   : Output Frequency (pu)
};

//--------------------------------------------------------------------------------------------------------------------------
// Structure for Flux & Speed Estimator
//-------------------------------------------------------------------------------------------------------------------------
struct FluxSpeedEstim
{
    // Motor time constants

    _iq EsAlpha;            // Variable : Back EMF in Rotating frame (pu)
    _iq EsBeta;             // Variable : "
    _iq PsisAlpha;          // Variable : Stator Flux in Rotating frame (pu)
    _iq PsisBeta;           // Variable : "
    _iq PsisAlpha_comp ;    // Gain and Phase Compensated Stator flux
    _iq PsisBeta_comp  ;    // Gain and Phase Compensated in Stator flux
    _iq Psis_constant;
    _iq PsisAlphaOld;       // Variable : [k-1] term of Stator Flux (pu)
    _iq PsisBetaOld;        // Variable : "
    _iq21 PsisAlphatemp;    // for mutiplication with wbase
    _iq21 PsisBetatemp;     // for multiplication with wbase

    _iq FluxEstWc;          // Parameter: Low Pass Filter term
    _iq SpeedEstTc;         // Parameter: Low Pass Filter Time constant
    _iq VsAlpha;            // Input    : Vs Alpha (pu)
    _iq VsBeta;             // Input    : Vs Beta (pu)
    _iq IsAlpha;            // Input    : Is Alpha (pu)
    _iq IsBeta;             // Input    : Is Beta (pu)

    _iq Isd;                // Variable : Is in Stationary frame (pu)
    _iq Isq;                // Variable : "

    _iq PsirAlpha;          // Variable : Rotor Flux in Rotating frame (pu)
    _iq PsirBeta;           // Variable : "
    _iq PsirAlphatemp;    // temporary variable for wbase multiplication modification
    _iq PsirBetatemp;     // temporary variable for wbase multiplication modification



    _iq SineTheta;          // Variable : Calculated Sine of estimated Rotor Flux angle (pu)
    _iq CosTheta;           // Variable : Calculated Cosine of estimated Rotor Flux angle (pu)
    _iq SineThetaOld;       // Variable : [k-1] term (pu)
    _iq CosThetaOld;        // Variable : [k-1] term (pu)

    _iq WEst;               // Output   : Estimated Rotor Speed (pu)
    _iq WSyntemp;           // Variable :

    _iq WEstFilt;           // Output   : Filtered Est Rotor Speed (pu)
    _iq WEstFiltOld;        // Variable : [k-1} term (pu)

    _iq K3;                 // Parameter:
    _iq K4;                 // Parameter:
    _iq K5;                 // Variable Parameter
    _iq K;                  // Variable corner frequency constant
    _iq ThetaFlux;          // Output   : Estimated Rotor flux angle (pu)
    _iq ThetaFluxtemp;      // wbase modification
    _iq ThetaFluxAtD;       // Output   : Estimated Rotor flux angle (pu)
    _iq ThetaFluxAtQ;       // Output   : Estimated Rotor flux angle (pu)
    _iq WSlip;              // Variable : Calculated Rotor slip (pu)
    _iq WSyn;               // Variable : Calcualted Synchronous Speed (pu)

};

//--------------------------------------------------------------------------------------------------------------------------
// Structure for Rotor Position Estimator
//-------------------------------------------------------------------------------------------------------------------------
struct RotorPosEstim
{
    int InjCnt;             // Counter for Current Injection
    int RotPosFlag;         // Set when Rotor Position Estimation is completed
    int RotPosCnt;          // Counter for the Estimation Function execution
    int RotPosCntLim;       // Limit for the Estimation Function execution

    _iq Isd_Max;            // Isd Maximum for North Pole Detection (Initial Position Estimation)
    _iq Isq_Max;            // Isq Maximum for Pole Detection (Initial Position Estimation)  ??
};

//--------------------------------------------------------------------------------------------------------------------------
// Structure for PI Controller
//-------------------------------------------------------------------------------------------------------------------------
struct PIcontrol
{
    _iq  Ref;               // Input    : Reference set-point
    _iq  Fbk;               // Input    : Feedback
    _iq  Out;               // Output   : Controller output
    _iq  Kp;                // Parameter: Proportional loop gain
    _iq  Ki;                // Parameter: Integral gain
    _iq  Umax;              // Parameter: Upper saturation limit
    _iq  Umin;              // Parameter: Lower saturation limit
//----------------------------------------------------------------------------------------------
    _iq  KpInt;             // Variable : Intermediate Proportional gain
    _iq  KiInt;             // Variable : Intermediate Integral gain
    _iq  up;                // Data     : Proportional term
    _iq  ui;                // Data     : Integral term
    _iq  ut;                // Data     : Integral temporary
    _iq  u1;                // Data     : Error storage: u(k-1)
    _iq  v1;                // Data     : Pre-saturated controller output
    _iq  i1;                // Data     : Integrator storage: ui(k-1)
    _iq  w1;                // Data     : Saturation record: [u(k-1) - v(k-1)]

};

//--------------------------------------------------------------------------------------------------------------------------
// Structure for SVPWM Generator
//-------------------------------------------------------------------------------------------------------------------------
struct PWMgen
{
    // SVPWM for V/F Control
    _iq  VoltAmp;           // Input    : Reference gain voltage (pu)
    _iq  Offset;            // Input    : Reference offset voltage (pu)
    _iq  Freq;              // Input    : Reference frequency (pu)
    _iq  FreqMax;           // Parameter: Maximum step angle = 6*base_freq*T (pu)
    _iq  SecAngle;          // History  : Sector angle (pu)
    _iq  NewEntry;          // History  : Sine (angular) look-up pointer (pu)
    _iq  StepAngle;         // Variable
    _iq  EntryOld;          // Variable
    _iq  dx;                // Variable
    _iq  dy;                // Variable

    // SVPWM for FO Control

    _iq  Alpha;             // Input    : Reference alpha-axis phase voltage
    _iq  Beta;              // Input    : Reference beta-axis phase voltage
    _iq  tmp1;              // Variable : temp variable
    _iq  tmp2;              // Variable : temp variable
    _iq  tmp3;              // Variable : temp variable

    Uint16 VecSector;       // Variable : Space vector sector
    _iq  Ta;                // Output   : Reference phase-a switching function (pu)
    _iq  Tb;                // Output   : Reference phase-b switching function (pu)
    _iq  Tc;                // Output   : Reference phase-c switching function (pu)

    _iq VaRef;              // Output   : Reference phase-a for THIPWM
    _iq VbRef;              // Output   : Reference phase-b for THIPWM
    _iq VcRef;              // Output   : Reference phase-c for THIPWM

    Uint16 PeriodMax;       // Parameter: PWM Half-Period in CPU clock cycles (Q0)
    Uint16 HalfPerMax;      // Parameter: Half of PeriodMax                   (Q0)
    Uint16 Deadband;        // Parameter: PWM deadband in CPU clock cycles    (Q0)

};

//--------------------------------------------------------------------------------------------------------------------------
// Structure for Drive Limits...
//--------------------------------------------------------------------------------------------------------------------------
struct LimitsData
{
    _iq     ABC_OC_Limit;
    _iq     Ds_OC_Limit;
    _iq     Qs_OC_Limit;

    _iq     DC_OV_Limit;
    _iq     DC_UV_Limit;
    int16     DC_UV_counter;

    _iq     DCBrakeActLimit;
    _iq     DCBrakeRelLimit;


    _iq15   Hsk_OT_Limit;
    _iq   Enc_OT_Limit;

    _iq15     Mot_OT_Limit;
//--------------------------------------------------------------------------------------------

    Uint16  DB_Pulse_Cnt;
       int16   InpCyc_cnt;
       int16   Inp_phase_unbalance_cnt;
       int16   Heat_Sink_Fan_counter;
       int16   SMPS_Fan_counter;
       int16   Enclosure_Temp_counter;
       int16   Temparature_cnt ;
       int16   Speed_cnt ;

       int16   AC_Current_Counter_Isa ;
       int16   AC_Current_Counter_Isb ;
       int16   AC_Current_Counter_Isc ;
       int16   AC_Current_window;
       int16   AC_Zero_Crossing_Window;

       int16   OutCyc_cnt;
       _iq    Vsalphagrid;
       _iq    Vsbetagrid ;
       _iq    thetagrid ;
       _iq    Vdgrid ;
       _iq    Vgridmin  ;
       _iq    Vgridmax  ;
       unsigned int Grid_Voltage_Flag:1;
       _iq    Vtempgrid ;
       _iq    SineThetagrid;          // Variable : Calculated Sine of estimated Rotor Flux angle (pu)
       _iq    CosThetagrid;
       int16  zero_grid;
       _iq    Vmot_Top_Switch;
       _iq    Vmot_Bot_Switch;
       _iq    Vmot_Rph_Counter;
       _iq    Vmot_Yph_Counter;
       _iq    Vmot_Bph_Counter;
       _iq    Phase_A_Flag;


       _iq   Isab_mirror;
       _iq   Isbc_mirror;
       _iq   Isac_mirror;

      int16 A_Phase_Missing;
      int16 B_Phase_Missing;
      int16 C_Phase_Missing;

      int16 A_Phase_counter;
      int16 B_Phase_counter;
      int16 C_Phase_counter;

      int16 A_phase_temp;
      int16 B_phase_temp;
      int16 C_phase_temp;


      unsigned int A_Phase_Flag:1;
      unsigned int B_Phase_Flag:1;
      unsigned int C_Phase_Flag:1;
      unsigned int A_Phase_Debug:1;
      unsigned int freq_A_Init_Flag_zero:1;
      unsigned int freq_B_Init_Flag_zero:1;
      unsigned int freq_C_Init_Flag_zero:1;

      int16 RY_phase_temp;
      int16 RY_Phase_counter;
      int16 RY_Phase_Flag;

      _iq A_Phase_freq;
      _iq B_Phase_freq;
      _iq C_Phase_freq;

      int16 A_Phase_Freq_Miss;
      int16 B_Phase_Freq_Miss;
      int16 C_Phase_Freq_Miss;

      int16 Phase_sequence;
      int16 Phase_Sequence_Flag;

      int16 Unbalance_current;

      _iq Low_freq;
      _iq Hig_freq;

      unsigned int zero_cross:4;
      unsigned int zero_cross_flag:1;


};

struct MotorLimts
{
    int16 Motor_Low_Limit;  // Motor Lower Speed Limit
    int16 Motor_High_Limit; // Motor High Speed Limits
    int16 Motor_Torque_Limit; // Motor Torque Max Limits
    int16 Current_Limit;  // Motor Current Max Limits
    int16 Max_freq_Limit; // Motor Freq limits
};


struct UserWarnings
{
    int16 Timeoutdelay ;  // Time in seconds for delay out
    int16 TimeoutFucntion;// User_Configurable
    int16 MainsFailure;   // Decision on Power Failure
};
//
//--------------------------------------------------------------------------------------------------------------------------
// Structure for Modbus Communication
//--------------------------------------------------------------------------------------------------------------------------
struct ModbusMode
{
    Uint16 Mode;        // 0 = Mode_RX, Receive mode
                        // 1 = Mode_TX, Transmit mode

    Uint16 Status;      // 00 = Waiting for Request Frame

                        // 10 = Read Holding Reg Read, Response Frame Initialisation
                        // 11 = Read Holding Reg Read, Response Frame Processing
                        // 20 = Write Holding Reg Read, Response Frame Initialisation
                        // 21 = Write Holding Reg Read, Response Frame Processing

                        // 30 = Response Frame Byte Sent Successfully
                        // 40 = Response Frame Sent Successfully
                        // 80 = Response Frame Sending Failed

    Uint16 Index;       // Index of RxBuffer
};

//--------------------------------------------------------------------------------------------------------------------------
// Structure for DataLogging
//--------------------------------------------------------------------------------------------------------------------------
struct DataLog
{
    Uint32 ch0Sel;

    Uint32 ch1Sel;

    Uint32 ch2Sel;

    Uint32 ch3Sel;

    Uint32 Mem_Type;// 0 => RAM / SDRAM Write
                    // 1 => SDRAM Read

    Uint32 Mem_Mode;// 00 - Idle
                    // 10 - Write Initialisation
                    // 01 - Write
                    // 20 - Read Initialisation
                    // 02 - Read

    Uint32 Mem_Chns;// No of Channels to Wr/Rd (1 - 3)

    Uint32 Mem_Totl;// No of Samples for 1 Chns, 800  to 128000
                    // No of Samples for 2 Chns, 1200 to 128000
                    // No of Samples for 3 Chns, 1600 to 128000

    Uint32 Mem_Strt;// Start Write/Read from Specific Sample

    Uint32 Mem_Resl;// Rate of Writing Data (ms)

    Uint32 Mem_Cont;// Accessing Memory Location

};

//--------------------------------------------------------------------------------------------------------------------------
// Structure variables defined here...
//--------------------------------------------------------------------------------------------------------------------------
extern union SystemFlags VFD_Status;
extern struct IntAdcData VFD_IntAdcData;
extern struct UserData User_Data;
extern struct SystemData VFD_Data;
extern struct LimitsData VFD_Limits;
extern struct MotorLimts Motor_Limits;
extern struct UserWarnings User_Warnings;
extern struct MotorData MOT_Data;
extern struct RampGen MOT_RG;
extern struct RampGen MOT_RGI;
extern struct RampCntrl MOT_RC;
extern struct RampGen MOT_RGD;
extern struct RampGen MOT_RGQ;
extern struct VbyFprofile MOT_VF;
extern struct FluxSpeedEstim MOT_FSE;
extern struct RotorPosEstim  PMSM_RPE;
extern struct PIcontrol PI_Isd;
extern struct PIcontrol PI_Isq;
extern struct PIcontrol PI_Spd;
extern struct PIcontrol PI_Imr;
extern struct PWMgen MOT_SVM;
extern struct RampCntrl Dyn_Con;
extern struct ModbusMode VFD_ModbusMode;
extern struct DataLog Dat_log;
//-------------------------------------------------------------------------------------------------------------------------------------------
//Variable Declarations
//--------------------------------------------------------------------------------------------------------------------------------------------
extern Uint16 Debounce_Counter;
extern Uint32 SDMemTot, SDMemInt, SDMemCnt;
extern Uint16 SDDelCnt, DACDelCnt, SDCh, SDChs, SDBuffCnt;

//*********************************************************************************************************************
//Variable Declarations for MODBUS communication
//*********************************************************************************************************************/
extern unsigned char TxCount;
extern unsigned char RxCount;
extern Uint16 timingCount;
extern unsigned char tmpwritevalue1;
extern unsigned char tmpwritevalue2;
extern long tmpwritevalue;
extern long *StructAddr[7];
extern int32 *StructAddrint[8];
extern float32 *StructAddrfloat[7];

//*********************************************************************************************************************
//Variable Declarations for RAM
//*********************************************************************************************************************/
extern unsigned int volatile * AddrPtr ;
extern unsigned int volatile * SegmentStrtAddr;

//--------------------------------------------------------------------------------------------------------------------------------------------------
//Function Declarations
//--------------------------------------------------------------------------------------------------------------------------------------------------
void delay(unsigned int count);
void Initialise_DSP();
void InitSysCtrl();
void DisableDog();
void InitPll();
void InitPeripheralClocks();
void Initialise_Variables();
void Init_User_Data();
void Init_VFD_Data_Vars();
void Init_VFD_Data_Reset();
void Init_FreqRamp_Vars();
void Init_VsdRamp_Vars();
void Init_VsqRamp_Vars();
void Init_Motor_Params();
void Init_VbyF_Vars();
void Init_FluxSpeedEst_Vars();
void Init_RotPosEst_Vars();
void Init_PIcntrl_Vars();
void Init_SVM_Vars();
void Init_Filt_Vars();
void Init_DynBrake_Vars();
void Init_DataLogger();
void Init_SDRAMLogger();
void Init_EPROMAccess();


void ADC_READ();
void EXT_DAC_Write(int DAC1_DATA,int DAC2_DATA, int DAC3_DATA, int DAC4_DATA);
void ADC_Processing();
void Offset_Reading();
void Digital_Op_Control(int op1, int op2, int op3);
void PTC_Relay_Control(int ip);
void Noise_Supp_Control(int ip);
void Fault_Latch_Reset();
void Relay_Control(int rl1, int rl2);
void SerialComm();
void Serial_CommA();
void Serial_CommB();
void Serial_CommC();
void SDRAMWrite();
void SDRAMRead();
void EPROM128();
void EPROMWrite();
void EPROMRead();
void Wait_for_Interrupt();
void Transform_3to2(_iq ,_iq ,_iq);                     // Function to Transform 3-axis to 2-axis
void Transform_3to2Grid(_iq ,_iq ,_iq);
void Transform_Forward(_iq ,_iq ,_iq ,_iq );            // Function to Transform alpha-beta to d-q
void Transform_Backward(_iq ,_iq ,_iq ,_iq );           // Function to Transform d-q to alpha-beta
void Transform_2to3(_iq, _iq);                          // Function to Convert 2-phase voltage to 3-phase voltage and

void Software_Current_Protection();                     // Over Current Protection
void Inverter_Voltage_Sensing();                        // Inverter Volatge Sensing
void Single_Phasing();
void One_PWM_Missing();
void Phase_Sequence();
void Unbalance_Current();
void Grid_Voltage_Sensing();
void DC_Link_Voltage_Sensing();
void Temperature_Sensing();
void Motor_Speed_Sensing();
void HeatSinkFan_Sensing();
void SMPSFan_Sensing();
void Enclosure_Temperature();


void Interface();                                       // Function to Configure I/O Control of VFD Operation
void Motor_Selector();                                  // Function to select the Motor type
void Motor_Control();                                   // Function to Control Motor
void Dyn_Brake_Con();                                   // Function to Control Dynamic Braking

void INV_Config();                                      // Function to Configure the Inverter Functions of the Controller

void Freq_Init_Ramp();
void Freq_Ramp_Gen();                                   // Function to Generate Frequency Ramp
void Theta_Change_Ramp();
void Angle_Ramp();                                      // Function to Generate Ramped Angle
void Vsd_Ramp_Gen();                                    // Function to Generate Vsd Ramp
void Vsq_Ramp_Gen();                                    // Function to Generate Vsq Ramp
void VbyF_Gen();                                        // Function to maintain V/F profile

void PI_Speed();                                        // Function to implement PI Controller
void PI_Dcurrent();                                     // Function to implement PI Controller
void PI_Qcurrent();                                     // Function to implement PI Controller
void PI_Imrcurrent();


void Vsd_Decoupling_Control();                          // Function to control the Isd Current Reference & generate the Vsd Ref
void Vsq_Decoupling_Control();                          // Function to control the Isq Current Reference & generate the Vsq Ref

void Rotor_Position_Estimator();                        // Function to Estimate the PMSM Rotor Position
void Flux_Speed_Estimator();                            // Function to Estimate the Flux and Speed (Sensor-less Control)

void VSI_Model ();                                      // Function to Estimate the actual output voltage from Inverter

void THIPWM_Gen();                                      // Function for Third Harmonic PWM Generation (FOC)
void EPWM_Generator();                                  // Function to Generate PWM from DSP

void Data_Logger();                                     // Function to log data in RAM
signed int Data_Selector(int16 Channel);

void INVERTER_ON();
void INVERTER_OFF();

//--------------------------------------------------------------------------------------------------------------------------------------------------
//Function Declarations for MODBUS Communication
//--------------------------------------------------------------------------------------------------------------------------------------------------
void Modbus();
void UartWrite(int data);
void Uart_Serial();
void Update_datalogger();
void Init_Modbus_Vars();
//--------------------------------------------------------------------------------------------------------------------------------------------------
extern float Timer_counter;
extern int Q_Decrement_Flag;
extern int dac_channel ;

extern int adc_init_counter;
void I2C_WriteRead();
extern int multi_counter ;
extern int adc_init_mot_volt ;

