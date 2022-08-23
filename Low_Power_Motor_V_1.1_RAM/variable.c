//==================================================================================================================================================
// FILE NAME		: variable.c
// DATE             : 01-Feb-2018
// Project			: VARIABLE FREQUENCY DRIVE FOR COMPRESSOR CONTROL APPLICATIONS
// Project Code		: PEG 124B
// Author			: Rohit V Thomas[ELGI], Manju R[CDAC], Rinu J Vijyan [CDAC], Prashobh [CDAC], Sapna Ravindran[CDAC]
//==================================================================================================================================================
// Include Header Files....
//==================================================================================================================================================
#include "DSP28234_SysCtrl.h"
#include "DSP28234_Device.h"
#include "DSP28234_Gpio.h"
#include "DSP28234_Xintf.h"
#include "DSP28234_EPwm.h"
#include "DSP28234_Piectrl.h"
#include "DSP28234_Sci.h"
#include "DSP2833x_Adc.h"
#include "I2C.h"
#include "variable.h"
#include "IQmathLib.h"


//-------------------------------------------------------------------------------------------------------------------------------------------
// Structure Declarations
//--------------------------------------------------------------------------------------------------------------------------------------------	
union SystemFlags VFD_Status;
struct IntAdcData VFD_IntAdcData;
struct UserData User_Data;
struct SystemData VFD_Data;
struct LimitsData VFD_Limits;
struct MotorLimts Motor_Limits;
struct UserWarnings  User_Warnings;
struct MotorData MOT_Data;
struct RampGen MOT_RG;
struct RampGen MOT_RGI;
struct RampCntrl MOT_RC;
struct RampGen MOT_RGD;
struct RampGen MOT_RGQ;
struct VbyFprofile MOT_VF;
struct FluxSpeedEstim MOT_FSE;
struct RotorPosEstim PMSM_RPE;
struct PIcontrol PI_Spd;
struct PIcontrol PI_Imr;
struct PIcontrol PI_Isd;
struct PIcontrol PI_Isq;
struct PWMgen MOT_SVM;
struct RampCntrl Dyn_Con;
struct ModbusMode VFD_ModbusMode;
struct DataLog Dat_log;

//-------------------------------------------------------------------------------------------------------------------------------------------
//Variable Declarations
//--------------------------------------------------------------------------------------------------------------------------------------------	
Uint16 Debounce_Counter=0;
Uint32 SDMemTot, SDMemInt, SDMemCnt;
Uint16 SDDelCnt, DACDelCnt, SDCh, SDChs, SDBuffCnt;

//-------------------------------------------------------------------------------------------------------------------------------------------
//Variable Declarations for MODBUS Communication
//-------------------------------------------------------------------------------------------------------------------------------------------
unsigned char TxCount=0;
unsigned char RxCount=0;
Uint16 timingCount;
unsigned char tmpwritevalue1;
unsigned char tmpwritevalue2;
long tmpwritevalue;
long *StructAddr[7];
int32 *StructAddrint[8];
float32 *StructAddrfloat[7];

//--------------------------------------------------------------------------------------------------------------------------------------------
Uint16 volatile * AddrPtr = (unsigned  int *) SRAMBaseAdddr;
Uint16 volatile * SegmentStrtAddr = (unsigned int *) SRAMStartAddr;
//--------------------------------------------------------------------------------------------------------------------------------------------
float Timer_counter=0;
int Q_Decrement_Flag = 0;
int dac_channel  = 0;
int temperature_counter = 0;
int adc_init_counter = 0;
int multi_counter = 0;
int adc_init_mot_volt = 0;

