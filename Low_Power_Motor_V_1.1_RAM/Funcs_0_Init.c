//==================================================================================================================================================
// FILE NAME        : Funcs_0_Init.c
// DATE             : 01-Feb-2018
// Project          : VARIABLE FREQUENCY DRIVE FOR COMPRESSOR CONTROL APPLICATIONS
// Project Code     : PEG 124B
// Author           : Rohit V Thomas[ELGI], Manju R[CDAC] & Prashobh[CDAC]
//==================================================================================================================================================
// Include Header Files....
//==================================================================================================================================================
#include "DSP28234_Device.h"
#include "DSP28234_GlobalPrototypes.h"                      // Prototypes for global functions within the .c files.
#include "variable.h"
#include "IQmathLib.h"
#include "DLOG_4CH_IQ.h"                                    // Include header for the DLOG_4CH object

// Define DATALOG module
DLOG_4CH_IQ dlog;
int16 DBUFF0[DLOG_SIZE];
int16 DBUFF1[DLOG_SIZE];
int16 DBUFF2[DLOG_SIZE];
int16 DBUFF3[DLOG_SIZE];
int16 dval0;
int16 dval1;
int16 dval2;
int16 dval3;

//--------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------------------------------
void delay(unsigned int count);
//--------------------------------------------------------------------------------------------------------------------------------------------------
#define EPWM_ISR_VecIDLo    *(volatile unsigned int *)0xD60     // EPWM UnderFlow Interrupt Vector ID
#define EPWM_ISR_VecIDHi    *(volatile unsigned int *)0xD61     // EPWM UnderFlow Interrupt Vector ID
#define EPWM_TZ_ISR_VecIDLo *(volatile unsigned int *)0xD50     // EPWM TZ Vector ID
#define EPWM_TZ_ISR_VecIDHi *(volatile unsigned int *)0xD51     // EPWM TZ Vector ID
#define TIMER_One_ISRLo    *(volatile unsigned int *)0xD1A
#define TIMER_One_ISRHi    *(volatile unsigned int *)0xD1B
//--------------------------------------------------------------------------------------------------------------------------------------------------

void Initialise_DSP()
{
//-------------------------------------------------------------------------------------------------------------------
// Status Register 0 [ST0]
//-------------------------------------------------------------------------------------------------------------------
// PM           - Product shift mode bits.
// OVM          - Overflow mode bit.
//                0 - Results overflow normally in ACC.
//                1 - ACC is filled with either its most positive or most negative value
//                    If ACC overflows in the positive direction (from 7FFFFFFF to 80000000 ),ACC is then filled with 7FFFFFFF
//                    If ACC overflows in the negative direction (from 80000000 to 7FFFFFFF ),ACC is then filled with 80000000
// SXM          - Sign-extension mode bit.
//                0 - Sign extension is suppressed. (The 16-bit value is treated as unsigned)
//                1 - Sign extension is enabled. (The 16-bit value is treated as signed)
//-------------------------------------------------------------------------------------------------------------------
// Status Register 1 [ST1]
//-------------------------------------------------------------------------------------------------------------------
// M0M1MAP      - The M0M1MAP bit should always remain set to 1 in the C28x object mode.The M0M1MAP bit may be set
//                low when operating in C27x-compatible mode
// OBJMODE      - Object compatibility mode bit.This mode is used to select between C27x object
//                mode (OBJMODE == 0) and C28x object mode (OBJMODE == 1) compatibility
// AMODE        - Address mode bit.
// EALLOW       - Emulation access enable bit.This bit, when set, enables access to emulation and
//                other protected registers
// VMAP         - Vector map bit.VMAP determines whether the CPU interrupt vectors are mapped to the lowest or highest addresses in program memory
//                0 - CPU interrupt vectors are mapped to the bottom of program memory
//                1 - CPU interrupt vectors are mapped to the top of program memory
// INTM         - Interrupt global mask bit.This bit globally enables or disables all maskable CPU interrupts
//                0 - Maskable interrupts are globally enabled
//                1 - Maskable interrupts are globally disabled
//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
    asm(" SETC  OBJMODE");      // C28xx Object Mode Selection
    asm(" CLRC  AMODE");        // C28xx Address Mode Selection
    asm(" SETC  M0M1MAP");      // The M0M1MAP bit should always remain set to 1 in the C28x object mode
    asm(" SETC  VMAP");         // Vector map bit
    asm(" SETC  INTM");         // Globally Disable all interrupts
    asm(" SETC  OVM");          // Saturation after Overflow
    asm(" SETC  SXM");          // Allow Sign Extension mode
    asm(" SPM   0");            // No Product Shift mode

    InitSysCtrl();              // Initialize System Control: PLL, WatchDog, enable Peripheral Clocks
    INVERTER_OFF();             // Setting default OFF state for inverter PWM to avoid false triggering of inverter fault.
    InitGpio();                 // Initialize General Purpose Input Output
    Fault_Latch_Reset();        // Reset
    InitInverterEPwm();         // Initialize the EPWM Module Registers
    InitSCIComm();              // Initialize the SCI Module Registers
    InitADCint();               // Initialize the Internal ADC Module Registers
    InitSPI();                  // Initialize the SPI Module Registers
    InitTimer();
    InitPieCtrl();              // Initialize the Peripheral Interupt control register
    InitPieVectTable();         // Initialize the PIE Vector table

}
//====================================================================================================================
// Initialise System Control...
//====================================================================================================================
// This function initializes the System Control registers to a known state.
// - Disables the watchdog
// - Set the PLLCR for proper SYSCLKOUT frequency
// - Set the pre-scaler for the high and low frequency peripheral clocks
// - Enable the clocks to the peripherals
//====================================================================================================================
void InitSysCtrl()
{
   DisableDog();                                    // Disable the watchdog
   InitPll();                                       // Initialize the PLL control register: PLLCR and DIVSEL
   InitPeripheralClocks();                          // Initialize the clocks to peripheral modules
}
//====================================================================================================================
// Function to disable the Watchdog Timer...
//====================================================================================================================
void DisableDog()
{
//-------------------------------------------------------------------------------------------------------------------
    EALLOW;                                         // Enable access to protected registers
//-------------------------------------------------------------------------------------------------------------------
    SysCtrlRegs.SCSR  = 0x0002;                     // System Control and Status Registers
//-------------------------------------------------------------------------------------------------------------------
//          000000000000 0 0 1 0
//          |||||||||||| | | | |
//          FEDCBA987654 3 2 1 0
// bit15-3  [RESERVED]      0..0      Stop immediately on Emulation suspend
// bit2     [WDINTS]           0      Watchdog interrupt status bit [Read only]
// bit1     [WDENINT]          1      Watchdog interrupt enable
// bit0     [WDOVERIDE]        0      Watchdog override
//--------------------------------------------------------------------------------------------------------------------------
    SysCtrlRegs.WDCR  = 0x00EF;                     // Watchdog Control Register
//--------------------------------------------------------------------------------------------------------------------------
//          00000000 1 1 101 000
//          |||||||| | | ||| |||
//          FEDCBA98 7 6 543 210
// bit15-8  [RESERVED]      0..0      Reserved
// bit7     [WDFLAG]           1      Watchdog reset status flag bit
// bit6     [WDDIS]            1      Watchdog disable.
// bit5..3  [WDCHK]          101      Set 101 to perform a write to this register unless the intent is to reset the device via software
// bit2..0  [WDPS]           000      Watchdog pre-scale to configure the watchdog counter clock, WDCLK = OSCCLK/512/1 (default)
//                           001      WDCLK = OSCCLK/512/1
//                           010      WDCLK = OSCCLK/512/2
//                           011      WDCLK = OSCCLK/512/4
//                           100      WDCLK = OSCCLK/512/8
//                           ...
//                           111      WDCLK = OSCCLK/512/64
//--------------------------------------------------------------------------------------------------------------------------
    SysCtrlRegs.WDKEY = 0x55;                       // Watchdog Reset Key Register
//--------------------------------------------------------------------------------------------------------------------------
//          00000000 11101000
//          |||||||| ||||||||
//          FEDCBA98 76543210
// bit15-8  [RESERVED]      0..0      Reserved
// bit7..0  [WDKEY]         0x55      Watchdog Key Sequence, WDCNTR is enabled to be reset if next value is 0xAA
//                          0xAA      Watchdog Key Sequence, WDCNTR is reset
//--------------------------------------------------------------------------------------------------------------------------
    SysCtrlRegs.WDKEY = 0xAA;                       // Writing 0x55 followed by 0xAA to WDKEY causes the WDCNTR bits to be cleared.
//--------------------------------------------------------------------------------------------------------------------------
//          00000000 11101000
//          |||||||| ||||||||
//          FEDCBA98 76543210
// bit15-8  [RESERVED]      0..0      Reserved
// bit7..0  [WDKEY]         0x55      Watchdog Key Sequence, WDCNTR is enabled to be reset if next value is 0xAA
//                          0xAA      Watchdog Key Sequence, WDCNTR is reset
//--------------------------------------------------------------------------------------------------------------------------
    EDIS;                                           // Disable access to protected registers
//--------------------------------------------------------------------------------------------------------------------------
}
//====================================================================================================================
// Function to initialize the PLL registers.
//====================================================================================================================
void InitPll()
{
//-------------------------------------------------------------------------------------------------------------------------
    EALLOW;                                         // Enable access to protected registers
//--------------------------------------------------------------------------------------------------------------------------
    SysCtrlRegs.PLLSTS.all = 0x0101;                // PLL Status Register
//--------------------------------------------------------------------------------------------------------------------------
//          0000000 10 0 0 0 0 0 0 1
//          ||||||| || | | | | | | |
//          FEDCBA9 87 6 5 4 3 2 1 0
// bit15-9  [RESERVED]      0..0      Reserved
// bit8..7  [DIVSEL]          00/01   Select Divide By 4 for CLKIN
//                            10      Select Divide By 2 for CLKIN
//                            11      Select Divide By 1 for CLKIN, used when PLL bypassed
// bit6     [MCLKOFF]          0      Missing clock-detect off bit
// bit5     [OSCOFF]           0      Oscillator Clock Off Bit[The OSCCLK signal from X1, X1/X2 or XCLKIN is fed to the PLL block. (default)]
// bit4     [MCLKCLR]          0      Missing Clock Clear Bit
// bit3     [MCLKSTS]          0      Missing Clock Status Bit.
// bit2     [PLLOFF]           0      PLL On (default)
// bit1     [RESERVED]         0      Reserved
// bit0     [PLLLOCKS]         1      PLL Lock Status Bit
//--------------------------------------------------------------------------------------------------------------------------
    SysCtrlRegs.PLLCR.bit.DIV = 0x8;                // PLL Control Register
//-------------------------------------------------------------------------------------------------------------------
//          000000000000 1000
//          |||||||||||| ||||
//          FEDCBA987654 3210
// bit15-4  [RESERVED]      0..0      Reserved
// bit3..0  [DIV]            0x0      PLL Bypass, (OSCCLK)/4 if DIVSEL=0/1, (OSCCLK)/2 if DIVSEL=2, (OSCCLK) if DIVSEL=3
//                           0x1      (OSCCLK*1)/4 if DIVSEL=0/1, (OSCCLK*1)/2 if DIVSEL=2, (OSCCLK) if DIVSEL=3
//                           0x2      (OSCCLK*2)/4 if DIVSEL=0/1, (OSCCLK*2)/2 if DIVSEL=2, (OSCCLK) if DIVSEL=3
//                           0x3      (OSCCLK*3)/4 if DIVSEL=0/1, (OSCCLK*3)/2 if DIVSEL=2, (OSCCLK) if DIVSEL=3
//                           ...
//                           0x8      (OSCCLK*8)/4 if DIVSEL=0/1, (OSCCLK*8)/2 if DIVSEL=2, (OSCCLK) if DIVSEL=3
//                           0x9-0xF   Reserved
//-------------------------------------------------------------------------------------------------------------------
    EDIS;                                           // Disable access to protected registers
//-------------------------------------------------------------------------------------------------------------------
}
//====================================================================================================================
// Function to Initialise the Interrupt Vector Table
//====================================================================================================================
void InitPeripheralClocks()
{
    EALLOW;                                                 // Enable access to protected registers


//  PERIPH CLOCK REG CONFIGURATION BITWISE
/*
//-------------------------------------------------------------------------------------------------------------------
// Peripheral Clock Control Register 0
//-------------------------------------------------------------------------------------------------------------------
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC   = ENABLE;           // Enable  ePWM Module Time Base Clock.Allows the user to globally synchronize all enabled ePWM modules to the time base clock
    SysCtrlRegs.PCLKCR0.bit.ADCENCLK    = ENABLE;           // Enable  ADC     module Clock
    SysCtrlRegs.PCLKCR0.bit.I2CAENCLK   = DISABLE;          // Enable  I2C     module Clock
    SysCtrlRegs.PCLKCR0.bit.SCICENCLK   = ENABLE;           // Enable  SCI-C   module Clock
    SysCtrlRegs.PCLKCR0.bit.SPIAENCLK   = ENABLE;          // Disable SPI-A   module Clock
    SysCtrlRegs.PCLKCR0.bit.SCIAENCLK   = ENABLE;           // Enable  SCI-A   module Clock
    SysCtrlRegs.PCLKCR0.bit.SCIBENCLK   = ENABLE;           // Enable  SCI-B   module Clock
    SysCtrlRegs.PCLKCR0.bit.MCBSPAENCLK = DISABLE;          // Disable McBSP-A module Clock
    SysCtrlRegs.PCLKCR0.bit.MCBSPBENCLK = DISABLE;          // Disable McBSP-B module Clock
    SysCtrlRegs.PCLKCR0.bit.ECANAENCLK  = DISABLE;          // Disable eCAN-A  module Clock
    SysCtrlRegs.PCLKCR0.bit.ECANBENCLK  = DISABLE;          // Disable eCAN-B  module Clock
//-------------------------------------------------------------------------------------------------------------------
// Peripheral Clock Control Register 1
//-------------------------------------------------------------------------------------------------------------------
    SysCtrlRegs.PCLKCR1.bit.EPWM1ENCLK  = ENABLE;           // Enable   ePWM1 module Clock
    SysCtrlRegs.PCLKCR1.bit.EPWM2ENCLK  = ENABLE;           // Enable   ePWM2 module Clock
    SysCtrlRegs.PCLKCR1.bit.EPWM3ENCLK  = ENABLE;           // Enable   ePWM3 module Clock
    SysCtrlRegs.PCLKCR1.bit.EPWM4ENCLK  = ENABLE;           // Enable   ePWM4 module Clock
    SysCtrlRegs.PCLKCR1.bit.EPWM5ENCLK  = ENABLE;           // Enable   ePWM5 module Clock
    SysCtrlRegs.PCLKCR1.bit.EPWM6ENCLK  = ENABLE;           // Enable   ePWM6 module Clock
    SysCtrlRegs.PCLKCR1.bit.ECAP1ENCLK  = DISABLE;          // Disable  eCAP1 module Clock
    SysCtrlRegs.PCLKCR1.bit.ECAP2ENCLK  = DISABLE;          // Disable  eCAP2 module Clock
    SysCtrlRegs.PCLKCR1.bit.ECAP3ENCLK  = DISABLE;          // Disable  eCAP3 module Clock
    SysCtrlRegs.PCLKCR1.bit.ECAP4ENCLK  = DISABLE;          // Disable  eCAP4 module Clock
    SysCtrlRegs.PCLKCR1.bit.ECAP5ENCLK  = DISABLE;          // Disable  eCAP5 module Clock
    SysCtrlRegs.PCLKCR1.bit.ECAP6ENCLK  = DISABLE;          // Disable  eCAP6 module Clock
    SysCtrlRegs.PCLKCR1.bit.EQEP1ENCLK  = DISABLE;          // Disable  eQEP1 module Clock
    SysCtrlRegs.PCLKCR1.bit.EQEP2ENCLK  = DISABLE;          // Disable  eQEP2 module Clock
//-------------------------------------------------------------------------------------------------------------------
// Peripheral Clock Control Register 3
//-------------------------------------------------------------------------------------------------------------------
    SysCtrlRegs.PCLKCR3.bit.CPUTIMER0ENCLK = DISABLE;       // Disable CPU Timer 0 Clock
    SysCtrlRegs.PCLKCR3.bit.CPUTIMER1ENCLK = ENABLE;       // Disable CPU Timer 1 Clock
    SysCtrlRegs.PCLKCR3.bit.CPUTIMER2ENCLK = DISABLE;       // Disable CPU Timer 2 Clock
    SysCtrlRegs.PCLKCR3.bit.DMAENCLK       = DISABLE;       // Disable DMA Module  Clock
    SysCtrlRegs.PCLKCR3.bit.GPIOINENCLK    = ENABLE;        // Enable  GPIO Input  Clock
    SysCtrlRegs.PCLKCR3.bit.XINTFENCLK     = ENABLE;        // Enable External Interface (XINTF) Clock
//--------------------------------------------------------------------------------------------------------------------------
    SysCtrlRegs.HISPCP.bit.HSPCLK = 0x6;                    // High-Speed Peripheral Clock Prescaler register
//--------------------------------------------------------------------------------------------------------------------------
//          0000000000000 110
//          ||||||||||||| |||
//          FEDCBA9876543 210
// bit15-3  [RESERVED]      0..0      Reserved
// bit2..0  [HSPCLK]         0x0      High speed clock = SYSCLKOUT/1
//                           0x1      High speed clock = SYSCLKOUT/2 (reset default)
//                           0x2      High speed clock = SYSCLKOUT/4
//                           0x3      High speed clock = SYSCLKOUT/6
//                           ...
//                           0x6      High speed clock = SYSCLKOUT/12
//                           0x7      High speed clock = SYSCLKOUT/14
//--------------------------------------------------------------------------------------------------------------------------
    SysCtrlRegs.LOSPCP.bit.LSPCLK = 0x2;                    // Low-Speed Peripheral Clock Prescaler register
//--------------------------------------------------------------------------------------------------------------------------
//          0000000000000 100
//          ||||||||||||| |||
//          FEDCBA9876543 210
// bit15-3  [RESERVED]      0..0      Reserved
// bit2..0  [HSPCLK]         0x0      Low speed clock = SYSCLKOUT/1
//                           0x1      Low speed clock = SYSCLKOUT/2
//                           0x2      Low speed clock = SYSCLKOUT/4 (reset default)
//                           0x3      Low speed clock = SYSCLKOUT/6
//                           0x4      Low speed clock = SYSCLKOUT/8
//                           ...
//                           0x7      Low speed clock = SYSCLKOUT/14



//-------------------------------------------------------------------------------------------------------------------
// XINTF Configuration Register
//-------------------------------------------------------------------------------------------------------------------
//    XintfRegs.XINTCNF2.bit.WRBUFF       = 0x0;  // Write Buffer Depth
    XintfRegs.XINTCNF2.bit.CLKMODE      = 0x0;  // XCLKOUT is equal to XTIMCLK  [External Interface Clock]
    XintfRegs.XINTCNF2.bit.CLKOFF       = 0x0;  // Enable XCLKOUT (default)
//    XintfRegs.XINTCNF2.bit.WLEVEL       = 0x0;  // WLEVEL [Write buffer level] [Read only]
//    XintfRegs.XINTCNF2.bit.HOLD         = 0x0;  // HOLD Signal [Read]
//    XintfRegs.XINTCNF2.bit.HOLDS        = 0x0;  // HOLDS Signal [Read]
//    XintfRegs.XINTCNF2.bit.HOLDAS       = 0x0;  // HOLDAS Signal [Read]
    XintfRegs.XINTCNF2.bit.XTIMCLK      = 0x0;  // 0x0      =>  XTIMCLK = SYSCLKOUT/1
                                                // 0x1      =>  XTIMCLK = SYSCLKOUT/2 (default)
                                                // 0x2-0x7  =>  Reserved
//-------------------------------------------------------------------------------------------------------------------
// XINTF Timing0 Register
//-------------------------------------------------------------------------------------------------------------------
    XintfRegs.XTIMING0.all  = 0x00433fff;
    XintfRegs.XTIMING7.all  = 0x00433fff;

//    XintfRegs.XTIMING0.bit.X2TIMING     = 1;    // Scaling factor 0 => 1:1
                                                //                1 => 2:1
//    XintfRegs.XTIMING0.bit.XSIZE        = 0x3;  // Data Lines   0x1 => zone uses 32 data lines
                                                //              0x3 => zone uses 16 data lines
//    XintfRegs.XTIMING0.bit.READYMODE    = 0;    // XREADY Input sampling for the zone   0 => XREADY input is synchronous for the zone
                                                //                                      1 => XREADY input is asynchronous for the zone
//    XintfRegs.XTIMING0.bit.USEREADY     = 0;    // Sample or Ignore XREADY input signal 0 => XREADY signal is ignored when accesses are made to the zone
                                                //                                      1 => XREADY signal can further extend the active portion of an access to the zone
//    XintfRegs.XTIMING0.bit.XRDLEAD      = 0x3;  // Read cycle lead wait state period    0x3 => 6  XTIMCLK cycles
//    XintfRegs.XTIMING0.bit.XRDACTIVE    = 0x7;  // Read cycle active wait state period  0x7 => 14 XTIMCLK cycles
//    XintfRegs.XTIMING0.bit.XRDTRAIL     = 0x3;  // Read cycle trail wait state period   0x3 => 6  XTIMCLK cycles
//    XintfRegs.XTIMING0.bit.XWRLEAD      = 0x3;  // Write cycle lead wait state period   0x3 => 6  XTIMCLK cycles
//    XintfRegs.XTIMING0.bit.XWRACTIVE    = 0x3;  // Write cycle active wait state period 0x7 => 14 XTIMCLK cycles
//    XintfRegs.XTIMING0.bit.XWRTRAIL     = 0x3;  // Write cycle trail wait state period  0x3 => 6  XTIMCLK cycles

*/

// PERIPH CLOCK REG CONFIGURATION BITALL

    SysCtrlRegs.PCLKCR0.all = 0x0D2C;
    SysCtrlRegs.PCLKCR1.all = 0x003F;
    SysCtrlRegs.PCLKCR3.all = 0x3200;
    SysCtrlRegs.HISPCP.all  = 0x0006;
    SysCtrlRegs.LOSPCP.all  = 0x0002;

    XintfRegs.XINTCNF2.all  = 0x00000D10;
    XintfRegs.XTIMING0.all  = 0x00433fff;
    XintfRegs.XTIMING6.all  = 0x00433fff;
    XintfRegs.XTIMING7.all  = 0x00433fff;

//

    EDIS;                                       // Disable access to protected registers
}
//====================================================================================================================
// Function to initialize the General Purpose Ports 
//  - Port A [GPIO0  - GPIO31]
//  - Port B [GPIO32 - GPIO63]
//  - Port C [GPIO64 - GPIO87]
//====================================================================================================================
void InitGpio()
{
// GPIO (GENERAL PURPOSE I/O) CONFIG
//-------------------------------------------------------------------------------------------------------------------
// This function initialises the General Purpose Ports A, B & C
// Each GPIO pin can be: 
//      a) a GPIO input/output
//      b) peripheral function 1
//      c) peripheral function 2
//      d) peripheral function 3
//      By default, all are GPIO Inputs
//-----------------------
// QUICK NOTES on USAGE:
//-----------------------
// If GpioCtrlRegs.GP?MUX?bit.GPIO?= 1, 2 or 3 (i.e. Non GPIO func), then leave
//  rest of lines commented
// If GpioCtrlRegs.GP?MUX?bit.GPIO?= 0 (i.e. GPIO func), then:
//  1) uncomment GpioCtrlRegs.GP?DIR.bit.GPIO? = ? and choose pin to be IN or OUT
//  2) If IN, can leave next line commented
//  3) If OUT, write ..GPA_CLEAR.. to force pin LOW or
//             write ..GPA_SET..   to force pin HIGH

//-------------------------------------------------------------------------------------------------------------------

   EALLOW;                          // Allow access to protected registers

//  GPIO CONFIGURATION BITWISE

//-------------------------------------------------------------------------------------------------------------------
// GPIO PortA MUX Register
//-------------------------------------------------------------------------------------------------------------------
   //-------------------------------------------------------------------------------------------------------------------
// GPIO-00 - PIN FUNCTION = PWM1A
   GpioCtrlRegs.GPAMUX1.bit.GPIO0   =  PERIPHERAL_1;        // GPIO0   = PWM1A
// GpioCtrlRegs.GPADIR.bit.GPIO0    =  OUTPUT;              // PWM1A
// GpioDataRegs.GPADAT.bit.GPIO0    =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-01 - PIN FUNCTION = PWM1B
    GpioCtrlRegs.GPAMUX1.bit.GPIO1   =  PERIPHERAL_1;        // GPIO1   = PWM1B
// GpioCtrlRegs.GPADIR.bit.GPIO1    =  OUTPUT;              // PWM1B
// GpioDataRegs.GPADAT.bit.GPIO1    =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-02 - PIN FUNCTION = PWM2A
    GpioCtrlRegs.GPAMUX1.bit.GPIO2   =  PERIPHERAL_1;        // GPIO2   = PWM2A
// GpioCtrlRegs.GPADIR.bit.GPIO2    =  OUTPUT;              // PWM2A
// GpioDataRegs.GPADAT.bit.GPIO2    =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-03 - PIN FUNCTION = PWM2B
    GpioCtrlRegs.GPAMUX1.bit.GPIO3   =  PERIPHERAL_1;        // GPIO3   = PWM2B
// GpioCtrlRegs.GPADIR.bit.GPIO3    =  OUTPUT;              // PWM2B
// GpioDataRegs.GPADAT.bit.GPIO3    =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-04 - PIN FUNCTION = PWM3A
    GpioCtrlRegs.GPAMUX1.bit.GPIO4   =  PERIPHERAL_1;        // GPIO4   = PWM3A
// GpioCtrlRegs.GPADIR.bit.GPIO4    =  OUTPUT;              // PWM3A
// GpioDataRegs.GPADAT.bit.GPIO4    =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-05 - PIN FUNCTION = PWM3B
   GpioCtrlRegs.GPAMUX1.bit.GPIO5   =  PERIPHERAL_1;        // GPIO5   = PWM3B
// GpioCtrlRegs.GPADIR.bit.GPIO5    =  OUTPUT;              // PWM3B
// GpioDataRegs.GPADAT.bit.GPIO5    =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------------------------
// GPIO PortA MUX1 Register
// GPIO-06 - PIN FUNCTION = GPIO Input
   GpioCtrlRegs.GPAMUX1.bit.GPIO6   =  GPIO;        // GPIO6   = DI_FailStop
   GpioCtrlRegs.GPADIR.bit.GPIO6    =  INPUT;              // DI_FailSTOP
// GpioDataRegs.GPADAT.bit.GPIO6    =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------   
// GPIO-07 - PIN FUNCTION = GPIO Input
   GpioCtrlRegs.GPAMUX1.bit.GPIO7   =  GPIO;        // GPIO7   = DI_GNDFault
   GpioCtrlRegs.GPADIR.bit.GPIO7    =  INPUT;              // DI_GNDFault
// GpioDataRegs.GPADAT.bit.GPIO7    =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------   
//  GPIO-08 - PIN FUNCTION = GPIO Input
   GpioCtrlRegs.GPAMUX1.bit.GPIO8   =  GPIO;        // GPIO8   = DI_OvrCurr
   GpioCtrlRegs.GPADIR.bit.GPIO8    =  INPUT;              // DI_OvrCurr
// GpioDataRegs.GPADAT.bit.GPIO8    =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------   
// GPIO-09 - PIN FUNCTION = GPIO Output
   GpioCtrlRegs.GPAMUX1.bit.GPIO9   =  GPIO;        // GPIO9   = Fault Latch Reset to reset the gate driver fault
   GpioCtrlRegs.GPADIR.bit.GPIO9    =  OUTPUT;
   GpioDataRegs.GPADAT.bit.GPIO9    =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------   
// GPIO-10 - PIN FUNCTION = PWM6A  for PWM Fan Heat Sink
   GpioCtrlRegs.GPAMUX1.bit.GPIO10  =  PERIPHERAL_1;        // GPIO10  = PWM6A
// GpioCtrlRegs.GPADIR.bit.GPIO10   =  OUTPUT;              // PWM6A
// GpioDataRegs.GPADAT.bit.GPIO10   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------      
// GPIO-11 - PIN FUNCTION = PWM6B for PWM FAN Enclosure
   GpioCtrlRegs.GPAMUX1.bit.GPIO11  =  PERIPHERAL_1;        // GPIO11  = PWM6B
// GpioCtrlRegs.GPADIR.bit.GPIO11   =  OUTPUT;              // PWM6B
// GpioDataRegs.GPADAT.bit.GPIO11    =  GPIO_CLEAR;         // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------  
// GPIO-12 - PIN FUNCTION = TZ1    
   GpioCtrlRegs.GPAMUX1.bit.GPIO12  =  PERIPHERAL_1;        // GPIO12  = TZ1
// GpioCtrlRegs.GPADIR.bit.GPIO12   =  OUTPUT;              // TZ1
// GpioDataRegs.GPADAT.bit.GPIO12    =  GPIO_CLEAR;         // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------     
// GPIO-13 - PIN FUNCTION = DYNAMIC BRAKE Control
   GpioCtrlRegs.GPAMUX1.bit.GPIO13  =  GPIO;        // GPIO13  = Dynamic Brake Control
   GpioCtrlRegs.GPADIR.bit.GPIO13   =  OUTPUT;              // TZ2
   GpioDataRegs.GPADAT.bit.GPIO13   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------   
// GPIO-14 - PIN FUNCTION = GPIO    
   GpioCtrlRegs.GPAMUX1.bit.GPIO14  =  GPIO;                // GPIO14  = RO_OP1
   GpioCtrlRegs.GPADIR.bit.GPIO14   =  OUTPUT;              // RO_OP1
   GpioDataRegs.GPADAT.bit.GPIO14   =  GPIO_SET;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------    
// GPIO-15 - PIN FUNCTION = GPIO    
   GpioCtrlRegs.GPAMUX1.bit.GPIO15  =  GPIO;                // GPIO15  = RO_OP2
   GpioCtrlRegs.GPADIR.bit.GPIO15   =  OUTPUT;              // RO_OP2
   GpioDataRegs.GPADAT.bit.GPIO15   =  GPIO_SET;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//-------------------------------------------------------------------------------------------------------------------
// GPIO PortA MUX2 Register
//-------------------------------------------------------------------------------------------------------------------
// GPIO-16 - PIN FUNCTION = SPISIMOA
   GpioCtrlRegs.GPAMUX2.bit.GPIO16  =  PERIPHERAL_1;        // GPIO16   = SPISIMOA
// GpioCtrlRegs.GPADIR.bit.GPIO16   =  OUTPUT;              // SPISIMOA
// GpioDataRegs.GPADAT.bit.GPIO16   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-17 - PIN FUNCTION = SPISIMIA
   GpioCtrlRegs.GPAMUX2.bit.GPIO17  =  PERIPHERAL_1;        // GPIO17   = SPISIMIA
// GpioCtrlRegs.GPADIR.bit.GPIO17   =  OUTPUT;              // SPISIMIA
// GpioDataRegs.GPADAT.bit.GPIO17   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-18 - PIN FUNCTION = SPICLKA
   GpioCtrlRegs.GPAMUX2.bit.GPIO18  =  PERIPHERAL_1;        // GPIO18   = SPICLKA
// GpioCtrlRegs.GPADIR.bit.GPIO18   =  OUTPUT;              // SPICLKA
// GpioDataRegs.GPADAT.bit.GPIO18   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-19 - PIN FUNCTION = SPISTEA
   GpioCtrlRegs.GPAMUX2.bit.GPIO19  =  PERIPHERAL_1;       // GPIO19   = SPISTEA
   GpioCtrlRegs.GPADIR.bit.GPIO19   =  OUTPUT;              // SPISTEA
   GpioDataRegs.GPADAT.bit.GPIO19   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT

//--------------------------------------------------------------------------------------
// GPIO-20 - PIN FUNCTION = GPIO
   GpioCtrlRegs.GPAMUX2.bit.GPIO20  =  GPIO;                // GPIO20   = SCIA_RxTx_EN
   GpioCtrlRegs.GPADIR.bit.GPIO20   =  OUTPUT;              // SCIA_RxTx_EN
   GpioDataRegs.GPADAT.bit.GPIO20   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-21 - PIN FUNCTION = GPIO
   GpioCtrlRegs.GPAMUX2.bit.GPIO21  =  GPIO;                // GPIO21   = SCIB_RxTx_EN
   GpioCtrlRegs.GPADIR.bit.GPIO21   =  OUTPUT;              // SCIB_RxTx_EN
   GpioDataRegs.GPADAT.bit.GPIO21   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-22 - PIN FUNCTION = SCITXDB
   GpioCtrlRegs.GPAMUX2.bit.GPIO22  =  PERIPHERAL_3;        // GPIO22   = SCITXDB
// GpioCtrlRegs.GPADIR.bit.GPIO22   =  OUTPUT;              // SCITXDB
// GpioDataRegs.GPADAT.bit.GPIO22   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-23 - PIN FUNCTION = SCIRXDB
   GpioCtrlRegs.GPAMUX2.bit.GPIO23  =  PERIPHERAL_3;        // GPIO23   = SCIRXDB
// GpioCtrlRegs.GPADIR.bit.GPIO23   =  OUTPUT;              // SCIRXDB
// GpioDataRegs.GPADAT.bit.GPIO23   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-24 - PIN FUNCTION = GPIO
   GpioCtrlRegs.GPAMUX2.bit.GPIO24  =  GPIO;                // GPIO24   = SCIC_RxTx_EN
   GpioCtrlRegs.GPADIR.bit.GPIO24   =  OUTPUT;              // SCIC_RxTx_EN
   GpioDataRegs.GPADAT.bit.GPIO24   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-25 - PIN FUNCTION = GPIO
   GpioCtrlRegs.GPAMUX2.bit.GPIO25  =  GPIO;                // GPIO25   = EPROM_WP
   GpioCtrlRegs.GPADIR.bit.GPIO25   =  OUTPUT;              // EPROM_WP
   GpioDataRegs.GPADAT.bit.GPIO25   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-26 - PIN FUNCTION = GPIO
   GpioCtrlRegs.GPAMUX2.bit.GPIO26  =  GPIO;                // GPIO26   = DO OP1
   GpioCtrlRegs.GPADIR.bit.GPIO26   =  OUTPUT;              // DO OP1
   GpioDataRegs.GPADAT.bit.GPIO26   =  GPIO_SET;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-27 - PIN FUNCTION = GPIO
   GpioCtrlRegs.GPAMUX2.bit.GPIO27  =  GPIO;                // GPIO27   = DO OP2
   GpioCtrlRegs.GPADIR.bit.GPIO27   =  OUTPUT;              // DO OP@
   GpioDataRegs.GPADAT.bit.GPIO27   =  GPIO_SET;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-28 - PIN FUNCTION = SCIRXDA
   GpioCtrlRegs.GPAMUX2.bit.GPIO28  =  PERIPHERAL_1;        // GPIO28   = SCIRXDA
// GpioCtrlRegs.GPADIR.bit.GPIO28   =  OUTPUT;              // SCIRXDA
// GpioDataRegs.GPADAT.bit.GPIO28   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-29 - PIN FUNCTION = SCITXDA
   GpioCtrlRegs.GPAMUX2.bit.GPIO29  =  PERIPHERAL_1;        // GPIO29   = SCITXDA
// GpioCtrlRegs.GPADIR.bit.GPIO29   =  OUTPUT;              // SCITXDA
// GpioDataRegs.GPADAT.bit.GPIO29   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-30 - PIN FUNCTION = GPIO
   GpioCtrlRegs.GPAMUX2.bit.GPIO30  =  GPIO;                // GPIO30   = ADC_SOC
   GpioCtrlRegs.GPADIR.bit.GPIO30   =  OUTPUT;              // In CC_01 PTC Relay pin is swapped.
   GpioDataRegs.GPADAT.bit.GPIO30   =  GPIO_SET;           // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-31 - PIN FUNCTION = GPIO
   GpioCtrlRegs.GPAMUX2.bit.GPIO31  =  GPIO;                // GPIO31   = DO_OP3
   GpioCtrlRegs.GPADIR.bit.GPIO31   =  OUTPUT;              // In CC_01 RFI_Relay pin is swapped.
   GpioDataRegs.GPADAT.bit.GPIO31   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//-------------------------------------------------------------------------------------------------------------------
// GPIO PortB MUX1 Register
//-------------------------------------------------------------------------------------------------------------------
// GPIO-32 - PIN FUNCTION = SDAA
//   GpioCtrlRegs.GPBMUX1.bit.GPIO32  =  PERIPHERAL_1;        // GPIO32   = SDAA
   GpioCtrlRegs.GPBDIR.bit.GPIO32   =  OUTPUT;              // SDAA
   GpioDataRegs.GPBDAT.bit.GPIO32   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-33 - PIN FUNCTION = SCLA
//   GpioCtrlRegs.GPBMUX1.bit.GPIO33  =  PERIPHERAL_1;        // GPIO33   = SCLA
   GpioCtrlRegs.GPBDIR.bit.GPIO33   =  OUTPUT;              // SCLA
   GpioDataRegs.GPBDAT.bit.GPIO33   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-34 - PIN FUNCTION = GPIO
   GpioCtrlRegs.GPBMUX1.bit.GPIO34  =  GPIO;                // GPIO34   =
   GpioCtrlRegs.GPBDIR.bit.GPIO34   =  INPUT;               // This pin is not used currently
// GpioDataRegs.GPBDAT.bit.GPIO34   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-35 - PIN FUNCTION = GPIO
   GpioCtrlRegs.GPBMUX1.bit.GPIO35  =  GPIO;                // GPIO35   =
   GpioCtrlRegs.GPBDIR.bit.GPIO35   =  INPUT;              // This pin is not used currently
//   GpioDataRegs.GPBDAT.bit.GPIO35   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-36 - PIN FUNCTION = XZCS0
   GpioCtrlRegs.GPBMUX1.bit.GPIO36  =  PERIPHERAL_3;        // GPIO36   = XZCS0
// GpioCtrlRegs.GPBDIR.bit.GPIO36   =  OUTPUT;              // XZCS0
// GpioDataRegs.GPBDAT.bit.GPIO36   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-37 - PIN FUNCTION = XZCS7
   GpioCtrlRegs.GPBMUX1.bit.GPIO37  =  PERIPHERAL_2;        // GPIO37   = XZCS7
// GpioCtrlRegs.GPBDIR.bit.GPIO37   =  OUTPUT;              // XZCS7
// GpioDataRegs.GPBDAT.bit.GPIO37   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-38 - PIN FUNCTION = XWE0
   GpioCtrlRegs.GPBMUX1.bit.GPIO38  =  GPIO;                // GPIO38   = XWE0, Set as GPIO to manually give XWE0 from Program
   GpioCtrlRegs.GPBDIR.bit.GPIO38   =  OUTPUT;              // XWE0     Since EXT_DAC requires XWE0 to be given first then Chip Select
   GpioDataRegs.GPBDAT.bit.GPIO38   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-39 - PIN FUNCTION = XA16
   GpioCtrlRegs.GPBMUX1.bit.GPIO39  =  PERIPHERAL_2;        // GPIO39   = XA16
// GpioCtrlRegs.GPBDIR.bit.GPIO39   =  OUTPUT;              // XA16
// GpioDataRegs.GPBDAT.bit.GPIO39   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-40 - PIN FUNCTION = XA0
   GpioCtrlRegs.GPBMUX1.bit.GPIO40  =  PERIPHERAL_2;        // GPIO40   = XA0
// GpioCtrlRegs.GPBDIR.bit.GPIO40   =  OUTPUT;              // XA0
// GpioDataRegs.GPBDAT.bit.GPIO40   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-41 - PIN FUNCTION = XA1
   GpioCtrlRegs.GPBMUX1.bit.GPIO41  =  PERIPHERAL_2;        // GPIO41   = XA1
// GpioCtrlRegs.GPBDIR.bit.GPIO41   =  OUTPUT;              // XA1
// GpioDataRegs.GPBDAT.bit.GPIO41   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-42 - PIN FUNCTION = XA2
   GpioCtrlRegs.GPBMUX1.bit.GPIO42  =  PERIPHERAL_2;        // GPIO42   = XA2
// GpioCtrlRegs.GPBDIR.bit.GPIO42   =  OUTPUT;              // XA2
// GpioDataRegs.GPBDAT.bit.GPIO42   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-43 - PIN FUNCTION = XA3
   GpioCtrlRegs.GPBMUX1.bit.GPIO43  =  PERIPHERAL_2;        // GPIO43   = XA3
// GpioCtrlRegs.GPBDIR.bit.GPIO43   =  OUTPUT;              // XA3
// GpioDataRegs.GPBDAT.bit.GPIO43   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-44 - PIN FUNCTION = XA4
   GpioCtrlRegs.GPBMUX1.bit.GPIO44  =  PERIPHERAL_2;        // GPIO44   = XA4
// GpioCtrlRegs.GPBDIR.bit.GPIO44   =  OUTPUT;              // XA4
// GpioDataRegs.GPBDAT.bit.GPIO44   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-45 - PIN FUNCTION = XA5
   GpioCtrlRegs.GPBMUX1.bit.GPIO45  =  PERIPHERAL_2;        // GPIO45   = XA5
// GpioCtrlRegs.GPBDIR.bit.GPIO45   =  OUTPUT;              // XA5
// GpioDataRegs.GPBDAT.bit.GPIO45   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-46 - PIN FUNCTION = XA6
   GpioCtrlRegs.GPBMUX1.bit.GPIO46  =  PERIPHERAL_2;        // GPIO46   = XA6
// GpioCtrlRegs.GPBDIR.bit.GPIO46   =  OUTPUT;              // XA6
// GpioDataRegs.GPBDAT.bit.GPIO46   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-47 - PIN FUNCTION = XA7
   GpioCtrlRegs.GPBMUX1.bit.GPIO47  =  PERIPHERAL_2;        // GPIO47   = XA7
// GpioCtrlRegs.GPBDIR.bit.GPIO47   =  OUTPUT;              // XA7
// GpioDataRegs.GPBDAT.bit.GPIO47   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//-------------------------------------------------------------------------------------------------------------------
// GPIO PortB MUX2 Register
//-------------------------------------------------------------------------------------------------------------------
// GPIO-48 - PIN FUNCTION = GPIO
   GpioCtrlRegs.GPBMUX2.bit.GPIO48  =  GPIO;                // GPIO48   = DI_IP1
   GpioCtrlRegs.GPBDIR.bit.GPIO48   =  INPUT;               // DI_IP1
// GpioDataRegs.GPBDAT.bit.GPIO48   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-49 - PIN FUNCTION = GPIO
   GpioCtrlRegs.GPBMUX2.bit.GPIO49  =  GPIO;                // GPIO49   = DI_IP2
   GpioCtrlRegs.GPBDIR.bit.GPIO49   =  INPUT;               // DI_IP2
// GpioDataRegs.GPBDAT.bit.GPIO49   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-50 - PIN FUNCTION = GPIO
   GpioCtrlRegs.GPBMUX2.bit.GPIO50  =  GPIO;                // GPIO50   = DI_IP3
   GpioCtrlRegs.GPBDIR.bit.GPIO50   =  INPUT;               // DI_IP3
// GpioDataRegs.GPBDAT.bit.GPIO50   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-51 - PIN FUNCTION = GPIO
   GpioCtrlRegs.GPBMUX2.bit.GPIO51  =  GPIO;                // GPIO51   = DI_IP4
   GpioCtrlRegs.GPBDIR.bit.GPIO51   =  INPUT;               // DI_IP4
// GpioDataRegs.GPBDAT.bit.GPIO51   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-52 - PIN FUNCTION = GPIO
   GpioCtrlRegs.GPBMUX2.bit.GPIO52  =  GPIO;                // GPIO52   = DI_IP5
   GpioCtrlRegs.GPBDIR.bit.GPIO52   =  INPUT;               // DI_IP5
// GpioDataRegs.GPBDAT.bit.GPIO52   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-53 - PIN FUNCTION = GPIO
   GpioCtrlRegs.GPBMUX2.bit.GPIO53  =  GPIO;                // GPIO53   = DI_IP6
   GpioCtrlRegs.GPBDIR.bit.GPIO53   =  INPUT;               // DI_IP6
// GpioDataRegs.GPBDAT.bit.GPIO53   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-54 - PIN FUNCTION = GPIO
   GpioCtrlRegs.GPBMUX2.bit.GPIO54  =  GPIO;                // GPIO54   = DI_IP7
   GpioCtrlRegs.GPBDIR.bit.GPIO54   =  INPUT;               // DI_IP7
// GpioDataRegs.GPBDAT.bit.GPIO54   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-55 - PIN FUNCTION = GPIO
   GpioCtrlRegs.GPBMUX2.bit.GPIO55  =  GPIO;                // GPIO55   = DI_IP8
   GpioCtrlRegs.GPBDIR.bit.GPIO55   =  INPUT;               // DI_IP8
// GpioDataRegs.GPBDAT.bit.GPIO55   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-56 - PIN FUNCTION = GPIO
   GpioCtrlRegs.GPBMUX2.bit.GPIO56  =  GPIO;                // GPIO56   = DI_IP9
   GpioCtrlRegs.GPBDIR.bit.GPIO56   =  INPUT;               // DI_IP9
// GpioDataRegs.GPBDAT.bit.GPIO56   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-57 - PIN FUNCTION = GPIO
   GpioCtrlRegs.GPBMUX2.bit.GPIO57  =  GPIO;                // GPIO57   = DI_IP10
   GpioCtrlRegs.GPBDIR.bit.GPIO57   =  INPUT;               // DI_IP10
// GpioDataRegs.GPBDAT.bit.GPIO57   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-58 - PIN FUNCTION = GPIO
   GpioCtrlRegs.GPBMUX2.bit.GPIO58  =  GPIO;                // GPIO58   = PTC_REL_CTRL
   GpioCtrlRegs.GPBDIR.bit.GPIO58   =  INPUT;              // PTC_REL_CTRL
  // GpioDataRegs.GPBDAT.bit.GPIO58   =  GPIO_SET;         This Pin is not used.  // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-59 - PIN FUNCTION = GPIO
   GpioCtrlRegs.GPBMUX2.bit.GPIO59  =  GPIO;                // GPIO59   = DO_OP3
   GpioCtrlRegs.GPBDIR.bit.GPIO59   =  OUTPUT;              // DO_OP1
   GpioDataRegs.GPBDAT.bit.GPIO59   =  GPIO_SET;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-60 - PIN FUNCTION = GPIO
   GpioCtrlRegs.GPBMUX2.bit.GPIO60  =  GPIO;                // GPIO60   = DO_OP4
   GpioCtrlRegs.GPBDIR.bit.GPIO60   =  OUTPUT;              // DO_OP2
   GpioDataRegs.GPBDAT.bit.GPIO60   =  GPIO_SET;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-61 - PIN FUNCTION = GPIO
   GpioCtrlRegs.GPBMUX2.bit.GPIO61  =  GPIO;                // GPIO61   = RFI Relay
   GpioCtrlRegs.GPBDIR.bit.GPIO61   =  INPUT;              // NT Used
  // GpioDataRegs.GPBDAT.bit.GPIO61   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-62 - PIN FUNCTION = SCIRXDC
   GpioCtrlRegs.GPBMUX2.bit.GPIO62  =  PERIPHERAL_1;        // GPIO62   = SCIRXDC
// GpioCtrlRegs.GPBDIR.bit.GPIO62   =  OUTPUT;              // SCIRXDC
// GpioDataRegs.GPBDAT.bit.GPIO62   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-63 - PIN FUNCTION = SCITXDC
   GpioCtrlRegs.GPBMUX2.bit.GPIO63  =  PERIPHERAL_1;        // GPIO62   = SCITXDC
// GpioCtrlRegs.GPBDIR.bit.GPIO63   =  OUTPUT;              // SCITXDC
// GpioDataRegs.GPBDAT.bit.GPIO63   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//-------------------------------------------------------------------------------------------------------------------
// GPIO PortC MUX1 Register
//-------------------------------------------------------------------------------------------------------------------
// GPIO-64 - PIN FUNCTION = XD15
   GpioCtrlRegs.GPCMUX1.bit.GPIO64  =  PERIPHERAL_2;        // GPIO64   = XD15
// GpioCtrlRegs.GPCDIR.bit.GPIO64   =  OUTPUT;              // XD15
// GpioDataRegs.GPCDAT.bit.GPIO64   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-65 - PIN FUNCTION = XD14
   GpioCtrlRegs.GPCMUX1.bit.GPIO65  =  PERIPHERAL_2;        // GPIO65   = XD14
// GpioCtrlRegs.GPCDIR.bit.GPIO65   =  OUTPUT;              // XD14
// GpioDataRegs.GPCDAT.bit.GPIO65   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-66 - PIN FUNCTION = XD13
   GpioCtrlRegs.GPCMUX1.bit.GPIO66  =  PERIPHERAL_2;        // GPIO66   = XD13
// GpioCtrlRegs.GPCDIR.bit.GPIO66   =  OUTPUT;              // XD13
// GpioDataRegs.GPCDAT.bit.GPIO66   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-67 - PIN FUNCTION = XD12
  GpioCtrlRegs.GPCMUX1.bit.GPIO67   =  PERIPHERAL_2;        // GPIO67   = XD12
// GpioCtrlRegs.GPCDIR.bit.GPIO67   =  OUTPUT;              // XD12
// GpioDataRegs.GPCDAT.bit.GPIO67   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-68 - PIN FUNCTION = XD11
   GpioCtrlRegs.GPCMUX1.bit.GPIO68  =  PERIPHERAL_2;        // GPIO68   = XD11
// GpioCtrlRegs.GPCDIR.bit.GPIO68   =  OUTPUT;              // XD11
// GpioDataRegs.GPCDAT.bit.GPIO68   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-69 - PIN FUNCTION = XD10
   GpioCtrlRegs.GPCMUX1.bit.GPIO69  =  PERIPHERAL_2;        // GPIO69   = XD10
// GpioCtrlRegs.GPCDIR.bit.GPIO69   =  OUTPUT;              // XD10
// GpioDataRegs.GPCDAT.bit.GPIO69   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-70 - PIN FUNCTION = XD9
   GpioCtrlRegs.GPCMUX1.bit.GPIO70  =  PERIPHERAL_2;        // GPIO70   = XD9
// GpioCtrlRegs.GPCDIR.bit.GPIO70   =  OUTPUT;              // XD9
// GpioDataRegs.GPCDAT.bit.GPIO70   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-71 - PIN FUNCTION = XD8
   GpioCtrlRegs.GPCMUX1.bit.GPIO71  =  PERIPHERAL_2;        // GPIO71   = XD8
// GpioCtrlRegs.GPCDIR.bit.GPIO71   =  OUTPUT;              // XD8
// GpioDataRegs.GPCDAT.bit.GPIO71   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-72 - PIN FUNCTION = XD7
   GpioCtrlRegs.GPCMUX1.bit.GPIO72  =  PERIPHERAL_2;        // GPIO72   = XD7
// GpioCtrlRegs.GPCDIR.bit.GPIO72   =  OUTPUT;              // XD7
// GpioDataRegs.GPCDAT.bit.GPIO72   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-73 - PIN FUNCTION = XD6
   GpioCtrlRegs.GPCMUX1.bit.GPIO73  =  PERIPHERAL_2;        // GPIO73   = XD6
// GpioCtrlRegs.GPCDIR.bit.GPIO73   =  OUTPUT;              // XD6
// GpioDataRegs.GPCDAT.bit.GPIO73   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-74 - PIN FUNCTION = XD5
   GpioCtrlRegs.GPCMUX1.bit.GPIO74  =  PERIPHERAL_2;        // GPIO74   = XD5
// GpioCtrlRegs.GPCDIR.bit.GPIO74   =  OUTPUT;              // XD5
// GpioDataRegs.GPCDAT.bit.GPIO74   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-75 - PIN FUNCTION = XD4
   GpioCtrlRegs.GPCMUX1.bit.GPIO75  =  PERIPHERAL_2;        // GPIO75   = XD4
// GpioCtrlRegs.GPCDIR.bit.GPIO75   =  OUTPUT;              // XD4
// GpioDataRegs.GPCDAT.bit.GPIO75   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-76 - PIN FUNCTION = XD3
   GpioCtrlRegs.GPCMUX1.bit.GPIO76  =  PERIPHERAL_2;        // GPIO76   = XD3
// GpioCtrlRegs.GPCDIR.bit.GPIO76   =  OUTPUT;              // XD3
// GpioDataRegs.GPCDAT.bit.GPIO76   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-77 - PIN FUNCTION = XD2
   GpioCtrlRegs.GPCMUX1.bit.GPIO77  =  PERIPHERAL_2;        // GPIO77   = XD2
// GpioCtrlRegs.GPCDIR.bit.GPIO77   =  OUTPUT;              // XD2
// GpioDataRegs.GPCDAT.bit.GPIO77   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-78 - PIN FUNCTION = XD1
   GpioCtrlRegs.GPCMUX1.bit.GPIO78  =  PERIPHERAL_2;        // GPIO78   = XD1
// GpioCtrlRegs.GPCDIR.bit.GPIO78   =  OUTPUT;              // XD1
// GpioDataRegs.GPCDAT.bit.GPIO78   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-79 - PIN FUNCTION = XD0
   GpioCtrlRegs.GPCMUX1.bit.GPIO79  =  PERIPHERAL_2;        // GPIO79   = XD0
// GpioCtrlRegs.GPCDIR.bit.GPIO79   =  OUTPUT;              // XD0
// GpioDataRegs.GPCDAT.bit.GPIO79   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//-------------------------------------------------------------------------------------------------------------------
// GPIO PortC MUX2 Register
//-------------------------------------------------------------------------------------------------------------------
// GPIO-80 - PIN FUNCTION = XA8
   GpioCtrlRegs.GPCMUX2.bit.GPIO80  =  PERIPHERAL_2;        // GPIO80   = XA8
// GpioCtrlRegs.GPCDIR.bit.GPIO80   =  OUTPUT;              // XA8
// GpioDataRegs.GPCDAT.bit.GPIO80   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-81 - PIN FUNCTION = XA9
   GpioCtrlRegs.GPCMUX2.bit.GPIO81  =  PERIPHERAL_2;        // GPIO81   = XA9
// GpioCtrlRegs.GPCDIR.bit.GPIO81   =  OUTPUT;              // XA9
// GpioDataRegs.GPCDAT.bit.GPIO81   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-82 - PIN FUNCTION = XA10
   GpioCtrlRegs.GPCMUX2.bit.GPIO82  =  PERIPHERAL_2;        // GPIO82   = XA10
// GpioCtrlRegs.GPCDIR.bit.GPIO82   =  OUTPUT;              // XA10
// GpioDataRegs.GPCDAT.bit.GPIO82   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-83 - PIN FUNCTION = XA11
   GpioCtrlRegs.GPCMUX2.bit.GPIO83  =  PERIPHERAL_2;        // GPIO83   = XA11
// GpioCtrlRegs.GPCDIR.bit.GPIO83   =  OUTPUT;              // XA11
// GpioDataRegs.GPCDAT.bit.GPIO83   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-84 - PIN FUNCTION = XA12
   GpioCtrlRegs.GPCMUX2.bit.GPIO84  =  PERIPHERAL_2;        // GPIO84   = XA12
// GpioCtrlRegs.GPCDIR.bit.GPIO84   =  OUTPUT;              // XA12
// GpioDataRegs.GPCDAT.bit.GPIO84   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-85 - PIN FUNCTION = XA13
   GpioCtrlRegs.GPCMUX2.bit.GPIO85  =  PERIPHERAL_2;        // GPIO85   = XA13
// GpioCtrlRegs.GPCDIR.bit.GPIO85   =  OUTPUT;              // XA13
// GpioDataRegs.GPCDAT.bit.GPIO85   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-86 - PIN FUNCTION = XA14
   GpioCtrlRegs.GPCMUX2.bit.GPIO86  =  PERIPHERAL_2;        // GPIO86   = XA14
// GpioCtrlRegs.GPCDIR.bit.GPIO86   =  OUTPUT;              // XA14
// GpioDataRegs.GPCDAT.bit.GPIO86   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
// GPIO-87 - PIN FUNCTION = XA15
   GpioCtrlRegs.GPCMUX2.bit.GPIO87  =  PERIPHERAL_2;        // GPIO87   = XA15
// GpioCtrlRegs.GPCDIR.bit.GPIO87   =  OUTPUT;              // XA15
// GpioDataRegs.GPCDAT.bit.GPIO87   =  GPIO_CLEAR;          // SET STATE OF PIN AS HIGH OR LOW, IF OUT
//--------------------------------------------------------------------------------------
   GpioCtrlRegs.GPBPUD.bit.GPIO32 = 0;         // Enable pull-up for GPIO32 (SDA)
   GpioCtrlRegs.GPBPUD.bit.GPIO33 = 0;         // Enable pull-up for GPIO33 (SCL)

   GpioCtrlRegs.GPAQSEL2.bit.GPIO16 = 3; // Asynch input GPIO16 (SPISIMOA)
   GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 3; // Asynch input GPIO17 (SPISOMIA)
   GpioCtrlRegs.GPAQSEL2.bit.GPIO18 = 3; // Asynch input GPIO18 (SPICLKA)
   GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = 3; // Asynch input GPIO19 (SPISTEA)

   GpioCtrlRegs.GPAQSEL1.bit.GPIO12  = 10 ; // Trip Zone sampling is increased.



 //GPIO CONFIGURATION BITALL
/*
   GpioCtrlRegs.GPAMUX1.all =  0x01500555;
   GpioCtrlRegs.GPAMUX2.all =  0x0500F015;
   GpioCtrlRegs.GPADIR.all  =  0x0F38E200;
   GpioDataRegs.GPADAT.all  =  0xFCC7D1C0;

   GpioCtrlRegs.GPBMUX1.all =  0xAAAA8B00;
   GpioCtrlRegs.GPBMUX2.all =  0x50000000;
   GpioCtrlRegs.GPBDIR.all  =  0x3C000043;
   GpioDataRegs.GPBDAT.all  =  0xDFFF013C;

   GpioCtrlRegs.GPCMUX1.all =  0xAAAAAAAA;
   GpioCtrlRegs.GPCMUX2.all =  0x0000AAAA;
   GpioCtrlRegs.GPCDIR.all  =  0x00000000;
   GpioDataRegs.GPCDAT.all  =  0x0000FFFF;

*/


   EDIS;                                                    // Disable register access
}
//====================================================================================================================
// Function to Initialise Inverter PWM control registers
//====================================================================================================================
void InitInverterEPwm()
{

    EALLOW;                                                 // Allow access to Protected Registers
//====================================================================================================================
// Function to Initialise the EPwm control registers
//   - Timer Control Registers
//   - Compare Control Registers
//   - Action Control Registers
//   - Deadband Control Registers
//   - PWM Chopper Control Registers
//====================================================================================================================

/* PWM CONFIGURATION BITWISE

//-------------------------------------------------------------------------------------------------------------------
//  PWM 1 Configuration
//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
// Time-Base Control Register [TBCTL]
//-------------------------------------------------------------------------------------------------------------------
    EPwm1Regs.TBCTL.bit.FREE_SOFT = TB_FREE_RUN;            // Emulation Halt Behaviour,    [Free Run]
    EPwm1Regs.TBCTL.bit.PHSDIR    = TB_DOWN;                // Phase Direction Bit,         [Count down after Sync]
    EPwm1Regs.TBCTL.bit.CLKDIV    = TB_DIV1;                // TB Clock Prescale,           [/1]    TBCLK=SYSCLKOUT/(HSPCLKDIV×CLKDIV)
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;                // High Speed TB Clock Prescale, [/1]   TBCLK=SYSCLKOUT/(HSPCLKDIV×CLKDIV)
    EPwm1Regs.TBCTL.bit.SWFSYNC   = DISABLE;                // Software Forced Sync Pulse,  [Disabled]
    EPwm1Regs.TBCTL.bit.SYNCOSEL  = TB_CTR_ZERO;            // Sync Output Select, Source of the EPWMxSYNCO signal, [CTR=0]
    EPwm1Regs.TBCTL.bit.PRDLD     = TB_IMMEDIATE;           // Active Period Register Load From Shadow Register,    [Immediately]
    EPwm1Regs.TBCTL.bit.PHSEN     = TB_DISABLE;             // Counter Register Load From Phase Register,   [Disabled]
    EPwm1Regs.TBCTL.bit.CTRMODE   = TB_COUNT_UPDOWN;        // Counter Mode,    [Up Down Mode]

    EPwm1Regs.TBPRD               = 0x4E70;                 // Time Base Period Register. TBPRD = Tprd/(2*TBCLK) = (1/3KHz)/(2/120MHz) = 0x4E70
    EPwm1Regs.TBPHS.all           = 0x0000;                 // Time-Base Phase Register.
    EPwm1Regs.TBCTR               = 0x0001;                 // Time base Counter Register
//-------------------------------------------------------------------------------------------------------------------
// Compare Control Register [CMPCTL]
//-------------------------------------------------------------------------------------------------------------------
    EPwm1Regs.CMPCTL.bit.SHDWBMODE  = CC_SHADOW;            // CMPB Register Operating Mode,    [Shadow mode, double buffer]
    EPwm1Regs.CMPCTL.bit.SHDWAMODE  = CC_SHADOW;            // CMPA Register Operating Mode,    [Shadow mode, double buffer]
    EPwm1Regs.CMPCTL.bit.LOADBMODE  = CC_CTR_ZERO;          // CMPB Shadow Load Mode,           [Load on CTR = Zero]
    EPwm1Regs.CMPCTL.bit.LOADAMODE  = CC_CTR_ZERO;          // CMPA Shadow Load Mode,           [Load on CTR = Zero]

    EPwm1Regs.CMPA.half.CMPA        = 0x0000;               // Counter-Compare A Register.
    EPwm1Regs.CMPB                  = 0x0000;               // Counter-Compare B Register.
    EPwm1Regs.CMPA.half.CMPAHR      = 0x0000;               // High-resolution portion (LSB 8-bits) of the Counter-Compare A value.
//-------------------------------------------------------------------------------------------------------------------
// Action Control Register [AQCTLA]
//-------------------------------------------------------------------------------------------------------------------
    EPwm1Regs.AQCTLA.bit.CBD      = AQ_NO_ACTION;           // Action when CTR = CMPB on Down Count,    [Disabled]
    EPwm1Regs.AQCTLA.bit.CBU      = AQ_NO_ACTION;           // Action when CTR = CMPB on Up Count,      [Disabled]
    EPwm1Regs.AQCTLA.bit.CAD      = AQ_NO_ACTION;           // Action when CTR = CMPA on Down Count,    [Disabled]
    EPwm1Regs.AQCTLA.bit.CAU      = AQ_NO_ACTION;           // Action when CTR = CMPA on Up Count,      [Disabled]
    EPwm1Regs.AQCTLA.bit.PRD      = AQ_NO_ACTION;           // Action when CTR = PRD,                   [Clear]
    EPwm1Regs.AQCTLA.bit.ZRO      = AQ_NO_ACTION;           // Action when CTR = 0,                     [Set]
//-------------------------------------------------------------------------------------------------------------------
// Action Control Register [AQCTLB]
//-------------------------------------------------------------------------------------------------------------------
    EPwm1Regs.AQCTLB.bit.CBD      = AQ_NO_ACTION;           // Action when CTR = CMPB on Down Count,    [Disabled]
    EPwm1Regs.AQCTLB.bit.CBU      = AQ_NO_ACTION;           // Action when CTR = CMPB on Up Count,      [Disabled]
    EPwm1Regs.AQCTLB.bit.CAD      = AQ_NO_ACTION;           // Action when CTR = CMPA on Down Count,    [Disabled]
    EPwm1Regs.AQCTLB.bit.CAU      = AQ_NO_ACTION;           // Action when CTR = CMPA on Up Count,      [Disabled]
    EPwm1Regs.AQCTLB.bit.PRD      = AQ_NO_ACTION;           // Action when CTR = PRD,                   [Set]
    EPwm1Regs.AQCTLB.bit.ZRO      = AQ_NO_ACTION;           // Action when CTR = 0,                     [Clear]
//-------------------------------------------------------------------------------------------------------------------
//  Action-Qualifier Software Force Register [AQSFRC]
//-------------------------------------------------------------------------------------------------------------------
    EPwm1Regs.AQSFRC.bit.RLDCSF   = AQ_IMMEDIATE;           // AQCSFRC Shadow Reload Options            [Load immediately]
    EPwm1Regs.AQSFRC.bit.OTSFB    = AQ_DISABLED;            // One-Time S/W Forced Event on Output B    [No Action]
    EPwm1Regs.AQSFRC.bit.ACTSFB   = AQ_NO_ACTION;           // Action when One-Time S/W Force on B      [Disabled]
    EPwm1Regs.AQSFRC.bit.OTSFA    = AQ_DISABLED;            // One-Time S/W Forced Event on Output A    [No Action]
    EPwm1Regs.AQSFRC.bit.ACTSFA   = AQ_NO_ACTION;           // Action when One-Time S/W Force on A      [Disabled]
//-------------------------------------------------------------------------------------------------------------------
//  Action-Qualifier Continuous Software Force Register [AQCSFRC]
//-------------------------------------------------------------------------------------------------------------------
    EPwm1Regs.AQCSFRC.bit.CSFB    = AQ_FRC_DISABLED;        // Continuous S/W Force on Output B     [Forcing disabled]
    EPwm1Regs.AQCSFRC.bit.CSFA    = AQ_FRC_DISABLED;        // Continuous S/W Force on Output A     [Forcing disabled]
//-------------------------------------------------------------------------------------------------------------------
//  Dead-Band Generator Control Register[DBCTL]
//-------------------------------------------------------------------------------------------------------------------
    EPwm1Regs.DBCTL.bit.IN_MODE   = DBA_RED_FED;            // In Mode Control,         [PWMxA is source for FED and RED]
    EPwm1Regs.DBCTL.bit.POLSEL    = DB_ACTV_HI;             // Polarity Select Control, [Active Low Complementary]
    EPwm1Regs.DBCTL.bit.OUT_MODE  = DB_DISABLE;             // Out Mode Control,        [FED and RED]
//-------------------------------------------------------------------------------------------------------------------
    EPwm1Regs.DBRED               = 0x0350;                 // DB Gen RED, 10-bit [7us/(1/120MHz) = 0x0350]
    EPwm1Regs.DBFED               = 0x0350;                 // DB Gen FED, 10-bit [7us/(1/120MHz) = 0x0350]
//-------------------------------------------------------------------------------------------------------------------
//  PWM Chopper Control Register[CCTL]
//-------------------------------------------------------------------------------------------------------------------
    EPwm1Regs.PCCTL.all           = 0x0000;                 // PWM-Chopper Submodule Control Register
//-------------------------------------------------------------------------------------------------------------------
//  Trip Zone Control Register[TZSEL]
//-------------------------------------------------------------------------------------------------------------------
    EPwm1Regs.TZSEL.bit.OSHT1     = TZ_ENABLE;              // Trip-zone 1 (TZ1) Select, [OST source Enabled]
    EPwm1Regs.TZSEL.bit.OSHT2     = TZ_DISABLE;             // Trip-zone 2 (TZ1) Select, [OST source Enabled]
    EPwm1Regs.TZSEL.bit.OSHT3     = TZ_DISABLE;             // Trip-zone 3 (TZ1) Select, [OST source Enabled]
    EPwm1Regs.TZSEL.bit.OSHT4     = TZ_DISABLE;             // Trip-zone 4 (TZ1) Select, [OST source Enabled]
    EPwm1Regs.TZSEL.bit.OSHT5     = TZ_DISABLE;             // Trip-zone 5 (TZ1) Select, [OST source Enabled]
    EPwm1Regs.TZSEL.bit.OSHT6     = TZ_DISABLE;             // Trip-zone 6 (TZ1) Select, [OST source Enabled]

    EPwm1Regs.TZSEL.bit.CBC1      = TZ_DISABLE;             // Trip-zone 1 (TZ1) Select, [CBC source Disabled]
    EPwm1Regs.TZSEL.bit.CBC2      = TZ_DISABLE;             // Trip-zone 2 (TZ2) Select, [CBC source Disabled]
    EPwm1Regs.TZSEL.bit.CBC3      = TZ_DISABLE;             // Trip-zone 3 (TZ2) Select, [CBC source Disabled]
    EPwm1Regs.TZSEL.bit.CBC4      = TZ_DISABLE;             // Trip-zone 4 (TZ2) Select, [CBC source Disabled]
    EPwm1Regs.TZSEL.bit.CBC5      = TZ_DISABLE;             // Trip-zone 5 (TZ2) Select, [CBC source Disabled]
    EPwm1Regs.TZSEL.bit.CBC6      = TZ_DISABLE;             // Trip-zone 6 (TZ2) Select, [CBC source Disabled]
//-------------------------------------------------------------------------------------------------------------------
//  Trip-Zone Control Register[TZCTL]
//-------------------------------------------------------------------------------------------------------------------
     EPwm1Regs.TZCTL.bit.TZB = TZ_FORCE_LO;
     EPwm1Regs.TZCTL.bit.TZA = TZ_FORCE_LO;

    EPwm1Regs.TZCTL.bit.TZB       = TZ_FORCE_HI;            // Trip Event Action, [Force EPWMxB to a High State]
    EPwm1Regs.TZCTL.bit.TZA       = TZ_FORCE_HI;            // Trip Event Action, [Force EPWMxA to a High State]
//-------------------------------------------------------------------------------------------------------------------
//  Trip-Zone Enable Interrupt Register[TXEINT]
//-------------------------------------------------------------------------------------------------------------------
    EPwm1Regs.TZEINT.bit.OST      = TZ_ENABLE;              // Trip-zone OST Interrupt EPWMx_TZINT PIE, [Enabled]
    EPwm1Regs.TZEINT.bit.CBC      = TZ_DISABLE;             // Trip Zone CBC Interrupt EPWMx_TZINT PIE, [Disabled]
//-------------------------------------------------------------------------------------------------------------------
//  Trip-Zone Clear Register[TZCLR]
//-------------------------------------------------------------------------------------------------------------------
    EPwm1Regs.TZCLR.bit.OST     = TZ_CLEAR;     // Clear Flag for OST Latch,        [Clear]
    EPwm1Regs.TZCLR.bit.CBC     = TZ_CLEAR;     // Clear Flag for CBC Trip Latch,   [Clear]
    EPwm1Regs.TZCLR.bit.INT     = TZ_CLEAR;     // Global Interrupt Clear Flag,     [Clear]
//                                              //   No further EPWMx_TZINT PIE interrupts will be generated until the flag is cleared.
//-------------------------------------------------------------------------------------------------------------------
//  Trip-Zone Force Register[TZFRC]
//-------------------------------------------------------------------------------------------------------------------
    EPwm1Regs.TZFRC.bit.OST     = TZ_DISABLE;   // Force a CBC Trip Event via S/W,  [Disabled]
    EPwm1Regs.TZFRC.bit.CBC     = TZ_DISABLE;   // Force a OST Event via S/W,       [Disabled]
//-------------------------------------------------------------------------------------------------------------------
//  Event Trigger Selection Register[ETSEL]
//-------------------------------------------------------------------------------------------------------------------
    EPwm1Regs.ETSEL.bit.SOCBEN    = ET_DISABLE;             // EPWMxSOCB,        [Disabled]
    EPwm1Regs.ETSEL.bit.SOCBSEL   = ET_CTR_ZERO;            // EPWMxSOCB Select, [CTR=0]
    EPwm1Regs.ETSEL.bit.SOCAEN    = ET_DISABLE;             // EPWMxSOCA,        [Disabled]
    EPwm1Regs.ETSEL.bit.SOCASEL   = ET_CTR_ZERO;            // EPWMxSOCA Select, [CTR=0]
    EPwm1Regs.ETSEL.bit.INTEN     = ET_ENABLE;              // EPWMxINT          [Enabled]
    EPwm1Regs.ETSEL.bit.INTSEL    = ET_CTR_ZERO;            // EPWMxINT Select,  [CTR=0]
//-------------------------------------------------------------------------------------------------------------------
//  Event Trigger Prescale Register[ETPS]
//-------------------------------------------------------------------------------------------------------------------
    EPwm1Regs.ETPS.bit.SOCBCNT   = ET_DISABLE;              // EPWMxSOCB Counter, [Disabled]
    EPwm1Regs.ETPS.bit.SOCBPRD   = ET_DISABLE;              // EPWMxSOCB Period,  [Disabled]
    EPwm1Regs.ETPS.bit.SOCACNT   = ET_DISABLE;              // EPWMxSOCA Counter, [Disabled]
    EPwm1Regs.ETPS.bit.SOCAPRD   = ET_DISABLE;              // EPWMxSOCA Period,  [Disabled]
    EPwm1Regs.ETPS.bit.INTCNT    = ET_DISABLE;              // EPWMxINT Counter,  [Disabled]
    EPwm1Regs.ETPS.bit.INTPRD    = ET_1ST;                  // EPWMxINT Period,   [1st Event]
//-------------------------------------------------------------------------------------------------------------------
//  PWM 2 Configuration
//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
// Time-Base Control Register [TBCTL]
//-------------------------------------------------------------------------------------------------------------------
    EPwm2Regs.TBCTL.bit.FREE_SOFT = TB_FREE_RUN;            // Emulation Halt Behaviour,     [Free Run]
    EPwm2Regs.TBCTL.bit.PHSDIR    = TB_DOWN;                // Phase Direction Bit,          [Count down after Sync]
    EPwm2Regs.TBCTL.bit.CLKDIV    = TB_DIV1;                // TB Clock Prescale,            [/1]   TBCLK=SYSCLKOUT/(HSPCLKDIV×CLKDIV)
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;                // High Speed TB Clock Prescale, [/1]   TBCLK=SYSCLKOUT/(HSPCLKDIV×CLKDIV)
    EPwm2Regs.TBCTL.bit.SWFSYNC   = DISABLE;                // Software Forced Sync Pulse,   [Disabled]
    EPwm2Regs.TBCTL.bit.SYNCOSEL  = TB_CTR_ZERO;            // Sync Output Select, Source of the EPWMxSYNCO signal, [CTR=0]
    EPwm2Regs.TBCTL.bit.PRDLD     = TB_IMMEDIATE;           // Active Period Register Load From Shadow Register,    [Immediately]
    EPwm2Regs.TBCTL.bit.PHSEN     = TB_DISABLE;             // Counter Register Load From Phase Register,   [Disabled]
    EPwm2Regs.TBCTL.bit.CTRMODE   = TB_COUNT_UPDOWN;        // Counter Mode,    [Up Down Mode]


    EPwm2Regs.TBPRD               = 0x4E70;                 // Time Base Period Register. TBPRD = Tprd/(2*TBCLK) = (1/3KHz)/(2/120MHz) = 0x4E70
    EPwm2Regs.TBPHS.all           = 0x0000;                 // Time-Base Phase Register.
    EPwm2Regs.TBCTR               = 0x0001;                 // Time base Counter Register
//-------------------------------------------------------------------------------------------------------------------
// Compare Control Register [CMPCTL]
//-------------------------------------------------------------------------------------------------------------------
    EPwm2Regs.CMPCTL.bit.SHDWBMODE  = CC_SHADOW;            // CMPB Register Operating Mode,    [Shadow mode, double buffer]
    EPwm2Regs.CMPCTL.bit.SHDWAMODE  = CC_SHADOW;            // CMPA Register Operating Mode,    [Shadow mode, double buffer]
    EPwm2Regs.CMPCTL.bit.LOADBMODE  = CC_CTR_ZERO;          // CMPB Shadow Load Mode,           [Load on CTR = Zero]
    EPwm2Regs.CMPCTL.bit.LOADAMODE  = CC_CTR_ZERO;          // CMPA Shadow Load Mode,           [Load on CTR = Zero]

    EPwm2Regs.CMPA.half.CMPA        = 0x0000;               // Counter-Compare A Register.
    EPwm2Regs.CMPB                  = 0x0000;               // Counter-Compare B Register.
    EPwm2Regs.CMPA.half.CMPAHR      = 0x0000;               // High-resolution portion (LSB 8-bits) of the Counter-Compare A value.
//-------------------------------------------------------------------------------------------------------------------
// Action Control Register [AQCTLA]
//-------------------------------------------------------------------------------------------------------------------
    EPwm2Regs.AQCTLA.bit.CBD      = AQ_NO_ACTION;           // Action when CTR = CMPB on Down Count,    [Disabled]
    EPwm2Regs.AQCTLA.bit.CBU      = AQ_NO_ACTION;           // Action when CTR = CMPB on Up Count,      [Disabled]
    EPwm2Regs.AQCTLA.bit.CAD      = AQ_NO_ACTION;           // Action when CTR = CMPA on Down Count,    [Disabled]
    EPwm2Regs.AQCTLA.bit.CAU      = AQ_NO_ACTION;           // Action when CTR = CMPA on Up Count,      [Disabled]
    EPwm2Regs.AQCTLA.bit.PRD      = AQ_NO_ACTION;           // Action when CTR = PRD,                   [Clear]
    EPwm2Regs.AQCTLA.bit.ZRO      = AQ_NO_ACTION;           // Action when CTR = 0,                     [Set]
//-------------------------------------------------------------------------------------------------------------------
// Action Control Register [AQCTLB]
//-------------------------------------------------------------------------------------------------------------------
    EPwm2Regs.AQCTLB.bit.CBD      = AQ_NO_ACTION;           // Action when CTR = CMPB on Down Count,    [Disabled]
    EPwm2Regs.AQCTLB.bit.CBU      = AQ_NO_ACTION;           // Action when CTR = CMPB on Up Count,      [Disabled]
    EPwm2Regs.AQCTLB.bit.CAD      = AQ_NO_ACTION;           // Action when CTR = CMPA on Down Count,    [Disabled]
    EPwm2Regs.AQCTLB.bit.CAU      = AQ_NO_ACTION;           // Action when CTR = CMPA on Up Count,      [Disabled]
    EPwm2Regs.AQCTLB.bit.PRD      = AQ_NO_ACTION;           // Action when CTR = PRD,                   [Set]
    EPwm2Regs.AQCTLB.bit.ZRO      = AQ_NO_ACTION;           // Action when CTR = 0,                     [Clear]
//-------------------------------------------------------------------------------------------------------------------
//  Action-Qualifier Software Force Register [AQSFRC]
//-------------------------------------------------------------------------------------------------------------------
    EPwm2Regs.AQSFRC.bit.RLDCSF   = AQ_IMMEDIATE;           // AQCSFRC Shadow Reload Options            [Load immediately]
    EPwm2Regs.AQSFRC.bit.OTSFB    = AQ_DISABLED;            // One-Time S/W Forced Event on Output B    [No Action]
    EPwm2Regs.AQSFRC.bit.ACTSFB   = AQ_NO_ACTION;           // Action when One-Time S/W Force on B      [Disabled]
    EPwm2Regs.AQSFRC.bit.OTSFA    = AQ_DISABLED;            // One-Time S/W Forced Event on Output A    [No Action]
    EPwm2Regs.AQSFRC.bit.ACTSFA   = AQ_NO_ACTION;           // Action when One-Time S/W Force on A      [Disabled]
//-------------------------------------------------------------------------------------------------------------------
//  Action-Qualifier Continuous Software Force Register [AQCSFRC]
//-------------------------------------------------------------------------------------------------------------------
    EPwm2Regs.AQCSFRC.bit.CSFB    = AQ_FRC_DISABLED;        // Continuous S/W Force on Output B     [Forcing disabled]
    EPwm2Regs.AQCSFRC.bit.CSFA    = AQ_FRC_DISABLED;        // Continuous S/W Force on Output A     [Forcing disabled]
//-------------------------------------------------------------------------------------------------------------------
//  Dead-Band Generator Control Register[DBCTL]
//-------------------------------------------------------------------------------------------------------------------
    EPwm2Regs.DBCTL.bit.IN_MODE   = DBA_RED_FED;            // In Mode Control,         [PWMxA is source for FED and RED]
    EPwm2Regs.DBCTL.bit.POLSEL    = DB_ACTV_HI;             // Polarity Select Control, [Active Low Complementary]
    EPwm2Regs.DBCTL.bit.OUT_MODE  = DB_DISABLE;             // Out Mode Control,        [FED and RED]
//-------------------------------------------------------------------------------------------------------------------
    EPwm2Regs.DBRED               = 0x0350;                 // DB Gen RED, 10-bit [7us/(1/120MHz) = 0x0350]
    EPwm2Regs.DBFED               = 0x0350;                 // DB Gen FED, 10-bit [7us/(1/120MHz) = 0x0350]
//-------------------------------------------------------------------------------------------------------------------
//  PWM Chopper Control Register[CCTL]
//-------------------------------------------------------------------------------------------------------------------
    EPwm2Regs.PCCTL.all           = 0x0000;                 // PWM-Chopper Submodule Control Register
//-------------------------------------------------------------------------------------------------------------------
//  Trip Zone Control Register[TZSEL]
//-------------------------------------------------------------------------------------------------------------------
    EPwm2Regs.TZSEL.bit.OSHT1   = TZ_ENABLE;    // Trip-zone 1 (TZ1) Select, [OST source Enabled]
    EPwm2Regs.TZSEL.bit.OSHT2   = TZ_DISABLE;   // Trip-zone 2 (TZ1) Select, [OST source Enabled]
    EPwm2Regs.TZSEL.bit.OSHT3   = TZ_DISABLE;   // Trip-zone 3 (TZ1) Select, [OST source Enabled]
    EPwm2Regs.TZSEL.bit.OSHT4   = TZ_DISABLE;   // Trip-zone 4 (TZ1) Select, [OST source Enabled]
    EPwm2Regs.TZSEL.bit.OSHT5   = TZ_DISABLE;   // Trip-zone 5 (TZ1) Select, [OST source Enabled]
    EPwm2Regs.TZSEL.bit.OSHT6   = TZ_DISABLE;   // Trip-zone 6 (TZ1) Select, [OST source Enabled]

    EPwm2Regs.TZSEL.bit.CBC1    = TZ_DISABLE;   // Trip-zone 1 (TZ1) Select, [CBC source Disabled]
    EPwm2Regs.TZSEL.bit.CBC2    = TZ_DISABLE;   // Trip-zone 2 (TZ2) Select, [CBC source Disabled]
    EPwm2Regs.TZSEL.bit.CBC3    = TZ_DISABLE;   // Trip-zone 3 (TZ2) Select, [CBC source Disabled]
    EPwm2Regs.TZSEL.bit.CBC4    = TZ_DISABLE;   // Trip-zone 4 (TZ2) Select, [CBC source Disabled]
    EPwm2Regs.TZSEL.bit.CBC5    = TZ_DISABLE;   // Trip-zone 5 (TZ2) Select, [CBC source Disabled]
    EPwm2Regs.TZSEL.bit.CBC6    = TZ_DISABLE;   // Trip-zone 6 (TZ2) Select, [CBC source Disabled]
//-------------------------------------------------------------------------------------------------------------------
//  Trip-Zone Control Register[TZCTL]
//-------------------------------------------------------------------------------------------------------------------
     EPwm2Regs.TZCTL.bit.TZB = TZ_FORCE_LO;
     EPwm2Regs.TZCTL.bit.TZA = TZ_FORCE_LO;

    EPwm2Regs.TZCTL.bit.TZB     = TZ_FORCE_HI;  // Trip Event Action, [Force EPWMxB to a High State]
    EPwm2Regs.TZCTL.bit.TZA     = TZ_FORCE_HI;  // Trip Event Action, [Force EPWMxA to a High State]
//-------------------------------------------------------------------------------------------------------------------
//  Trip-Zone Enable Interrupt Register[TXEINT]
//-------------------------------------------------------------------------------------------------------------------
    EPwm2Regs.TZEINT.bit.OST    = TZ_ENABLE;    // Trip-zone OST Interrupt EPWMx_TZINT PIE, [Enabled]
    EPwm2Regs.TZEINT.bit.CBC    = TZ_DISABLE;   // Trip Zone CBC Interrupt EPWMx_TZINT PIE,  [Disabled]
//-------------------------------------------------------------------------------------------------------------------
//  Trip-Zone Clear Register[TZCLR]
//-------------------------------------------------------------------------------------------------------------------
    EPwm2Regs.TZCLR.bit.OST     = TZ_CLEAR;     // Clear Flag for OST Latch,        [Clear]
    EPwm2Regs.TZCLR.bit.CBC     = TZ_CLEAR;     // Clear Flag for CBC Trip Latch,   [Clear]
    EPwm2Regs.TZCLR.bit.INT     = TZ_CLEAR;     // Global Interrupt Clear Flag,     [Clear]
//                                              //   No further EPWMx_TZINT PIE interrupts will be generated until the flag is cleared.
//-------------------------------------------------------------------------------------------------------------------
//  Trip-Zone Force Register[TZFRC]
//-------------------------------------------------------------------------------------------------------------------
    EPwm2Regs.TZFRC.bit.OST     = TZ_DISABLE;   // Force a CBC Trip Event via S/W,  [Disabled]
    EPwm2Regs.TZFRC.bit.CBC     = TZ_DISABLE;   // Force a OST Event via S/W,       [Disabled]
//-------------------------------------------------------------------------------------------------------------------
//  PWM 3 Configuration
//-------------------------------------------------------------------------------------------------------------------                                                   // Allow access to Protected Registers
//-------------------------------------------------------------------------------------------------------------------
// Time-Base Control Register [TBCTL]
//-------------------------------------------------------------------------------------------------------------------
    EPwm3Regs.TBCTL.bit.FREE_SOFT = TB_FREE_RUN;            // Emulation Halt Behaviour,    [Free Run]
    EPwm3Regs.TBCTL.bit.PHSDIR    = TB_DOWN;                // Phase Direction Bit,         [Count down after Sync]
    EPwm3Regs.TBCTL.bit.CLKDIV    = TB_DIV1;                // TB Clock Prescale,           [/1]    TBCLK=SYSCLKOUT/(HSPCLKDIV×CLKDIV)
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;                // High Speed TB Clock Prescale, [/1]   TBCLK=SYSCLKOUT/(HSPCLKDIV×CLKDIV)
    EPwm3Regs.TBCTL.bit.SWFSYNC   = DISABLE;                // Software Forced Sync Pulse,  [Disabled]
    EPwm3Regs.TBCTL.bit.SYNCOSEL  = TB_CTR_ZERO;            // Sync Output Select, Source of the EPWMxSYNCO signal, [CTR=0]
    EPwm3Regs.TBCTL.bit.PRDLD     = TB_IMMEDIATE;           // Active Period Register Load From Shadow Register,    [Immediately]
    EPwm3Regs.TBCTL.bit.PHSEN     = TB_DISABLE;             // Counter Register Load From Phase Register,   [Disabled]
    EPwm3Regs.TBCTL.bit.CTRMODE   = TB_COUNT_UPDOWN;        // Counter Mode,    [Up Down Mode]

    EPwm3Regs.TBPRD               = 0x4E70;                 // Time Base Period Register. TBPRD = Tprd/(2*TBCLK) = (1/3KHz)/(2/120MHz) = 0x4E70
    EPwm3Regs.TBPHS.all           = 0x0000;                 // Time-Base Phase Register.
    EPwm3Regs.TBCTR               = 0x0001;                 // Time base Counter Register
//-------------------------------------------------------------------------------------------------------------------
// Compare Control Register [CMPCTL]
//-------------------------------------------------------------------------------------------------------------------
    EPwm3Regs.CMPCTL.bit.SHDWBMODE  = CC_SHADOW;            // CMPB Register Operating Mode,    [Shadow mode, double buffer]
    EPwm3Regs.CMPCTL.bit.SHDWAMODE  = CC_SHADOW;            // CMPA Register Operating Mode,    [Shadow mode, double buffer]
    EPwm3Regs.CMPCTL.bit.LOADBMODE  = CC_CTR_ZERO;          // CMPB Shadow Load Mode,           [Load on CTR = Zero]
    EPwm3Regs.CMPCTL.bit.LOADAMODE  = CC_CTR_ZERO;          // CMPA Shadow Load Mode,           [Load on CTR = Zero]

    EPwm3Regs.CMPA.half.CMPA        = 0x0000;               // Counter-Compare A Register.
    EPwm3Regs.CMPB                  = 0x0000;               // Counter-Compare B Register.
    EPwm3Regs.CMPA.half.CMPAHR      = 0x0000;               // High-resolution portion (LSB 8-bits) of the Counter-Compare A value.
//-------------------------------------------------------------------------------------------------------------------
// Action Control Register [AQCTLA]
//-------------------------------------------------------------------------------------------------------------------
    EPwm3Regs.AQCTLA.bit.CBD      = AQ_NO_ACTION;           // Action when CTR = CMPB on Down Count,    [Disabled]
    EPwm3Regs.AQCTLA.bit.CBU      = AQ_NO_ACTION;           // Action when CTR = CMPB on Up Count,      [Disabled]
    EPwm3Regs.AQCTLA.bit.CAD      = AQ_NO_ACTION;           // Action when CTR = CMPA on Down Count,    [Disabled]
    EPwm3Regs.AQCTLA.bit.CAU      = AQ_NO_ACTION;           // Action when CTR = CMPA on Up Count,      [Disabled]
    EPwm3Regs.AQCTLA.bit.PRD      = AQ_NO_ACTION;           // Action when CTR = PRD,                   [Clear]
    EPwm3Regs.AQCTLA.bit.ZRO      = AQ_NO_ACTION;           // Action when CTR = 0,                     [Set]
//-------------------------------------------------------------------------------------------------------------------
// Action Control Register [AQCTLB]
//-------------------------------------------------------------------------------------------------------------------
    EPwm3Regs.AQCTLB.bit.CBD      = AQ_NO_ACTION;           // Action when CTR = CMPB on Down Count,    [Disabled]
    EPwm3Regs.AQCTLB.bit.CBU      = AQ_NO_ACTION;           // Action when CTR = CMPB on Up Count,      [Disabled]
    EPwm3Regs.AQCTLB.bit.CAD      = AQ_NO_ACTION;           // Action when CTR = CMPA on Down Count,    [Disabled]
    EPwm3Regs.AQCTLB.bit.CAU      = AQ_NO_ACTION;           // Action when CTR = CMPA on Up Count,      [Disabled]
    EPwm3Regs.AQCTLB.bit.PRD      = AQ_NO_ACTION;           // Action when CTR = PRD,                   [Set]
    EPwm3Regs.AQCTLB.bit.ZRO      = AQ_NO_ACTION;           // Action when CTR = 0,                     [Clear]
//-------------------------------------------------------------------------------------------------------------------
//  Action-Qualifier Software Force Register [AQSFRC]
//-------------------------------------------------------------------------------------------------------------------
    EPwm3Regs.AQSFRC.bit.RLDCSF   = AQ_IMMEDIATE;           // AQCSFRC Shadow Reload Options            [Load immediately]
    EPwm3Regs.AQSFRC.bit.OTSFB    = AQ_DISABLED;            // One-Time S/W Forced Event on Output B    [No Action]
    EPwm3Regs.AQSFRC.bit.ACTSFB   = AQ_NO_ACTION;           // Action when One-Time S/W Force on B      [Disabled]
    EPwm3Regs.AQSFRC.bit.OTSFA    = AQ_DISABLED;            // One-Time S/W Forced Event on Output A    [No Action]
    EPwm3Regs.AQSFRC.bit.ACTSFA   = AQ_NO_ACTION;           // Action when One-Time S/W Force on A      [Disabled]
//-------------------------------------------------------------------------------------------------------------------
//  Action-Qualifier Continuous Software Force Register [AQCSFRC]
//-------------------------------------------------------------------------------------------------------------------
    EPwm3Regs.AQCSFRC.bit.CSFB    = AQ_FRC_DISABLED;        // Continuous S/W Force on Output B     [Forcing disabled]
    EPwm3Regs.AQCSFRC.bit.CSFA    = AQ_FRC_DISABLED;        // Continuous S/W Force on Output A     [Forcing disabled]
//-------------------------------------------------------------------------------------------------------------------
//  Dead-Band Generator Control Register[DBCTL]
//-------------------------------------------------------------------------------------------------------------------
    EPwm3Regs.DBCTL.bit.IN_MODE   = DBA_RED_FED;            // In Mode Control,         [PWMxA is source for FED and RED]
    EPwm3Regs.DBCTL.bit.POLSEL    = DB_ACTV_HI;             // Polarity Select Control, [Active Low Complementary]
    EPwm3Regs.DBCTL.bit.OUT_MODE  = DB_DISABLE;             // Out Mode Control,        [FED and RED]
//-------------------------------------------------------------------------------------------------------------------
    EPwm3Regs.DBRED               = 0x0350;                 // DB Gen RED, 10-bit [7us/(1/120MHz) = 0x0350]     RED = DBRED x TBCLK
    EPwm3Regs.DBFED               = 0x0350;                 // DB Gen FED, 10-bit [7us/(1/120MHz) = 0x0350]     FED = DBFED x TBCLK
//-------------------------------------------------------------------------------------------------------------------
//  PWM Chopper Control Register[DBCTL]
//-------------------------------------------------------------------------------------------------------------------
    EPwm3Regs.PCCTL.all           = 0x0000;                 // PWM-Chopper Submodule Control Register
//-------------------------------------------------------------------------------------------------------------------
//  Trip Zone Control Register[TZSEL]
//-------------------------------------------------------------------------------------------------------------------
    EPwm3Regs.TZSEL.bit.OSHT1   = TZ_ENABLE;    // Trip-zone 1 (TZ1) Select, [OST source Enabled]
    EPwm3Regs.TZSEL.bit.OSHT2   = TZ_DISABLE;   // Trip-zone 2 (TZ1) Select, [OST source Enabled]
    EPwm3Regs.TZSEL.bit.OSHT3   = TZ_DISABLE;   // Trip-zone 3 (TZ1) Select, [OST source Enabled]
    EPwm3Regs.TZSEL.bit.OSHT4   = TZ_DISABLE;   // Trip-zone 4 (TZ1) Select, [OST source Enabled]
    EPwm3Regs.TZSEL.bit.OSHT5   = TZ_DISABLE;   // Trip-zone 5 (TZ1) Select, [OST source Enabled]
    EPwm3Regs.TZSEL.bit.OSHT6   = TZ_DISABLE;   // Trip-zone 6 (TZ1) Select, [OST source Enabled]

    EPwm3Regs.TZSEL.bit.CBC1    = TZ_DISABLE;   // Trip-zone 1 (TZ1) Select, [CBC source Disabled]
    EPwm3Regs.TZSEL.bit.CBC2    = TZ_DISABLE;   // Trip-zone 2 (TZ2) Select, [CBC source Disabled]
    EPwm3Regs.TZSEL.bit.CBC3    = TZ_DISABLE;   // Trip-zone 3 (TZ2) Select, [CBC source Disabled]
    EPwm3Regs.TZSEL.bit.CBC4    = TZ_DISABLE;   // Trip-zone 4 (TZ2) Select, [CBC source Disabled]
    EPwm3Regs.TZSEL.bit.CBC5    = TZ_DISABLE;   // Trip-zone 5 (TZ2) Select, [CBC source Disabled]
    EPwm3Regs.TZSEL.bit.CBC6    = TZ_DISABLE;   // Trip-zone 6 (TZ2) Select, [CBC source Disabled]
//-------------------------------------------------------------------------------------------------------------------
//  Trip-Zone Control Register[TZCTL]
//-------------------------------------------------------------------------------------------------------------------
  EPwm3Regs.TZCTL.bit.TZB = TZ_FORCE_LO;
  EPwm3Regs.TZCTL.bit.TZA = TZ_FORCE_LO;


    EPwm3Regs.TZCTL.bit.TZB     = TZ_FORCE_HI;  // Trip Event Action, [Force EPWMxB to a High State]
    EPwm3Regs.TZCTL.bit.TZA     = TZ_FORCE_HI;  // Trip Event Action, [Force EPWMxA to a High State]
//-------------------------------------------------------------------------------------------------------------------
//  Trip-Zone Enable Interrupt Register[TXEINT]
//-------------------------------------------------------------------------------------------------------------------
    EPwm3Regs.TZEINT.bit.OST    = TZ_ENABLE;    // Trip-zone OST Interrupt EPWMx_TZINT PIE, [Enabled]
    EPwm3Regs.TZEINT.bit.CBC    = TZ_DISABLE;   // Trip Zone CBC Interrupt EPWMx_TZINT PIE,  [Disabled]
//-------------------------------------------------------------------------------------------------------------------
//  Trip-Zone Clear Register[TZCLR]
//-------------------------------------------------------------------------------------------------------------------
    EPwm3Regs.TZCLR.bit.OST     = TZ_CLEAR;     // Clear Flag for OST Latch,        [Clear]
    EPwm3Regs.TZCLR.bit.CBC     = TZ_CLEAR;     // Clear Flag for CBC Trip Latch,   [Clear]
    EPwm3Regs.TZCLR.bit.INT     = TZ_CLEAR;     // Global Interrupt Clear Flag,     [Clear]
//                                              //   No further EPWMx_TZINT PIE interrupts will be generated until the flag is cleared.
//-------------------------------------------------------------------------------------------------------------------
//  Trip-Zone Force Register[TZFRC]
//-------------------------------------------------------------------------------------------------------------------
    EPwm3Regs.TZFRC.bit.OST     = TZ_DISABLE;   // Force a CBC Trip Event via S/W,  [Disabled]
    EPwm3Regs.TZFRC.bit.CBC     = TZ_DISABLE;   // Force a OST Event via S/W,       [Disabled]
    //SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;

*/

// PWM CONFIGURATION BITALL
    // PWM 1
    VFD_Status.bits.StartFlag = 0; // Start Flag Initialised before PWM Interrupt
    EPwm1Regs.TBCTL.all = 0x401A;
    EPwm1Regs.TBPRD     = 0x4E70;
    EPwm1Regs.TBPHS.all = 0x0000;
    EPwm1Regs.TBCTR     = 0x0001;

    EPwm1Regs.CMPA.half.CMPA    = 0x0000;
    EPwm1Regs.CMPB              = 0x0000;
    EPwm1Regs.CMPCTL.all        = 0x0000;//0x0000;
    EPwm1Regs.CMPA.half.CMPAHR  = 0x0000;

    EPwm1Regs.AQCTLA.all        = 0x0000;
    EPwm1Regs.AQCTLB.all        = 0x0000;
    EPwm1Regs.AQCSFRC.all       = 0x000A;
    EPwm1Regs.AQSFRC.all        = 0x00C0;

    EPwm1Regs.DBCTL.all = 0x0000;
    EPwm1Regs.DBFED     = 0x0350;
    EPwm1Regs.DBRED     = 0x0350;

    EPwm1Regs.TZSEL.all     = 0x0100;
    EPwm1Regs.TZCTL.all     = 0x0005;
    EPwm1Regs.TZEINT.all    = 0x0004;
    EPwm1Regs.TZCLR.all     = 0x0000;
    EPwm1Regs.TZFRC.all     = 0x0000;
    EPwm1Regs.ETSEL.all     = 0x1109;
    EPwm1Regs.ETPS.all      = 0x0001;

    EPwm1Regs.PCCTL.all = 0x0000;


    // PWM 2
    EPwm2Regs.TBCTL.all = 0x401A;
    EPwm2Regs.TBPRD     = 0x4E70;
    EPwm2Regs.TBPHS.all = 0x0000;
    EPwm2Regs.TBCTR     = 0x0001;

    EPwm2Regs.CMPA.half.CMPA    = 0x0000;
    EPwm2Regs.CMPB              = 0x0000;
    EPwm2Regs.CMPCTL.all        = 0x0000;//0x0000;
    EPwm2Regs.CMPA.half.CMPAHR  = 0x0000;

    EPwm2Regs.AQCTLA.all        = 0x0000;
    EPwm2Regs.AQCTLB.all        = 0x0000;
    EPwm2Regs.AQCSFRC.all       = 0x000A;
    EPwm2Regs.AQSFRC.all        = 0x00C0;

    EPwm2Regs.DBCTL.all = 0x0000;
    EPwm2Regs.DBFED     = 0x0350;
    EPwm2Regs.DBRED     = 0x0350;

    EPwm2Regs.TZSEL.all     = 0x0100;
    EPwm2Regs.TZCTL.all     = 0x0005;
    EPwm2Regs.TZEINT.all    = 0x0004;
    EPwm2Regs.TZCLR.all     = 0x0000;
    EPwm2Regs.TZFRC.all     = 0x0000;
    EPwm2Regs.ETSEL.all     = 0x1109;
    EPwm2Regs.ETPS.all      = 0x0001;

    EPwm2Regs.PCCTL.all = 0x0000;


    // PWM 3
    EPwm3Regs.TBCTL.all = 0x401A;
    EPwm3Regs.TBPRD     = 0x4E70;
    EPwm3Regs.TBPHS.all = 0x0000;
    EPwm3Regs.TBCTR     = 0x0001;

    EPwm3Regs.CMPA.half.CMPA    = 0x0000;
    EPwm3Regs.CMPB              = 0x0000;
    EPwm3Regs.CMPCTL.all        = 0x0000;//0x0000; for multisampling immediate load is preferred..
    EPwm3Regs.CMPA.half.CMPAHR  = 0x0000;

    EPwm3Regs.AQCTLA.all        = 0x0000;
    EPwm3Regs.AQCTLB.all        = 0x0000;
    EPwm3Regs.AQCSFRC.all       = 0x000A;
    EPwm3Regs.AQSFRC.all        = 0x00C0;

    EPwm3Regs.DBCTL.all = 0x0000;
    EPwm3Regs.DBFED     = 0x0350;
    EPwm3Regs.DBRED     = 0x0350;

    EPwm3Regs.TZSEL.all     = 0x0100;
    EPwm3Regs.TZCTL.all     = 0x0005;
    EPwm3Regs.TZEINT.all    = 0x0004;
    EPwm3Regs.TZCLR.all     = 0x0000;
    EPwm3Regs.TZFRC.all     = 0x0000;
    EPwm3Regs.ETSEL.all     = 0x1109;
    EPwm3Regs.ETPS.all      = 0x0001;

    EPwm3Regs.PCCTL.all = 0x0000;

//
    EDIS;                                       // Disable access to protected registers
}
//====================================================================================================================
//  Function to Initialise the SCI Communication Registers
//====================================================================================================================
void InitSCIComm()
{

//SCI CONFIGURATION BITWISE

    /*
//-------------------------------------------------------------------------------------------------------------------
// SCI A Registers
//-------------------------------------------------------------------------------------------------------------------
// SCI A Communication Control Register[SCICCR]
    SciaRegs.SCICCR.bit.STOPBITS        = SCI_1STOP_BIT;    // Stop Bits,   [1 Stop Bit]

    SciaRegs.SCICCR.bit.PARITY          = SCI_ODD_PARITY;   // Parity Bits, [Odd Parity]
    SciaRegs.SCICCR.bit.PARITYENA       = SCI_DISABLED;     // Parity,      [Disabled]
    SciaRegs.SCICCR.bit.LOOPBKENA       = SCI_DISABLED;     // Loop Back,   [Disabled]
    SciaRegs.SCICCR.bit.ADDRIDLE_MODE   = SCI_IDLE_LINE;    // Mode,        [Idle Line]
    SciaRegs.SCICCR.bit.SCICHAR         = 111;              // Data Bits,   [111]

// SCI A Control Register1[SCICTL1]
    SciaRegs.SCICTL1.bit.RXERRINTENA    = SCI_DISABLED;     // Rx Error Interrupt,  [Disabled]
    SciaRegs.SCICTL1.bit.SWRESET        = SCI_SW_RELEASE;   // S/W Reset,           [Release from Reset]
    SciaRegs.SCICTL1.bit.TXWAKE         = SCI_DISABLED;     // Tx Wake up Method,   [Disabled]
    SciaRegs.SCICTL1.bit.SLEEP          = SCI_DISABLED;     // Sleep,               [Disabled]
    SciaRegs.SCICTL1.bit.TXENA          = SCI_ENABLED;      // Tx,                  [Enabled]
    SciaRegs.SCICTL1.bit.RXENA          = SCI_ENABLED;      // Rx,                  [Enabled]

// SCI A Baud Rate Register[SCIHBAUD] [SCILBAUD]
    SciaRegs.SCIHBAUD                   = 0x00;             // Baud Rate MSB, [0x00] for BRR = {LSPCLK/(SCI BR)x8} - 1
    SciaRegs.SCILBAUD                   = 0xC2;             // Baud Rate LSB, [0xC2] for BRR = 0, SCI BR = LSPCLK/16
                                                            // LSPCLK=120MHz/4=30MHz, so BRR = {15*10^6/(19200*4)}-1 = 194 = 0x00C2
// SCI A Control Register2[SCICTL2]
    SciaRegs.SCICTL2.bit.TXRDY          = SCI_TX_FULL;      // SCI Tx Ready,    [SCITXBUF is full]
    SciaRegs.SCICTL2.bit.TXEMPTY        = SCI_TX_FULL;      // SCI Tx Empty,    [TXBUF or Shift Register loaded with data]
    SciaRegs.SCICTL2.bit.RXBKINTENA     = SCI_DISABLED;     // SCI Rx Ready/BRKDT Interrupt, [Disabled]
    SciaRegs.SCICTL2.bit.TXINTENA       = SCI_DISABLED;     // SCI Tx Ready Interrupt,       [Disabled]

// SCI A FIFO Transmit Register[SCIFFTX]
    SciaRegs.SCIFFTX.bit.TXFFIENA       = SCI_DISABLED;     // Tx FIFO Interrupt Enable,     [Disable]

// SCI A FIFO Receive Register[SCIFFRX]
    SciaRegs.SCIFFRX.bit.RXFFIENA       = SCI_DISABLED;     // Rx FIFO Interrupt Enable,     [Disable]

// SCI A FIFO Control Register[SCIFFCT}
    SciaRegs.SCIFFCT.bit.CDC            = SCI_DISABLED;     // CDC calibrate, [Disabled auto-baud alignment]

// SCI A Priority Control Register[SCIPRI]
    SciaRegs.SCIPRI.bit.SOFT            = 0;                // Action when Emulation Suspend Occurs,    [01]
    SciaRegs.SCIPRI.bit.FREE            = 1;                // 00 => Immediate Stop on Suspend
                                                            // 01 => Complete current Rx/Tx sequence before stopping
                                                            // x1 => Free Run, Continues SCI operation regardless of Suspend
//-------------------------------------------------------------------------------------------------------------------
// SCI B Registers
//-------------------------------------------------------------------------------------------------------------------
// SCI B Communication Control Register[SCICCR]
    ScibRegs.SCICCR.bit.STOPBITS        = SCI_1STOP_BIT;    // Stop Bits,   [1 Stop Bit]
    ScibRegs.SCICCR.bit.PARITY          = SCI_ODD_PARITY;   // Parity Bits, [Odd Parity]
    ScibRegs.SCICCR.bit.PARITYENA       = SCI_DISABLED;     // Parity,      [Disabled]
    ScibRegs.SCICCR.bit.LOOPBKENA       = 1;     // Loop Back,   [Disabled]
    ScibRegs.SCICCR.bit.ADDRIDLE_MODE   = SCI_IDLE_LINE;    // Mode,        [Idle Line]
    ScibRegs.SCICCR.bit.SCICHAR         = 111;              // Data Bits,   [111]

// SCI B Control Register1[SCICTL1]
    ScibRegs.SCICTL1.bit.RXERRINTENA    = SCI_DISABLED;     // Rx Error Interrupt,  [Disabled]
    ScibRegs.SCICTL1.bit.SWRESET        = SCI_SW_RELEASE;   // S/W Reset,           [Release from Reset]
    ScibRegs.SCICTL1.bit.TXWAKE         = SCI_DISABLED;     // Tx Wake up Method,   [Disabled]
    ScibRegs.SCICTL1.bit.SLEEP          = SCI_DISABLED;     // Sleep,               [Disabled]
    ScibRegs.SCICTL1.bit.TXENA          = SCI_ENABLED;      // Tx,                  [Enabled]
    ScibRegs.SCICTL1.bit.RXENA          = SCI_ENABLED;      // Rx,                  [Enabled]

// SCI B Baud Rate Register[SCIHBAUD] [SCILBAUD]
    ScibRegs.SCIHBAUD                   = 0x00;             // Baud Rate MSB, [0x00]    BRR = 1 to 65535, BR = LSPCLK/{(BRR+1)x8}
    ScibRegs.SCILBAUD                   = 0xC2;             // Baud Rate LSB, [0xC2]    BRR = 0         , BR = LSPCLK/16
                                                            // LSPCLK=120MHz/4=30MHz, so BRR = {15*10^6/(19200*4)}-1 = 194 = 0x00C2

// SCI B Control Register2[SCICTL2]
    ScibRegs.SCICTL2.bit.TXRDY          = SCI_TX_FULL;      // SCI Tx Ready,    [SCITXBUF is full]
    ScibRegs.SCICTL2.bit.TXEMPTY        = SCI_TX_FULL;      // SCI Tx Empty,    [TXBUF or Shift Register loaded with data]
    ScibRegs.SCICTL2.bit.RXBKINTENA     = SCI_DISABLED;     // SCI Rx Ready/BRKDT Interrupt, [Disabled]
    ScibRegs.SCICTL2.bit.TXINTENA       = SCI_DISABLED;     // SCI Tx Ready Interrupt,       [Disabled]

// SCI B FIFO Transmit Register[SCIFFTX]
    ScibRegs.SCIFFTX.bit.TXFFIENA       = SCI_DISABLED;     // Tx FIFO Interrupt Enable,     [Disable]

// SCI B FIFO Receive Register[SCIFFRX]
    ScibRegs.SCIFFRX.bit.RXFFIENA       = SCI_DISABLED;     // Rx FIFO Interrupt Enable,     [Disable]

// SCI B FIFO Control Register[SCIFFCT}
    ScibRegs.SCIFFCT.bit.CDC            = SCI_DISABLED;     // CDC calibrate, [Disabled auto-baud alignment]

// SCI B Priority Control Register[SCIPRI]
    ScibRegs.SCIPRI.bit.SOFT            = 0;                // Action when Emulation Suspend Occurs
    ScibRegs.SCIPRI.bit.FREE            = 1;                // 00 => Immediate Stop on Suspend
                                                            // 01 => Complete current Rx/Tx sequence before stopping
                                                            // x1 => Free Run, Continues SCI operation regardless of Suspend
//-------------------------------------------------------------------------------------------------------------------
// SCI C Registers
//-------------------------------------------------------------------------------------------------------------------
// SCI C Communication Control Register[SCICCR]
    ScicRegs.SCICCR.bit.STOPBITS        = SCI_1STOP_BIT;    // Stop Bits,   [1 Stop Bit]
    ScicRegs.SCICCR.bit.PARITY          = SCI_ODD_PARITY;   // Parity Bits, [Odd Parity]
    ScicRegs.SCICCR.bit.PARITYENA       = SCI_DISABLED;     // Parity,      [Disabled]
    ScicRegs.SCICCR.bit.LOOPBKENA       = SCI_DISABLED;     // Loop Back,   [Disabled]
    ScicRegs.SCICCR.bit.ADDRIDLE_MODE   = SCI_IDLE_LINE;    // Mode,        [Idle Line]
    ScicRegs.SCICCR.bit.SCICHAR         = 111;              // Data Bits,   [111]

// SCI C Control Register1[SCICTL1]
    ScicRegs.SCICTL1.bit.RXERRINTENA    = SCI_DISABLED;     // Rx Error Interrupt,  [Disabled]
    ScicRegs.SCICTL1.bit.SWRESET        = SCI_SW_RELEASE;   // S/W Reset,           [Release from Reset]
    ScicRegs.SCICTL1.bit.TXWAKE         = SCI_DISABLED;     // Tx Wake up Method,   [Disabled]
    ScicRegs.SCICTL1.bit.SLEEP          = SCI_DISABLED;     // Sleep,               [Disabled]
    ScicRegs.SCICTL1.bit.TXENA          = SCI_ENABLED;      // Tx,                  [Enabled]
    ScicRegs.SCICTL1.bit.RXENA          = SCI_ENABLED;      // Rx,                  [Enabled]

// SCI C Baud Rate Register[SCIHBAUD] [SCILBAUD]
    ScicRegs.SCIHBAUD                   = 0x00;             // Baud Rate MSB, [0x00]    BRR = 1 to 65535, BR = LSPCLK/{(BRR+1)x8}
    ScicRegs.SCILBAUD                   = 0xC2;             // Baud Rate LSB, [0xC2]    BRR = 0         , BR = LSPCLK/16
                                                            // LSPCLK=120MHz/4=30MHz, so BRR = {15*10^6/(19200*4)}-1 = 194 = 0x00C2

// SCI C Control Register2[SCICTL2]
    ScicRegs.SCICTL2.bit.TXRDY          = SCI_TX_FULL;      // SCI Tx Ready,    [SCITXBUF is full]
    ScicRegs.SCICTL2.bit.TXEMPTY        = SCI_TX_FULL;      // SCI Tx Empty,    [TXBUF or Shift Register loaded with data]
    ScicRegs.SCICTL2.bit.RXBKINTENA     = SCI_DISABLED;     // SCI Rx Ready/BRKDT Interrupt, [Disabled]
    ScicRegs.SCICTL2.bit.TXINTENA       = SCI_DISABLED;     // SCI Tx Ready Interrupt,       [Disabled]

// SCI C FIFO Transmit Register[SCIFFTX]
    ScicRegs.SCIFFTX.bit.TXFFIENA       = SCI_DISABLED;     // Tx FIFO Interrupt Enable,     [Disable]

// SCI C FIFO Receive Register[SCIFFRX]
    ScicRegs.SCIFFRX.bit.RXFFIENA       = SCI_DISABLED;     // Rx FIFO Interrupt Enable,     [Disable]

// SCI C FIFO Control Register[SCIFFCT}
    ScicRegs.SCIFFCT.bit.CDC            = SCI_DISABLED;     // CDC calibrate, [Disabled auto-baud alignment]

// SCI C Priority Control Register[SCIPRI]
    ScicRegs.SCIPRI.bit.SOFT            = 0;                // Action when Emulation Suspend Occurs
    ScicRegs.SCIPRI.bit.FREE            = 1;                // 00 => Immediate Stop on Suspend
                                                            // 01 => Complete current Rx/Tx sequence before stopping
                                                            // x1 => Free Run, Continues SCI operation regardless of Suspend

*/

// SCI CONFIGURATION BITALL

    // SCI A
    SciaRegs.SCICCR.all     = 0x0007;
    SciaRegs.SCICTL1.all    = 0x0023;
    SciaRegs.SCIHBAUD       = 0x01;		// for 9600 = 0x01; for 19200 = 0x00
    SciaRegs.SCILBAUD       = 0x85;		// for 9600 = 0x85; for 19200 = 0xC2
    SciaRegs.SCICTL2.all    = 0x00C0;
    SciaRegs.SCIFFTX.all    = 0xA000;
    SciaRegs.SCIFFRX.all    = 0x201F;
    SciaRegs.SCIFFCT.all    = 0x0000;
    SciaRegs.SCIPRI.all     = 0x0008;

    // SCI B
    ScibRegs.SCICCR.all     = 0x0007;
    ScibRegs.SCICTL1.all    = 0x0023;
    ScibRegs.SCIHBAUD       = 0x01;		// for 9600 = 0x01; for 19200 = 0x00
    ScibRegs.SCILBAUD       = 0x85;		// for 9600 = 0x85; for 19200 = 0xC2
    ScibRegs.SCICTL2.all    = 0x00C0;
    ScibRegs.SCIFFTX.all    = 0xA000;
    ScibRegs.SCIFFRX.all    = 0x201F;
    ScibRegs.SCIFFCT.all    = 0x0000;
    ScibRegs.SCIPRI.all     = 0x0008;

    SCIB_RxTx_En = 1;       // Permanently Enabled Data Transfer of B Port.

    // SCI C
    ScicRegs.SCICCR.all     = 0x0007;
    ScicRegs.SCICTL1.all    = 0x0023;
    ScicRegs.SCIHBAUD       = 0x01;		// for 9600 = 0x01; for 19200 = 0x00
    ScicRegs.SCILBAUD       = 0x85;		// for 9600 = 0x85; for 19200 = 0xC2
    ScicRegs.SCICTL2.all    = 0x00C0;
    ScicRegs.SCIFFTX.all    = 0xA000;
    ScicRegs.SCIFFRX.all    = 0x201F;
    ScicRegs.SCIFFCT.all    = 0x0000;
    ScicRegs.SCIPRI.all     = 0x0008;



//


}
//====================================================================================================================
//  Function to Initialise the Internal ADC Registers
//====================================================================================================================
void InitADCint ()
{

// ADC CONFIGURATION BITWISE

// ADC Control Register 1[ADCTRL1]
    AdcRegs.ADCTRL1.bit.RESET           = ADC_RESET;        // ADC Module Reset,        [Reset]
    AdcRegs.ADCTRL1.bit.SUSMOD          = ADC_FREE_RUN;     // Emulation Suspend Mode,  [Free Run]
    AdcRegs.ADCTRL1.bit.ACQ_PS          = 0x4;              // Acquisition Time Prescale (S/H)
                                                            // (ACQ Window = (ACQ_PS+1)*(1/ADCCLK)

    AdcRegs.ADCTRL1.bit.CPS             = 1;                // Conversion Prescale, 0=> ADCCLK=FCLK/1
                                                            //                      1=> ADCCLK=FCLK/2
    AdcRegs.ADCTRL1.bit.CONT_RUN        = 0;                // Continuous Run,      0=> Stops after reaching End of Sequence
                                                            //                      1=> Continuous, Starts again from Initial State

    AdcRegs.ADCTRL1.bit.SEQ_OVRD        = 0;                // Sequencer Override,  (functions only if CONT_RUN = 1)
                                                            //                      0=> Sequencer Pointer resets to initial state at end of MAX_CONVn
                                                            //                      1=> Sequencer Pointer resets to initial state after End State
    AdcRegs.ADCTRL1.bit.SEQ_CASC        = ADC_SEQ_CASC;     // Sequencer Mode,      [Dual Mode]
                                                            // The sequence mode is changed to cascaded in new design for ADC all 16 values
// ADC Control Register 2[ADCTRL2]

    AdcRegs.ADCTRL2.bit.EPWM_SOCB_SEQ   = 0;                // ePWM SOC B,              0=> No Action
                                                            //   (Cascaded Mode only)   1=> Start by ePWM Signal
    AdcRegs.ADCTRL2.bit.RST_SEQ1        = 1;                // Reset Seq 1,             0=> No action
                                                            //                          1=> Immediate Reset Seq 1 to initial state
    AdcRegs.ADCTRL2.bit.SOC_SEQ1        = 0;                // Start Conversion (Seq1), 0=> Clear Pending SOC trigger
                                                            //                          1=> Software Trigger Start Seq 1
    AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1    = 0;                // Interrupt Enable (Seq1), 0=> Interrupt Disable
                                                            //                          1=> Interrupt Enable
    AdcRegs.ADCTRL2.bit.INT_MOD_SEQ1    = 0;                // Interrupt Mode (Seq1),   0=> Interrupt every EOS
                                                            //                          1=> Interrupt every other EOS
    AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1  = 0;                // ePWM SOC A Seq 1 Mask Bit, 0=> cannot be triggered by ePWM trigger
                                                            //                            1=> can be started by ePWM trigger
    AdcRegs.ADCTRL2.bit.EXT_SOC_SEQ1    = 0;                // External SOC (Seq1),     0=> No Action
                                                            //                          1=> Start by Signal from ADCSOC pin
    AdcRegs.ADCTRL2.bit.RST_SEQ2        = 1;                // Reset Seq 2,             0=> No Action
                                                            //                          1=> Immediate Reset Seq 2 to initial state
    AdcRegs.ADCTRL2.bit.SOC_SEQ2        = 0;                // Start Conversion (Seq2), 0=> Clear Pending SOC trigger
                                                            //                          1=> Software Trigger Start Seq 2
    AdcRegs.ADCTRL2.bit.INT_ENA_SEQ2    = 0;                // Interrupt Enable (Seq2), 0=> Interrupt Disable
                                                            //                          1=> Interrupt Enable
    AdcRegs.ADCTRL2.bit.INT_MOD_SEQ2    = 0;                // Interrupt Mode (Seq2),   0=> Interrupt every EOS
                                                            //                          1=> Interrupt every other EOS
    AdcRegs.ADCTRL2.bit.EPWM_SOCB_SEQ2  = 0;                // ePWM SOC B Seq 2 Mask Bit, 0=> cannot be triggered by ePWM trigger
                                                         //                            1=> can be started by ePWM trigger
// ADC Control Register 3[ADCTRL3]
    AdcRegs.ADCTRL3.bit.ADCBGRFDN       = ADC_POW_BGRF_UP;  // ADC Bandgap and Reference Power Down,  [Powered Up]
    AdcRegs.ADCTRL3.bit.ADCPWDN         = ADC_POW_UP;       // ADC Power Down (except Bandgap & Ref), [Powered Up]
    AdcRegs.ADCTRL3.bit.ADCCLKPS        = ADC_CLK_DIV1 ;//ADC_CLK_DIV0;     // ADC Clock Prescale,  [HSPCLK/{1x(ADCCTRL1.bit.CPS+1)}]
    AdcRegs.ADCTRL3.bit.SMODE_SEL       = ADC_SEQ_MODE;     // Sampling Mode Select, [Sequential Mode]


    AdcRegs.ADCMAXCONV.bit.MAX_CONV1    = 0xFF;              // Defines the Maximum number of conversions executed in an autoconversion session.
    AdcRegs.ADCREFSEL.bit.REF_SEL       = ADC_REF_INT;      // ADC Reference Voltage Select, [Internal Reference]
    AdcRegs.ADCOFFTRIM.bit.OFFSET_TRIM  = 0x0;              // ADC Offset Trim

    AdcRegs.ADCCHSELSEQ1.bit.CONV00     = 0x0;
    AdcRegs.ADCCHSELSEQ1.bit.CONV01     = 0x1;
    AdcRegs.ADCCHSELSEQ1.bit.CONV02     = 0x2;
    AdcRegs.ADCCHSELSEQ1.bit.CONV03     = 0x3;

    AdcRegs.ADCCHSELSEQ2.bit.CONV04     = 0x4;
    AdcRegs.ADCCHSELSEQ2.bit.CONV05     = 0x5;
    AdcRegs.ADCCHSELSEQ2.bit.CONV06     = 0x6;
    AdcRegs.ADCCHSELSEQ2.bit.CONV07     = 0x7;

    AdcRegs.ADCCHSELSEQ3.bit.CONV08     = 0x8;
    AdcRegs.ADCCHSELSEQ3.bit.CONV09     = 0x9;
    AdcRegs.ADCCHSELSEQ3.bit.CONV10     = 0xA;
    AdcRegs.ADCCHSELSEQ3.bit.CONV11     = 0xB;

    AdcRegs.ADCCHSELSEQ4.bit.CONV12     = 0xC;
    AdcRegs.ADCCHSELSEQ4.bit.CONV13     = 0xD;
    AdcRegs.ADCCHSELSEQ4.bit.CONV14     = 0xE;
    AdcRegs.ADCCHSELSEQ4.bit.CONV15     = 0xF;




// ADC CONFIGURATION BITALL

    AdcRegs.ADCTRL1.all     = 0x0490;


    AdcRegs.ADCTRL2.all     = 0x0000;

    AdcRegs.ADCTRL3.all     = 0x00E2;

    AdcRegs.ADCMAXCONV.all  = 0x000F;
    AdcRegs.ADCREFSEL.all   = 0x0000;
    AdcRegs.ADCOFFTRIM.all  = 0x0000;

    AdcRegs.ADCCHSELSEQ1.all    = 0x3210;
    AdcRegs.ADCCHSELSEQ2.all    = 0x7654;
    AdcRegs.ADCCHSELSEQ3.all    = 0xBA98;
    AdcRegs.ADCCHSELSEQ4.all    = 0xFEDC;

 //   AdcRegs.ADCTRL2.all = 0x4040 ; // Wait till SOC
 //   AdcRegs.ADCTRL2.all = 0x2020;
    VFD_Data.ADC_Flag = 0;







//

}
//====================================================================================================================
//  Function to Initialise the SPI Control Registers
//====================================================================================================================
void InitSPI()
{
// SPI Configuration Control Registers[SPICCR]

    SpiaRegs.SPICCR.bit.SPISWRESET      = 0;                // SPI S/W Reset,   0 => SPI Flags Reset
                                                            //                  1 => Normal Operation
    SpiaRegs.SPICCR.bit.CLKPOLARITY     = 0;                // Clock Polarity,  0 => Rising Edge Data Transfer
                                                            //                  1 => Falling Edge Data Transfer
    SpiaRegs.SPICCR.bit.SPILBK          = 0;                // SPI Loopback Mode,       0 = > Disabled, default value after reset
                                                            //  (valid in master mode)  1 = > Enabled, default value after reset
    SpiaRegs.SPICCR.bit.SPICHAR         = 15;               // Character Length, 0=1bit, .., 15=16bits

    SpiaRegs.SPISTS.bit.OVERRUN_FLAG = 0;
    SpiaRegs.SPISTS.bit.INT_FLAG = 0;



    SpiaRegs.SPICTL.bit.MASTER_SLAVE    = 1;               // SPI configured as Master.

    SpiaRegs.SPICTL.bit.TALK            = 1; // To Enable Data Transmission
    SpiaRegs.SPIBRR                     = 50;              // Baud Rate MSB, [127] SPIBRR = 3 to 127, BR = LSPCLK/(SPIBRR+1)
                                                            //                      SPIBRR = 0, 1, 2,  BR = LSPCLK/4
// SPI FIFO Registers[SPIFFTX], [SPIFFRX]
    SpiaRegs.SPIFFTX.all                = 0xE040;           // FIFO Transmit Register
//    SpiaRegs.SPIFFRX.all                = 0x204F;           // FIFO Receive Register
// SPI FIFO Control Register[SPIFFCT]
    SpiaRegs.SPIFFCT.bit.TXDLY          = 0x0;              // FIFO Tx Delay bits,
// SPI Priority Control Register[SPIPRI]
    SpiaRegs.SPIPRI.bit.SOFT            = 0;                // Emulation Suspend Event, 00 => Tx stops midway
    SpiaRegs.SPIPRI.bit.FREE            = 1;                //                          10 => Stop after Tx the words in the shift register and buffer
                                                            //                          x1 => Free Run, continue SPI operation
  //  SpiaRegs.SPIFFTX.bit.SPIFFENA = 0;
    SpiaRegs.SPICCR.bit.SPISWRESET      = 1;                // SPI Made to Normal Operation

}

void InitTimer()
{
    CpuTimer1Regs.TIM.all = 0;
    Timer_counter = (float32)(Sampling_Time);
    CpuTimer1Regs.PRD.all = Timer_counter*Clock_freq; // The counter is loaded with ISR freq counts
   /*         CpuTimer1Regs.TCR.bit.TIF =0; // Flag is initialsed to zero
            CpuTimer1Regs.TCR.bit.TIE =1; // Interrupt is enabled
            CpuTimer1Regs.TCR.bit.rsvd1 =0;
            CpuTimer1Regs.TCR.bit.rsvd2 =0;
            CpuTimer1Regs.TCR.bit.rsvd3 =0;
            CpuTimer1Regs.TCR.bit.FREE = TB_FREE_RUN; // Free Run
            CpuTimer1Regs.TCR.bit.TRB = 0; // Initilased to zero
            CpuTimer1Regs.TCR.bit.TSS = 1; // Timer Not started.
            CpuTimer1Regs.TPR.all = 0; // Prescaler made to zero. ;
            CpuTimer1Regs.TPRH.all=0;
            */
     CpuTimer1Regs.TCR.all = 0x4811;// Timer Not started and Timer Interrupt made enable  ;

}

//====================================================================================================================
// Function to initialize the PIE Control
//====================================================================================================================
void InitPieCtrl()
{
    EALLOW;                                         // Enable access to protected registers

    DINT;                                           // Disable Interrupts at the CPU level:
//-------------------------------------------------------------------------------------------------------------------
//  PIECTRL Register [PIECTRL]
//-------------------------------------------------------------------------------------------------------------------
    PieCtrlRegs.PIECTRL.bit.ENPIE = 0;              // Disable the PIE
//-------------------------------------------------------------------------------------------------------------------
// PIE Interrupt Acknowledge Register [PIEACK]
//-------------------------------------------------------------------------------------------------------------------
    PieCtrlRegs.PIEACK.all      = 0xFFFF;           // Clears the respective interrupt and enables the PIE block to drive a pulse into the CPU interrupt
//-------------------------------------------------------------------------------------------------------------------
// PIE Interrupt Enable Registers[PIEIERx Register]
//-------------------------------------------------------------------------------------------------------------------
    PieCtrlRegs.PIEIER1.all  = 0x0000;                      // Disable INT1  Group interrupts
    PieCtrlRegs.PIEIER2.all  = 0x0000;                      // Disable INT2  Group interrupts
    PieCtrlRegs.PIEIER3.all  = 0x0000;                      // Disable INT3  Group interrupts
    PieCtrlRegs.PIEIER4.all  = 0x0000;                      // Disable INT4  Group interrupts
    PieCtrlRegs.PIEIER5.all  = 0x0000;                      // Disable INT5  Group interrupts
    PieCtrlRegs.PIEIER6.all  = 0x0000;                      // Disable INT6  Group interrupts
    PieCtrlRegs.PIEIER7.all  = 0x0000;                      // Disable INT7  Group interrupts
    PieCtrlRegs.PIEIER8.all  = 0x0000;                      // Disable INT8  Group interrupts
    PieCtrlRegs.PIEIER9.all  = 0x0000;                      // Disable INT9  Group interrupts
    PieCtrlRegs.PIEIER10.all = 0x0000;                      // Disable INT10 Group interrupts
    PieCtrlRegs.PIEIER11.all = 0x0000;                      // Disable INT11 Group interrupts
    PieCtrlRegs.PIEIER12.all = 0x0000;                      // Disable INT12 Group interrupts

//-------------------------------------------------------------------------------------------------------------------
// PIE Interrupt Flag Registers [PIEIFRx Register]
//-------------------------------------------------------------------------------------------------------------------
    PieCtrlRegs.PIEIFR1.all  = 0x0000;                      // Clear INT1  Group interrupts
    PieCtrlRegs.PIEIFR2.all  = 0x0000;                      // Clear INT2  Group interrupts
    PieCtrlRegs.PIEIFR3.all  = 0x0000;                      // Clear INT3  Group interrupts
    PieCtrlRegs.PIEIFR4.all  = 0x0000;                      // Clear INT4  Group interrupts
    PieCtrlRegs.PIEIFR5.all  = 0x0000;                      // Clear INT5  Group interrupts
    PieCtrlRegs.PIEIFR6.all  = 0x0000;                      // Clear INT6  Group interrupts
    PieCtrlRegs.PIEIFR7.all  = 0x0000;                      // Clear INT7  Group interrupts
    PieCtrlRegs.PIEIFR8.all  = 0x0000;                      // Clear INT8  Group interrupts
    PieCtrlRegs.PIEIFR9.all  = 0x0000;                      // Clear INT9  Group interrupts
    PieCtrlRegs.PIEIFR10.all = 0x0000;                      // Clear INT10 Group interrupts
    PieCtrlRegs.PIEIFR11.all = 0x0000;                      // Clear INT11 Group interrupts
    PieCtrlRegs.PIEIFR12.all = 0x0000;                      // Clear INT12 Group interrupts
//-------------------------------------------------------------------------------------------------------------------
    EDIS;                                       // Disable access to protected registers
}
//====================================================================================================================
// Function to initialize the PIE Vector Table.
//====================================================================================================================
void InitPieVectTable()
{
    EALLOW;                                         // Enable access to protected registers

    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;                      // Enable vector fetching from PIE vector table

    PieCtrlRegs.PIEIER2.bit.INTx1 = 1;                      // Enable EPWM Trip Zone interrupt at peripheral level
//                                                          //      PIEIER2.bit.INTx1 => INT2.1 [EPWM1_TZINT]
//                                                          //      PIEIER2.bit.INTx2 => INT2.2 [EPWM2_TZINT]
//                                                          //      PIEIER2.bit.INTx3 => INT2.3 [EPWM3_TZINT]
//                                                          //      PIEIER2.bit.INTx4 => INT2.4 [EPWM4_TZINT]
//                                                          //      PIEIER2.bit.INTx5 => INT2.5 [EPWM5_TZINT]
//                                                          //      PIEIER2.bit.INTx6 => INT2.6 [EPWM6_TZINT]
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;                      // Enable EPWM interrupt at peripheral level
//                                                          //      PIEIER3.bit.INTx1 => INT3.1 [EPWM1_INT]
//                                                          //      PIEIER3.bit.INTx2 => INT3.2 [EPWM2_INT]
//                                                          //      PIEIER3.bit.INTx3 => INT3.3 [EPWM3_INT]
//                                                          //      PIEIER3.bit.INTx4 => INT3.4 [EPWM4_INT]
//                                                          //      PIEIER3.bit.INTx5 => INT3.5 [EPWM5_INT]
//                                                          //      PIEIER3.bit.INTx6 => INT3.6 [EPWM6_INT]
//-------------------------------------------------------------------------------------------------------------------
// Interrupt Flag Register [IFR]
//-------------------------------------------------------------------------------------------------------------------
    IFR =   IFR & 0x0000;
//-------------------------------------------------------------------------------------------------------------------
// Interrupt Enable Register [IER]
//-------------------------------------------------------------------------------------------------------------------
    IER =   IER | 0x1006;                                   // Enable Interrupt 3 INT3 => EPWM_INT,     bit2
//                                                          // Enable Interrupt 2 INT2 => EPWM_TZINT,   bit1
//-------------------------------------------------------------------------------------------------------------------
// Interrupt Service Routine Address Location
//-------------------------------------------------------------------------------------------------------------------
    EPWM_ISR_VecIDLo    = 0xF000;                           // EPWM ISR Locations Specified
    EPWM_ISR_VecIDHi    = 0x0000;

    EPWM_TZ_ISR_VecIDLo = 0xF020;                           // EPWM TZ ISR Locations Specified
    EPWM_TZ_ISR_VecIDHi = 0x0000;

    TIMER_One_ISRLo = 0xF060;
    TIMER_One_ISRHi = 0x0000;
    EINT;                                                   // Enable Global interrupt INTM
    EDIS;                                                   // Disable access to protected registers
}


//====================================================================================================================
// INITIALISE USER DATA
//====================================================================================================================
void Init_User_Data()
{

    if ( User_Data.Motor_Type == 01)    // INDM Motor Selection
    {

    #if (INDMotorSpec == INDM_03k7)    // 03.7kW INDM SPECS

//================================================================================================
//  Control Parameters
//================================================================================================

        User_Data.Control_Mode  = 13;   /* 11 => INDM V/F - Open Loop       */
                                        /* 12 => INDM V/F - Closed Loop     */
                                        /* 13 => INDM FOC - Sensorless Mode */
        // Motor Run time Settings
        User_Data.Sw_Freq       = 03;   /* Switching Frequency (kHz)        */
        User_Data.Mod_Ind       = 1.225;/* Modulation Index (1.00 - 1.30)   */
                                        /* Default max = 1.225              */
                                        /* 0.00 to 1.00: Perfect Sinusoidal */
                                        /* 1.00 to 1.225: Quasi-Sinusoidal  */
                                        /* 1.225 to 1.30: Trapezoidal       */

        User_Data.Op_Freq       = 30.00;
                                        /* Operating Frequency  (Hz)        */
   //     User_Data.FluxStrt      = 20;   /* Starting Flux (20 - 50) (%)      */
//        User_Data.TorqStrt      = 00;   /* Starting Torq (00 - 40) (%)      */

        User_Data.Accel_Rate    = 3.0; /* Rate of Acceleration (Hz/s)      */
        User_Data.Mot_Rot       = 0;    /* 0 = Anticlockwise                */
                                        /* 1 = Clockwise                    */

//================================================================================================
//  Motor Ratings and Parameters - 03.7kW INDM
//================================================================================================

    // Motor Name Plate Ratings
        User_Data.Prated    = 3.7;  // Rated Power of M/C       (kW)
        User_Data.Vrated    = 415;  // Rated Voltage of M/C     (V)
        User_Data.Irated    = 7.5;  // Rated Current of M/C     (A)
                                    // CHECK CURRENT SCALING in variable.h
        User_Data.Frated    = 50.0; // Rated Frequency of M/C   (Hz)
        User_Data.Nrated    = 1450; // Rated Speed of M/C       (rpm)

        MOT_Data.Jm         = 0.0520;   // Moment of Inertia    (kg m^2)

    // Motor Parameters
        User_Data.Rs    = 2.0000000;    /* INDM Stator resistance (ohm)      */
        User_Data.Rr    = 2.0000000;    /* INDM Rotor resistance (ohm)       */
        User_Data.Ls    = 0.3210000;    /* INDM Stator inductance (H)        */
        User_Data.Lr    = 0.3210000;    /* INDM Rotor inductance (H)         */
        User_Data.Lm    = 0.3050;       /* INDM Magnetizing inductance (H)   */
        User_Data.Energy_Efficient = 0;   // Energy Efficient Mode
        User_Data.AnaVoltMin    = 1.0;      /* Default Analog Minimum Value */
        User_Data.AnaVoltMax    = 2.0;    /* Default Analog Max Value */
        User_Data.SpdAnaMin     = 0;      /* Default Minimum Speed */
        User_Data.SpdAnaMax     = User_Data.Nrated ;  /* Default Maximum */


    #endif

    #if (INDMotorSpec == INDM_22k0)    // 22.0kW INDM SPECS

//================================================================================================
//  Control Parameters - 22.0kW INDM
//================================================================================================

        User_Data.Control_Mode  = 13;   /* 11 => INDM V/F - Open Loop       */
                                        /* 12 => INDM V/F - Closed Loop     */
                                        /* 13 => INDM FOC - Sensorless Mode */
        // Motor Run time Settings
        User_Data.Sw_Freq       = 03;   /* Switching Frequency (kHz)        */
        User_Data.Mod_Ind       = 1.225;/* Modulation Index (1.00 - 1.30)   */
                                        /* Default max = 1.225              */
                                        /* 0.00 to 1.00: Perfect Sinusoidal */
                                        /* 1.00 to 1.225: Quasi-Sinusoidal  */
                                        /* 1.225 to 1.30: Trapezoidal       */

        User_Data.Op_Freq       = 49;   // 49 Hz => 1470 rpm
                                        /* Operating Frequency  (Hz)        */
 //       User_Data.FluxStrt      = 20;   /* Starting Flux (20 - 50) (%)      */
//        User_Data.TorqStrt      = 00;   /* Starting Torq (00 - 40) (%)      */

        User_Data.Accel_Rate    = 3.0; /* Rate of Acceleration (Hz/s)      */
        User_Data.Mot_Rot       = 0;    /* 0 = Anticlockwise                */
                                        /* 1 = Clockwise                    */
//================================================================================================
//  Motor Ratings and Parameters - 22.0kW INDM
//================================================================================================

    // Motor Name Plate Ratings
        User_Data.Prated    = 22.0; // Rated Power of M/C       (kW)
        User_Data.Vrated    = 400;  // Rated Voltage of M/C     (V)
        User_Data.Irated    = 40;   // Rated Current of M/C     (A)
                                    // CHECK CURRENT SCALING in variable.h
        User_Data.Frated    = 50.0; // Rated Frequency of M/C   (Hz)
        User_Data.Nrated    = 1473; // Rated Speed of M/C       (rpm)

        MOT_Data.Jm         = 0.2325;   // Moment of Inertia    (kg m^2)

    // Motor Parameters
        User_Data.Rs    = 0.1310000;    /* INDM Stator resistance (ohm)      */
        User_Data.Rr    = 0.1846250;    /* INDM Rotor resistance (ohm)       */
        User_Data.Ls    = 0.0547051;    /* INDM Stator inductance (H)        */
        User_Data.Lr    = 0.0547051;    /* INDM Rotor inductance (H)         */
        User_Data.Lm    = 0.0527399;    /* INDM Magnetizing inductance (H)   */
        User_Data.Energy_Efficient = 0;
        User_Data.AnaVoltMin    = 1.0;      /* Default Analog Minimum Value */
        User_Data.AnaVoltMax    = 2.0;    /* Default Analog Max Value */
        User_Data.SpdAnaMin     = 0;      /* Default Minimum Speed */
        User_Data.SpdAnaMax     = User_Data.Nrated ;  /* Default Maximum */

    #endif
    }

    if (User_Data.Motor_Type == 02)     // PMSM Motor Selection
    {

    #if (PMSMotorSpec == PMSM_05k8)    // 05.8kW PMSM SPECS

//================================================================================================
//  Control Parameters
//================================================================================================

        User_Data.Control_Mode  = 21;   /* 21 => PMSM FOC - Current Mode    */
                                        /* 22 => PMSM FOC - Sensorless Mode */
        // Motor Run time Settings
        User_Data.Sw_Freq       = 10;   /* Switching Frequency (kHz)        */
        User_Data.Mod_Ind       = 1.225;/* Modulation Index (1.00 - 1.30)   */
                                        /* Default max = 1.225              */
                                        /* 0.00 to 1.00: Perfect Sinusoidal */
                                        /* 1.00 to 1.225: Quasi-Sinusoidal  */
                                        /* 1.225 to 1.30: Trapezoidal       */

        User_Data.Op_Freq       = (1.0 * 133.333333333);
                                        /* Operating Frequency  (Hz)        */
//        User_Data.FluxStrt      = 20;   /* Starting Flux (20 - 50) (%)      */
        User_Data.TorqStrt      = 30;   /* Starting Torq (00 - 40) (%)      */
        User_Data.Accel_Rate    = 05.0; /* Rate of Acceleration (Hz/s)      */
        User_Data.Mot_Rot       = 0;    /* 0 = Anticlockwise                */
                                        /* 1 = Clockwise                    */
//================================================================================================
//  Motor Ratings and Parameters - 05.8kW PMSM
//================================================================================================
    // Motor Name Plate Ratings

        User_Data.Prated    = 5.83; // Rated Power of M/C       (kW)
        User_Data.Vrated    = 300;  // Rated Voltage of M/C     (V)
        User_Data.Irated    = 24;   // Rated Current of M/C     (A)
                                    // CHECK CURRENT SCALING in variable.h
        User_Data.Frated    = 133.333333333;  // Rated Frequency of M/C   (Hz)
        User_Data.Nrated    = 2000; // Rated Speed of M/C       (rpm)

        MOT_Data.Jm         = 0.0101;   // Moment of Inertia    (kg m^2)

    // Motor Parameters
        User_Data.Rdc   = 0.2250000;    /* PMSM Stator resistance (ohm)      */
        User_Data.Ld    = 0.0013000;    /* PMSM D axis inductance (H)        */
        User_Data.Lq    = 0.0013000;    /* PMSM Q axis inductance (H)        */
        User_Data.Ebc   = 88.4;         /* PMSM Back EMF Constant (V/krpm)   */
        User_Data.AnaVoltMin    = 1.0;      /* Default Analog Minimum Value */
        User_Data.AnaVoltMax    = 2.0;    /* Default Analog Max Value */
        User_Data.SpdAnaMin     = 0;      /* Default Minimum Speed */
        User_Data.SpdAnaMax     = User_Data.Nrated ;  /* Default Maximum */
    #endif

    #if (PMSMotorSpec == PMSM_22k8)    // 22.8kW PMSM SPECS

//================================================================================================
//  Control Parameters - 22.8kW PMSM
//================================================================================================

        User_Data.Control_Mode  = 22;   /* 21 => PMSM FOC - Current Mode    */
                                        /* 22 => PMSM FOC - Sensorless Mode */
        // Motor Run time Settings
        User_Data.Sw_Freq       = 10;   /* Switching Frequency (kHz)        */
        User_Data.Mod_Ind       = 1.225;/* Modulation Index (1.00 - 1.30)   */
                                        /* Default max = 1.225              */
                                        /* 0.00 to 1.00: Perfect Sinusoidal */
                                        /* 1.00 to 1.225: Quasi-Sinusoidal  */
                                        /* 1.225 to 1.30: Trapezoidal       */

        User_Data.Op_Freq       = (1.0 *100.00);
                                        /* Operating Frequency  (Hz)        */
 //       User_Data.FluxStrt      = 0;// 30;   /* Starting Flux (20 - 50) (%)      */
        User_Data.TorqStrt      = 30;   /* Starting Torq (00 - 40) (%)      */
        User_Data.Accel_Rate    = 3.0; /* Rate of Acceleration (Hz/s)      */
        User_Data.Mot_Rot       = 0;    /* 0 = Anticlockwise                */
                                        /* 1 = Clockwise                    */
//================================================================================================
//  Motor Ratings and Parameters - 22.8kW PMSM
//================================================================================================

    //Motor Name Plate
        User_Data.Prated    = 22.8; // Rated Power of M/C       (kW)
        User_Data.Vrated    = 360;  // Rated Voltage of M/C     (V)
        User_Data.Irated    = 43;   // Rated Current of M/C     (A)
                                    // CHECK CURRENT SCALING in variable.h
        User_Data.Frated    = 100;  // Rated Frequency of M/C   (Hz)
        User_Data.Nrated    = 1500; // Rated Speed of M/C       (rpm)

        MOT_Data.Jm         = 0.0626;   // Moment of Inertia    (kg m^2)

    //Motor Parameters
        User_Data.Rdc   = 0.1510000;    /* PMSM Stator resistance (ohm)      */
        User_Data.Ld    = 0.00553000;    /* PMSM D axis inductance (H)       */
        User_Data.Lq    = 0.00553000;    /* PMSM Q axis inductance (H)       */
        User_Data.Ebc   = 245.0;        /* PMSM Back EMF Constant (V/krpm)   */
        User_Data.AnaVoltMin    = 1.0;      /* Default Analog Minimum Value */
        User_Data.AnaVoltMax    = 2.0;    /* Default Analog Max Value */
        User_Data.SpdAnaMin     = 0;      /* Default Minimum Speed */
        User_Data.SpdAnaMax     = User_Data.Nrated ;  /* Default Maximum */

    #endif
    }

}
// Initialise Control Variables
void Initialise_Variables()
{
    VFD_Status.all                  = 0x0000;               // Clear the system status bits
    VFD_Status.bits.UpdateVars      = 1;                    // Update VFD Variables
    User_Data.ReInitParm            = 0;

//------------------------------------------------------------------------------------------------------
// Configure the Inverter Functions
//------------------------------------------------------------------------------------------------------
    INV_Config();
//===================================================================================================================
// VFD CONTROL PARAMETERS
//===================================================================================================================
    Init_Motor_Params();        // Induction Motor Parameters
    Init_VFD_Data_Vars();       // Initialise VFD Data Variables
    Init_FreqRamp_Vars();       // Initialise Frequency Ramp Controller Parameters
    Init_VsdRamp_Vars();        // Initialise Torq Ramp Controller Parameters
    Init_VsqRamp_Vars();        // Initialise VsQ Ramp Controller Parameters
    Init_VbyF_Vars();           // Initialise VbyF Control Profile Parameters
    Init_FluxSpeedEst_Vars();   // Set Motor time constants for Flux & Speed Estimation
    Init_PIcntrl_Vars();        // Initialise PI Control
    Init_SVM_Vars();            // Initialise SVPWM Generator Structure
    Init_RotPosEst_Vars();      // Initialise Rotor postion Estimaton Structure        ( Initialise Only First Time )
    Init_DynBrake_Vars();       // Initialise Dynamic Brake Control Variables
    Init_Modbus_Vars();         // Initialise MODBUS Communication Structure
    delay(5000); // Delay for Initilsation
    delay(5000); // Delay for Initilsation

//------------------------------------------------------------------------------------------------------
// ADC Scaling Factor Initialisation
//------------------------------------------------------------------------------------------------------

    VFD_IntAdcData.EncTem_Scal   = _IQ((float32) ((100.0) / (DigVal_ET)));      // Check Excel Sheet
    VFD_IntAdcData.HskTem_Scal   = _IQ((float32) (DigVal_HT) );      // Check Excel Sheet

    VFD_IntAdcData.SpdRef_Scal   = _IQ((float32) 1 / (DigVal_SP) );        // Check Excel Sheet //_IQ(0.00006537657);   // Ext ADC, 2.5V = 15296 dec

//    VFD_IntAdcData.Pt100_Scal    = _IQ((float32) Pt100_Vptr / (DigVal_Pt100) );      // Check Excel Sheet

    // Check Init_VFD_Data_Vars() for Current, PT & DC Bus Voltage Scaling

//------------------------------------------------------------------------------------------------------
// Set the Current, Voltage and Temperature Limits
//------------------------------------------------------------------------------------------------------
    VFD_Limits.ABC_OC_Limit         = _IQ(2.1);             // Above 1.82 ADC saturates and hence
    VFD_Limits.Ds_OC_Limit          = _IQ(2.00);             // Ds OverCurrLimit
    VFD_Limits.Qs_OC_Limit          = _IQ(2.00);             // Qs OverCurrLimit

    VFD_Limits.Hsk_OT_Limit         = _IQ15(Hsk_max);        // Maximum Permitted Heat Sink Temperature
    VFD_Limits.Enc_OT_Limit         = _IQ((float32)(Enc_max / 25.0));        // Maximum Permitted Enclosure Temperature

    VFD_Limits.Mot_OT_Limit         = _IQ15(MotTmp_max);     // Maximum Permitted Motor Temperature


    VFD_Limits.DCBrakeActLimit      = _IQmpyIQX(_IQ15(Vdc_BrkAct), 15, _IQ(VFD_Data.InvGain_1), 24);
    VFD_Limits.DCBrakeActLimit      = _IQmpy(VFD_Limits.DCBrakeActLimit, _IQ(0.5));
    VFD_Limits.DCBrakeRelLimit      = _IQmpyIQX(_IQ15(Vdc_BrkRel), 15, _IQ(VFD_Data.InvGain_1), 24);
    VFD_Limits.DCBrakeRelLimit      = _IQmpy(VFD_Limits.DCBrakeRelLimit, _IQ(0.5));

    VFD_Limits.DC_OV_Limit          = _IQmpyIQX(_IQ15(Vdc_max), 15, _IQ(VFD_Data.InvGain_1), 24);
    VFD_Limits.DC_OV_Limit          = _IQmpy(VFD_Limits.DC_OV_Limit, _IQ(0.5));
    VFD_Limits.DC_UV_Limit          = _IQmpyIQX(_IQ15(Vdc_min), 15, _IQ(VFD_Data.InvGain_1), 24);
    VFD_Limits.DC_UV_Limit          = _IQmpy(VFD_Limits.DC_UV_Limit, _IQ(0.5));


    // Initialise ADC Monitoring Variables in VFD_Data
        VFD_Data.Vdc    = VFD_Data.SpdRef   = 0;
        VFD_Data.Isa    = VFD_Data.Isa_abs  = 0;
        VFD_Data.Isb    = VFD_Data.Isb_abs  = 0;
        VFD_Data.Isc    = VFD_Data.Isc_abs  = 0;
        VFD_Data.Vgry   = VFD_Data.Vgyb     = 0;
        VFD_Data.HskTem  = _IQ(0.0) ;
  //      VFD_Data.Dig15V = VFD_Data.Dig5V    = 0;
  //      VFD_Data.Renc     = 0;
        VFD_Data.Rhsk   = VFD_Data.Rpt100   = 0;
        VFD_Data.EncTem = VFD_Data.HskTem   = 0;
        VFD_Data.Pt100                      = 0;
        VFD_Data.VdcFilt    = VFD_Data.VdcFiltOld   = 0;
        VFD_Data.RhskFilt  = VFD_Data.RhskFiltOld  = 0 ;
        VFD_Data. DAxisFilt = VFD_Data.DAxisFiltOld  = 0;

        VFD_Data. GridFilt = VFD_Data.GridFiltOld  = 0;



        VFD_IntAdcData.Mon_Vdc = 0;
        AdcRegs.ADCRESULT6 = 0;
        VFD_Data.offset_Isa = 0 ;
        VFD_Data.offset_Isb = 0;
        VFD_Data.offset_Isc = 0;
        VFD_Data.ADC_Flag  = 0;

        VFD_Limits.Vdgrid   = _IQ(0.0);
        VFD_Limits.Vmot_Rph_Counter = 0;
      VFD_Limits.Vmot_Yph_Counter = 0;
      VFD_Limits.Vmot_Bph_Counter = 0;
      VFD_Data.average_Vmot_r = 0;
      VFD_Data.average_Vmot_y = 0;
      VFD_Data.average_Vmot_b = 0;



    //    AdcRegs.ADCRESULT15 = 0;


    MOT_RC.Angle            = 0;    // Initialise Only First Time
    MOT_RC.RampOut          = 0;    // Initialise Only First Time
    MOT_FSE.ThetaFlux       = 0;    // Initialise Only First Time
    MOT_FSE.ThetaFluxAtD    = 0;    // Initialise Only First Time
    MOT_FSE.ThetaFluxAtQ    = 0;    // Initialise Only First Time
    VFD_Status.bits.StartFlag = 0;  // Initilase to a known state


}
// Inverter Configuration
void INV_Config()
{
//------------------------------------------------------------------------------------------------------
// Configure the Inverter Functions - Switching Frequency (kHz), Dead Band time (us)
//------------------------------------------------------------------------------------------------------
    VFD_Data.Sys_Freq   = 120;                          // DSP System Frequency = 120MHz

    if (User_Data.Sw_Freq < Swfreq_min)
        VFD_Data.Sw_Freq    = User_Data.Sw_Freq = Swfreq_min;               // Minimum Limit for Switching Frequency (2kHz)
    else if (User_Data.Sw_Freq > Swfreq_max)
        VFD_Data.Sw_Freq    = User_Data.Sw_Freq = Swfreq_max;               // Maximum Limit for Switching Frequency (12kHz)
    else
        VFD_Data.Sw_Freq    = User_Data.Sw_Freq;        // Tested @ 2,3,4,5,6,7,8,9,10,12 kHz, working fine

    VFD_Data.Dead_Band  = 4;                            // Dead Band = 4us, Check Sheet for details

  //  VFD_Data.Ts         = _IQ((float32) 0.001 / VFD_Data.Sw_Freq);      // Initialise the VFD Sampling time
  //  VFD_Data.Ts_1       = _IQ15((float32) VFD_Data.Sw_Freq * 1000);

    VFD_Data.Ts =  _IQ((float32)(.000001*Sampling_Time));
    VFD_Data.Ts_1 = _IQ15((float32)(Sampling_Frequency*1000000)); //
    VFD_Data.Tsdc         = _IQ((float32) 0.001/ VFD_Data.Sw_Freq );  // DC Link Voltage sampled at 333us

    MOT_SVM.PeriodMax   = ((50 * VFD_Data.Sys_Freq) / (0.1 * VFD_Data.Sw_Freq))+1;// For 3kHz Sw Freq, TBPRD = Tprd/(2*TBCLK) = (1/3kHz)/(2/120MHz) = 0x4E70
    EPwm1Regs.TBPRD     = MOT_SVM.PeriodMax;
    EPwm2Regs.TBPRD     = MOT_SVM.PeriodMax;
    EPwm3Regs.TBPRD     = MOT_SVM.PeriodMax;

    MOT_SVM.HalfPerMax  = MOT_SVM.PeriodMax/2;                                  // Max Value for Modulation Index = 1

    MOT_SVM.Deadband    = (VFD_Data.Dead_Band * VFD_Data.Sys_Freq);             // For 7us Dead Band, [7us/(1/120MHz) = 7*120 = 0x0350]
    EPwm1Regs.DBRED     = MOT_SVM.Deadband;
    EPwm1Regs.DBFED     = MOT_SVM.Deadband;
    EPwm2Regs.DBRED     = MOT_SVM.Deadband;
    EPwm2Regs.DBFED     = MOT_SVM.Deadband;
    EPwm3Regs.DBRED     = MOT_SVM.Deadband;
    EPwm3Regs.DBFED     = MOT_SVM.Deadband;

    //------------------------------------------------------------------------------------------------------
    // Motor Type & Control Settings
    //------------------------------------------------------------------------------------------------------
    // Motor Type Setting
        if (User_Data.Motor_Type == 2)
            VFD_Data.Mot_Type = User_Data.Motor_Type  = 2;        // PMSM Selected
        else
            VFD_Data.Mot_Type = User_Data.Motor_Type  = 1;        // INDM By Default
    // Control Mode Setting
        if ( ((User_Data.Control_Mode < 23) && (User_Data.Control_Mode > 20)) && VFD_Data.Mot_Type == 2)     // PMSM Current mode, PMSM FOC
            VFD_Data.Con_Mode  =   User_Data.Control_Mode;
        else if (VFD_Data.Mot_Type == 2 )
            VFD_Data.Con_Mode  =   User_Data.Control_Mode = 21;              // PMSM - Current Mode
        else if ((User_Data.Control_Mode < 14) && (User_Data.Control_Mode > 10) && VFD_Data.Mot_Type == 1)   // INDM - V/F Open, V/F Close,  FOC
            VFD_Data.Con_Mode  =   User_Data.Control_Mode;
        else
            VFD_Data.Con_Mode  =   User_Data.Control_Mode = 11;             // INDM - V/F Open Loop
}

//------------------------------------------------------------------------------------------------------
void Init_Motor_Params()
{
    // Setting Base Voltage
    if (User_Data.Vrated < Vrated_min)
    {
          MOT_Data.Vb     = Vrated_min/Sqrt3;                 // PMSM_VB/Sqrt3;// Vph = Vll/sqrt(3)
          User_Data.Vrated= Vrated_min;
    }
    else if (User_Data.Vrated > Vrated_max)
    {
          MOT_Data.Vb     = Vrated_max/Sqrt3;                 // PMSM_VB/Sqrt3;// Vph = Vll/sqrt(3)
          User_Data.Vrated= Vrated_max;
    }
    else
      MOT_Data.Vb     = User_Data.Vrated/Sqrt3;           // P//   MOT_Data.Vb     = (User_Data.Vrated*Sqrt2)/(Sqrt3);


    // Setting Base Current
    MOT_Data.Ib         = (User_Data.Prated*1000)/(User_Data.Vrated*Sqrt3);

    if (User_Data.Irated < MOT_Data.Ib)
        MOT_Data.Ib     = User_Data.Irated = MOT_Data.Ib;                      // Rated Current, Max at 0.7 PF, Min at 1 PF
    else if (User_Data.Irated > (MOT_Data.Ib/PowFac_min))
        MOT_Data.Ib     = User_Data.Irated = MOT_Data.Ib/PowFac_min;           // Rated Current, Max at 0.7 PF, Min at 1 PF
    else
        MOT_Data.Ib     = User_Data.Irated;                 // Rated Current, Max at 0.7 PF, Min at 1 PF

        MOT_Data.Ib     = MOT_Data.Ib * Sqrt2;              // Irms pk taken as Base for Control System


        MOT_Data.Ib_1   = (float32) 1/MOT_Data.Ib;
        MOT_Data.Zb     = (float32) MOT_Data.Vb*MOT_Data.Ib_1; // Vb/Ib
        MOT_Data.Zb_1   = (float32) 1/MOT_Data.Zb;

    // Setting Base Frequency
    if (User_Data.Frated < Frated_min)
        MOT_Data.Wb     = (float32) PIx2 * Frated_min;              // 2*pi*Fb
    else if (User_Data.Frated > Frated_max)
        MOT_Data.Wb     = (float32) PIx2 * Frated_max;              // 2*pi*Fb
    else
        MOT_Data.Wb     = (float32) PIx2 * User_Data.Frated;        // 2*pi*Fb
        MOT_Data.Wb_1   = (float32) 1 / MOT_Data.Wb;                // 1/Wb
        MOT_Data.Lb     = (float32) MOT_Data.Zb * MOT_Data.Wb_1;    // Zb/Wb
        MOT_Data.Lb_1   = (float32) 1/ MOT_Data.Lb;                 // 1/Lb

    // Setting Rated Speed & Torque
        MOT_Data.Nrated = User_Data.Nrated;
        MOT_Data.Trated = (float32)((User_Data.Prated) / (PIx2 * MOT_Data.Nrated) * 1000 * 60);

    // Setting No of Poles
       MOT_Data.Poles   =   (MOT_Data.Wb * 120.0)/ (PIx2 * MOT_Data.Nrated );
       if (MOT_Data.Poles < 4)
           MOT_Data.Poles   = 2;
       else if (MOT_Data.Poles < 6)
           MOT_Data.Poles   = 4;
       else if (MOT_Data.Poles < 8)
           MOT_Data.Poles   = 6;
       else if (MOT_Data.Poles < 10)
           MOT_Data.Poles   = 8;
       else if (MOT_Data.Poles < 12)
           MOT_Data.Poles   = 10;
       else
           MOT_Data.Poles   = 12;

     // Setting Synchronous Speed
       MOT_Data.Nsynch  = (MOT_Data.Wb * 120.0)/ (PIx2 * MOT_Data.Poles );

//------------------------------------------------------------------------------------------------------
// Induction Motor Parameters
//------------------------------------------------------------------------------------------------------	
    if(VFD_Data.Mot_Type == 1)  // INDM
    {
        MOT_Data.Rs		= User_Data.Rs;
        MOT_Data.Rr		= User_Data.Rr;
        MOT_Data.Ls		= User_Data.Ls;
        MOT_Data.Lr		= User_Data.Lr;
        MOT_Data.Lm		= User_Data.Lm;
		MOT_Data.Lm_1	= (float32) 1/MOT_Data.Lm;

        MOT_Data.Sigma  = (float32) (MOT_Data.Lm * MOT_Data.Lm);            // 1 - Lm*Lm/(Ls*Lr)
        MOT_Data.Sigma  = 1 - (float32) (MOT_Data.Sigma / (MOT_Data.Ls * MOT_Data.Lr));

        MOT_Data.SigmaS = (float32)((MOT_Data.Ls - MOT_Data.Lm) * MOT_Data.Lm_1);
        MOT_Data.SigmaR = (float32)((MOT_Data.Lr - MOT_Data.Lm) * MOT_Data.Lm_1);

        MOT_Data.Kt     = (float32) (MOT_Data.Lm / (1 + MOT_Data.SigmaR));  // 3P/4*Lm/(1+SigmaR)
        MOT_Data.Kt     = (float32) 0.75 * MOT_Data.Poles * MOT_Data.Kt;
    }
//------------------------------------------------------------------------------------------------------
// PM Sync Motor Parameters
//------------------------------------------------------------------------------------------------------	
	if (VFD_Data.Mot_Type == 2) // PMSM
    {
        MOT_Data.Rs     = User_Data.Rdc;
        MOT_Data.Ld     = User_Data.Ld;
        MOT_Data.Lq     = User_Data.Lq;
        MOT_Data.Lmdf   = (float32) Sqrt2 * Sqrt3_1 * User_Data.Ebc * 60;   // sqrt(2)/sqrt(3)*Ebc / (1000 * 0.5*P * 2*PI/60)
        MOT_Data.Lmdf   = (float32) MOT_Data.Lmdf / (1000 * 0.5* MOT_Data.Poles * PIx2);

        MOT_Data.Kt     = (float32) 0.75 * MOT_Data.Poles * MOT_Data.Lmdf;
    }


//------------------------------------------------------------------------------------------------------
// INDM Parameters in pu
//------------------------------------------------------------------------------------------------------
	if(VFD_Data.Mot_Type == 1)  // INDM
	{
	    MOT_Data.Rspu   = _IQmpy(_IQ(MOT_Data.Rs), _IQ(MOT_Data.Zb_1));     // Rs/Zb
        MOT_Data.Rrpu   = _IQmpy(_IQ(MOT_Data.Rr), _IQ(MOT_Data.Zb_1));     // Rr/Zb
        MOT_Data.Lspu   = _IQmpy(_IQ(MOT_Data.Ls), _IQ(MOT_Data.Lb_1));     // Ls/Wb
        MOT_Data.Lrpu   = _IQmpy(_IQ(MOT_Data.Lr), _IQ(MOT_Data.Lb_1));     // Lr/Wb
        MOT_Data.Lmpu   = _IQmpy(_IQ(MOT_Data.Lm), _IQ(MOT_Data.Lb_1));     // Lm/Wb
        MOT_Data.Tst    = _IQmpy(_IQ(MOT_Data.Sigma), _IQ(MOT_Data.Ls));    // SigmaLs/Rs
        MOT_Data.Tst    = _IQdiv(MOT_Data.Tst, _IQ(MOT_Data.Rs));
        MOT_Data.Trt    = _IQdiv(_IQ(MOT_Data.Lr), _IQ(MOT_Data.Rr));       // Lr/Rr
        MOT_Data.Trt_Imr  = _IQdiv(VFD_Data.Ts,MOT_Data.Trt);
        MOT_Data.Trt_Imr_fbk = _IQ(1.0) - MOT_Data.Trt_Imr;

	}
//------------------------------------------------------------------------------------------------------
// PMSM Parameters in pu
//------------------------------------------------------------------------------------------------------
	if(VFD_Data.Mot_Type == 2)  // PMSM
	{
	    MOT_Data.Rspu   = _IQmpy(_IQ(MOT_Data.Rs), _IQ(MOT_Data.Zb_1));     // Rs/Zb
        MOT_Data.Ldpu   = _IQmpyIQX(_IQ20(MOT_Data.Lb_1), 20, _IQ(MOT_Data.Ld), GLOBAL_Q);// Ld/Lb
        MOT_Data.Lqpu   = _IQmpyIQX(_IQ20(MOT_Data.Lb_1), 20, _IQ(MOT_Data.Lq), GLOBAL_Q);// Lq/Lb
        MOT_Data.Lmdfpu = _IQmpy(_IQ(MOT_Data.Lmdf), _IQ(MOT_Data.Zb_1));
        MOT_Data.Lmdfpu = _IQmpyIQX(MOT_Data.Lmdfpu, GLOBAL_Q,_IQ20(MOT_Data.Wb), 20);
        MOT_Data.Lmdfpu = _IQmpy(MOT_Data.Lmdfpu, _IQ(MOT_Data.Ib_1));
        MOT_Data.Tst    = _IQdiv( _IQ(MOT_Data.Ld), _IQ(MOT_Data.Rs));      // Ld/Rs
	}
	 // Modulation Index Setting
	    if (User_Data.Mod_Ind < ModInd_min)
	        VFD_Data.Mod_Ind    = User_Data.Mod_Ind = ModInd_min;
	    else if (User_Data.Mod_Ind > ModInd_max)
	        VFD_Data.Mod_Ind    = User_Data.Mod_Ind = ModInd_max;
	    else
	        VFD_Data.Mod_Ind    = User_Data.Mod_Ind;

	 // Motor Direction Setting
	    if (User_Data.Mot_Rot > 1)
	        VFD_Data.Mot_Rot = User_Data.Mot_Rot = 1;
	    else
	        VFD_Data.Mot_Rot = User_Data.Mot_Rot;

}
//------------------------------------------------------------------------------------------------------
void Init_VFD_Data_Vars()
{
//------------------------------------------------------------------------------------------------------
// Bandwidth of Current PI Controller
//------------------------------------------------------------------------------------------------------
        VFD_Data.FbwIs      = _IQ20((float32) (VFD_Data.Sw_Freq*1000) / 15 );
                                                                    // Base Frequency * 4
        VFD_Data.TbwIs      = _IQ((float32) 1 / _IQ20toF(VFD_Data.FbwIs));
        VFD_Data.TbwIs      = _IQdiv(VFD_Data.TbwIs, _IQ(PIx2));
        VFD_Data.GridFc    = _IQ20(20.0);

//------------------------------------------------------------------------------------------------------
// Initialise Low Pass Filters for Flux & Speed Estimation
//------------------------------------------------------------------------------------------------------
  if (VFD_Data.Mot_Type == 1)
 {
      MOT_FSE.K    = _IQ(0.1);
    VFD_Data.FluxEstFc  = _IQmpy(_IQ(User_Data.Frated), MOT_FSE.K); //_IQ(5.0);     // Flux  Est LP Filter Corner Freq
    VFD_Data.SpeedEstFc = _IQmpy(_IQ(User_Data.Frated), _IQ(0.1)); //_IQ(5.0);     // Speed Est LP Filter Corner Freq


    VFD_Data.VdcFc      = _IQ20(5.0);  // Vdc LP Filter Corner Freq
    VFD_Data.HskTemFc   =  _IQ20(30.0);
    VFD_Data.DAxisFc   = _IQ20(10.0);





 //   VFD_Data.FluxEstFc  = _IQ(7.0) ;
 //   VFD_Data.SpeedEstFc = _IQ(7.0) ;

 /*   VFD_Data.FbwWm      = VFD_Data.FluxEstFc + VFD_Data.SpeedEstFc ; //_IQ(2.88);//_IQmpy(_IQ(User_Data.Frated), _IQ(0.1));

    VFD_Data.FbwWm      = _IQmpy(VFD_Data.FbwWm,_IQ(.25));   // ( fc +  fsf )/(4*1.732*zeta^2)

    VFD_Data.FbwWm      = _IQmpy( VFD_Data.FbwWm,_IQ(Sqrt3_1));

    VFD_Data.FbwWm      = _IQmpy( VFD_Data.FbwWm,_IQ(2.0)); */

    VFD_Data.FbwWm =  _IQ(2.88);

  //  VFD_Data.FbwWm      = _IQ(3.0);

          // FbwIs / 100
    VFD_Data.TbwWm      = _IQ((float32) 1 / _IQtoF(VFD_Data.FbwWm));
    VFD_Data.TbwWm      = _IQdiv(VFD_Data.TbwWm, _IQ(PIx2));

            //------------------------------------------------------------------------------------------------------
            // Bandwidth of Imr PI Controller
            //------------------------------------------------------------------------------------------------------
    VFD_Data.FbwImr      = _IQ20mpy(VFD_Data.FbwIs, _IQ20(0.025));
                                                                               // FbwIs / 40
    VFD_Data.TbwImr      = _IQ((float32) 1 / _IQ20toF(VFD_Data.FbwImr));
    VFD_Data.TbwImr      = _IQdiv(VFD_Data.TbwImr, _IQ(PIx2));

 }

  if (VFD_Data.Mot_Type == 2)
  {
   VFD_Data.FluxEstFc  = _IQ(10.0);     // Flux  Est LP Filter Corner Freq
   VFD_Data.SpeedEstFc = _IQ(10.0);     // Speed Est LP Filter Corner Freq
   VFD_Data.VdcFc      = _IQ20(30.0);  // Vdc LP Filter Corner Freq
   VFD_Data.HskTemFc   =  _IQ20(30.0);

   VFD_Data.FbwWm      = _IQmpy(_IQ(User_Data.Frated), _IQ(0.1));
   VFD_Data.TbwWm      = _IQ((float32) 1 / _IQtoF(VFD_Data.FbwWm));
   VFD_Data.TbwWm      = _IQdiv(VFD_Data.TbwWm, _IQ(PIx2));
  }

//------------------------------------------------------------------------------------------------------
// DC Bus Voltage Scaling
//------------------------------------------------------------------------------------------------------
        VFD_Data.VdcRef     = (float32)(MOT_Data.Vb * Sqrt3 / (PWMUtilFac * VFD_Data.Mod_Ind));     // VdcRef   = Vb*Sqrt(3)/(PWMUtilFac*MI)
        VFD_Data.InvGain_1  = (float32)(2 / VFD_Data.VdcRef);       // Inv Gain = VdcRef

        VFD_IntAdcData.VdcVlt_Scal = _IQ((float32) (IntADC_Vmax)/(Vdc_ADC * IntDigValmax )); // Vdc_Nom showing...
        VFD_IntAdcData.Volt_Scal     = _IQ((float32) (IntADC_Vmax)/(Voltage_ADC * IntDigValmax ));

        VFD_IntAdcData.Volt_grid_Scal     = _IQ((float32) (IntADC_Vmax)/(Voltage_ADC_Grid * IntDigValmax ));


   // Filter for Vdc // This shall be eliminated..
       VFD_Data.VdcTc      = _IQ15mpyIQX(_IQ(PIx2), GLOBAL_Q, VFD_Data.VdcFc, 20);
       VFD_Data.VdcTc      = _IQ15mpyIQX(VFD_Data.Ts, GLOBAL_Q, VFD_Data.VdcTc, 15);

       VFD_Data. DAxisTc      = _IQ15mpyIQX(_IQ(PIx2), GLOBAL_Q, VFD_Data. DAxisFc, 20);
       VFD_Data. DAxisTc      = _IQ15mpyIQX(VFD_Data.Ts, GLOBAL_Q, VFD_Data. DAxisTc, 15);

       VFD_Data. GridTc      = _IQ15mpyIQX(_IQ(PIx2), GLOBAL_Q, VFD_Data. GridFc, 20);
       VFD_Data. GridTc      = _IQ15mpyIQX(VFD_Data.Ts, GLOBAL_Q, VFD_Data. GridTc, 15);

       VFD_Data.HskTemTc      = _IQ15mpyIQX(_IQ(PIx2), GLOBAL_Q, VFD_Data.HskTemFc, 20);
       VFD_Data.HskTemTc      = _IQ15mpyIQX(VFD_Data.Tsdc, GLOBAL_Q, VFD_Data.HskTemTc, 15);

//------------------------------------------------------------------------------------------------------
// Current & PT Scaling
//------------------------------------------------------------------------------------------------------
   //    VFD_IntAdcData.offset_counts =
       VFD_IntAdcData.Curr_Scal = _IQ((float32)((IntADC_Vmax)/(Actual_Curr_Value))/(IntDigValmax));
   //    VFD_IntAdcData.Curr_Scal = _IQmpy(VFD_IntAdcData.Curr_Scal,_IQ(0.95));
       //   VFD_IntAdcData.Curr_Scal     = _IQ((float32) LEM_Irms * Sqrt2 / ( DigVal_cur * MOT_Data.Ib));    // Check Excel Sheet
        #if (INDMotorSpec == INDM_03k7)
       VFD_IntAdcData.Curr_Scal = _IQ(0.00644315); // for 3kW Motor
       #endif
       VFD_IntAdcData.Idc_Curr_Scal =_IQ((float32)((IntADC_Vmax)/(Actual_DC_Curr_Value))/(IntDigValmax));
 //      VFD_IntAdcData.PT_Scal   = _IQ((float32)  (IntADC_Vmax)  / (PTsndV*IntDigValmax));     // Check Excel Sheet



//------------------------------------------------------------------------------------------------------
// INDM Controller Limit Calculations
//------------------------------------------------------------------------------------------------------
    if (VFD_Data.Mot_Type == 1) // INDM
    {
        // Calculate the actual Vdq & Idq Maximum values
                VFD_Data.IsdNorm= (float32)(MOT_Data.Vb*MOT_Data.Wb_1*1.35*Sqrt2)/(MOT_Data.Lm);
                VFD_Data.VdqMax  = (float32)(Sqrt3 * MOT_Data.Vb);    // VdqMax = sqrt(3/2)*Sqrt(2) = sqrt(3)
                                                                                    // VdqMax = sqrt(3)*Vb
                VFD_Data.IsdMax  = (float32)(VFD_Data.VdqMax / MOT_Data.Ls);
                VFD_Data.IsdMax  = (float32)(VFD_Data.IsdMax / MOT_Data.Wb);
                                                                                    // IsdMax = VdqMax/(Ls*Wb)
                VFD_Data.IsqMax  = (float32)(MOT_Data.Trated * 6 * (1 + MOT_Data.SigmaR));
                VFD_Data.IsqMax  = (float32)(VFD_Data.IsqMax / (2 * MOT_Data.Poles * MOT_Data.Lm * VFD_Data.IsdMax));
                                                                                    // IsqMax = Trated*6*(1+SigmaR)/(2*Poles*Lm*IsdMax) ???
            // Initialise IsD & IsQ LIMITS
                VFD_Data.IsdLim  = _IQmpy(_IQ(VFD_Data.IsdMax), _IQ(1.25));
                VFD_Data.IsdLim  = _IQmpy(VFD_Data.IsdLim, _IQ(MOT_Data.Ib_1));
                VFD_Data.IsdLim  = _IQabs(VFD_Data.IsdLim);          // IsdLim = IsdMax*1.25/Ib
                VFD_Data.IsqLim  = _IQmpy(_IQ(VFD_Data.IsqMax), _IQ(MOT_Data.Ib_1));
                VFD_Data.IsqLim  = _IQabs(VFD_Data.IsqLim);          // IsqLim = IsqMax/Ib

            // Initialise VsD & VsQ Maximum values
                VFD_Data.VsdMax  = (float32)(1.25*MOT_Data.Sigma*MOT_Data.Ls*MOT_Data.Wb*VFD_Data.IsqMax);
                                                                     // VsdMax = Sigma*Ls*Wb*IsqMax
                VFD_Data.VsdMax  = (float32)(MOT_Data.Rs * VFD_Data.IsdMax) - VFD_Data.VsdMax;
                                                                     // VsdMax = Rs*IsdMax - Sigma*Ls*Wb*IsqMax

                VFD_Data.VsqMax  = (float32)(MOT_Data.Sigma*MOT_Data.Ls*MOT_Data.Wb*VFD_Data.IsdMax);
                                                                     // VsqMax = Sigma*Ls*Wb*IsdMax
                VFD_Data.VsqMax  = VFD_Data.VsqMax + (float32)(MOT_Data.Lm*MOT_Data.Wb*VFD_Data.IsdMax / (1 + MOT_Data.SigmaR));
                                                                     // VsqMax = Sigma*Ls*Wb*IsdMax + Lm*Wb*IsdMax/(1+SigmaR)
                VFD_Data.VsqMax  = (float32)(MOT_Data.Rs * VFD_Data.IsqMax) + VFD_Data.VsqMax;
                                                                     // VsqMax = Rs*IsdMax + Sigma*Ls*Wb*IsdMax + Lm*Wb*IsdMax/(1+SigmaR)

            // Initialise VsD & VsQ LIMITS

                VFD_Data.VsdLim     = _IQmpyIQX(_IQ20(VFD_Data.VsdMax), 20, _IQ(VFD_Data.InvGain_1), GLOBAL_Q);
                VFD_Data.VsdLim     = _IQabs(VFD_Data.VsdLim);              // VsdLim = VsdMax/InvGain_1
                VFD_Data.VsqLim     = _IQmpyIQX(_IQ20(VFD_Data.VsqMax), 20, _IQ(VFD_Data.InvGain_1), GLOBAL_Q);
                VFD_Data.VsqLim     = _IQabs(VFD_Data.VsqLim);              // VsqLim = VsqMax/InvGain_1

    }
//------------------------------------------------------------------------------------------------------
// PMSM Controller Limit Calculations
//------------------------------------------------------------------------------------------------------
    if (VFD_Data.Mot_Type == 2) // PMSM
    {

////////////////////////////////////////////////////////////////

        // Calculate the actual Vdq & Idq Maximum values
               VFD_Data.VdqMax  = (float32)(Sqrt3 * MOT_Data.Vb);    // VdqMax = sqrt(3/2)*Sqrt(2) = sqrt(3)
                                                                                   // VdqMax = sqrt(3)*Vb
               VFD_Data.IsdMax  = 0;
                                                                                   // IsdMax = 0
               VFD_Data.IsqMax  = (float32)(MOT_Data.Trated * 4);
               VFD_Data.IsqMax  = (float32)(VFD_Data.IsqMax / (3* MOT_Data.Poles * MOT_Data.Lmdf));
                                                                                   // IsqMax =  Te*4 / (3*P*Lmdf)
           // Initialise IsD & IsQ LIMITS
               VFD_Data.IsdLim  = _IQ(0.0);                         // IsdLim = 0
               VFD_Data.IsqLim  = _IQmpy(_IQ(VFD_Data.IsqMax), _IQ(1.25));
               VFD_Data.IsqLim  = _IQmpy(VFD_Data.IsqLim, _IQ(MOT_Data.Ib_1));
               VFD_Data.IsqLim  = _IQabs(VFD_Data.IsqLim);          // IsqLim = IsqMax/Ib

           // Initialise VsD & VsQ Maximum values
               VFD_Data.VsdMax  = (float32)(-MOT_Data.Lq*MOT_Data.Wb*VFD_Data.IsqMax);
                                                                    // VsdMax = -Lq*Wb*IsqMax
               VFD_Data.VsqMax  = (float32)(MOT_Data.Wb*MOT_Data.Lmdf);

               VFD_Data.VsqMax  = (float32)(MOT_Data.Rs * VFD_Data.IsqMax) + VFD_Data.VsqMax;
                                                                    // Vsq = Rs*Isq max + Wb*Lmdf

           // Initialise VsD & VsQ LIMITS

               VFD_Data.VsdLim     = _IQmpyIQX(_IQ20(VFD_Data.VsdMax), 20, _IQ(VFD_Data.InvGain_1), GLOBAL_Q);
               VFD_Data.VsdLim     = _IQabs(VFD_Data.VsdLim);              // VsdLim = VsdMax/InvGain_1
               VFD_Data.VsqLim     = _IQmpyIQX(_IQ20(VFD_Data.VsqMax), 20, _IQ(VFD_Data.InvGain_1), GLOBAL_Q);
               VFD_Data.VsqLim     = _IQabs(VFD_Data.VsqLim);              // VsqLim = VsqMax/InvGain_1


    }

//------------------------------------------------------------------------------------------------------
// Initialise References
//------------------------------------------------------------------------------------------------------
       VFD_Data.VsdRef = _IQ(0.00);        // Vsd reference (pu), Flux Reference V Command
       VFD_Data.VsqRef = _IQ(0.00);        // Vsq reference (pu), Torque Reference V Command
       VFD_Data.IsdRef = _IQ(0.00);        // Isd reference (pu), Flux Reference I Command
       VFD_Data.IsqRef = _IQ(0.00);        // Isq reference (pu), Torque Reference I Command

//------------------------------------------------------------------------------------------------------
// Initialise VFD Data Variables
//------------------------------------------------------------------------------------------------------
        VFD_Data.IsAlpha    = VFD_Data.IsBeta   = 0;
        VFD_Data.Isd        = VFD_Data.Isq      = 0;

        VFD_Data.Imr    = 0;
        VFD_Data.VrRef  = VFD_Data.VrRef_   = 0;
        VFD_Data.VyRef  = VFD_Data.VyRef_   = 0;
        VFD_Data.VbRef  = VFD_Data.VbRef_   = 0;

        VFD_Data.VsdRef = VFD_Data.VsqRef   = 0;
        VFD_Data.VsdRefTmp = VFD_Data.VsqRefTmp   = 0;

        VFD_Data.VsAlphaRef     = VFD_Data.VsBetaRef    = 0;

        VFD_Data.SineEpsilon    = _IQ(1.0);
        VFD_Data.CosEpsilon     = 0;
        VFD_Data.SineTheta      = _IQ(1.0);
        VFD_Data.CosTheta       = 0;

        VFD_Data.SpeedAct       = VFD_Data.SpeedRmp = 0;

//------------------------------------------------------------------------------------------------------
// INDM Flux & Torq reference Calculation
//------------------------------------------------------------------------------------------------------
    if (VFD_Data.Mot_Type == 1)
    {
    // Initialise Flux for INDM
        VFD_Data.FluxRef = _IQmpy(_IQ(VFD_Data.IsdNorm), _IQ(MOT_Data.Ib_1));

        if(User_Data.Energy_Efficient ==1)
        VFD_Data.Energy_Efficient =1;
        else
        VFD_Data.Energy_Efficient =0;

    }
//------------------------------------------------------------------------------------------------------
// PMSM Flux & Torq reference for Current Mode
//------------------------------------------------------------------------------------------------------
    if (VFD_Data.Mot_Type == 2)
    {
//    // Initial Flux for PMSM starting in Current Mode
    if (User_Data.TorqStrt < 10)
    User_Data.TorqStrt = 10;
    else if (User_Data.TorqStrt > 40)
    User_Data.TorqStrt = 40;


    VFD_Data.FluxRef  = _IQ(0.0);// _IQmpy(VFD_Data.FluxRef, _IQ(MOT_Data.Ib_1));   // FluxRef = IsqMax/Ib
    VFD_Data.TorqRef  = _IQmpy(_IQ(User_Data.TorqStrt),_IQ(.01)); // Conversion to per unit
    VFD_Data.Torqlower = _IQmpy(VFD_Data.TorqRef,_IQ(.5));
	// Initial Torq for PMSM starting in Current Mode
/*        if (User_Data.TorqStrt < 00)
            VFD_Data.TorqRef = User_Data.TorqStrt = _IQ(0.0);
        else if (User_Data.TorqStrt > 50)
            VFD_Data.TorqRef = User_Data.TorqStrt = _IQmpy(_IQ(VFD_Data.IsqMax), _IQ(0.5));
        else
            VFD_Data.TorqRef = _IQmpy(_IQ(VFD_Data.IsqMax), _IQ(User_Data.TorqStrt * 0.01));

        VFD_Data.TorqRef  = _IQmpy(VFD_Data.TorqRef, _IQ(MOT_Data.Ib_1));   // TorqRef = IsqMax/Ib
*/
    }

//------------------------------------------------------------------------------------------------------
// Speed reference Calculation
//------------------------------------------------------------------------------------------------------
    if (User_Data.Refip_Srce == 0)
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
            VFD_Data.SpeedRef = _IQ(1.00);               // INDM Flux Weakening Region (TEMP limited to 1.0)
            User_Data.Op_Freq = 1.00 * MOT_Data.Wb / PIx2;
        }
        else
            VFD_Data.SpeedRef = _IQ((float32) User_Data.Op_Freq / (MOT_Data.Wb / PIx2));
    }
    else
        VFD_Data.SpeedRef = _IQ(0.0);

    if (User_Data.Refip_Srce == 02)
    {
    if(User_Data.AnaVoltMin < 0)
        User_Data.SpdAnaMin = 0;  //
    else if(User_Data.AnaVoltMin > SPD_Vsp )
        User_Data.SpdAnaMax =  SPD_Vsp; // Maximum Value is Hardware Set

    VFD_Data.AnaMin = _IQdiv(_IQ(User_Data.AnaVoltMin),_IQ(SPD_Vsp)); // Minimum analog input value

    VFD_Data.AnaMax = _IQdiv(_IQ(User_Data.AnaVoltMax),_IQ(SPD_Vsp)); // Max Analog value

    VFD_Data.SpdMin =  VFD_Data.SpdMin = _IQ((float32)(User_Data.SpdAnaMin/(User_Data.Nrated)));

    VFD_Data.SpdMax = _IQ((float32)(User_Data.SpdAnaMax/(User_Data.Nrated)));

    if((VFD_Data.SpdMin) < _IQ(0.5))
    {
        User_Data.SpdAnaMin = .5 * User_Data.Nrated ;

        VFD_Data.SpdMin = _IQ(.5);
    }// Minimum
    else if((VFD_Data.SpdMin) > _IQ(1.0))
    {
        User_Data.SpdAnaMax = User_Data.Nrated;
        VFD_Data.SpdMax = _IQ(1.0);
    }

       VFD_Data.SpdAnalConstant = _IQdiv((VFD_Data.SpdMax - VFD_Data.SpdMin),(VFD_Data.AnaMax-VFD_Data.AnaMin));
   }


}
//------------------------------------------------------------------------------------------------------
void Init_VFD_Data_Reset()
{

    //------------------------------------------------------------------------------------------------------
    // Initialise VFD_Data Variables
    //------------------------------------------------------------------------------------------------------
  /*  if (VFD_Data.Mot_Type == 1) // INDM
    {
    // Filter for IsD
  //      VFD_Data.IsdFilt    =  VFD_Data.IsdFiltOld  = 0;

    // Filter for IsQ
 //       VFD_Data.IsqFilt    =  VFD_Data.IsqFiltOld  = 0;
    }*/


    //------------------------------------------------------------------------------------------------------
    // Initialise References
    //------------------------------------------------------------------------------------------------------
       VFD_Data.VsdRef = _IQ(0.00);        // Vsd reference (pu), Flux Reference V Command
       VFD_Data.VsqRef = _IQ(0.00);        // Vsq reference (pu), Torque Reference V Command
       VFD_Data.VsdRefTmp = _IQ(0.00);
       VFD_Data.VsqRefTmp = _IQ(0.00);

       VFD_Data.IsdRef = _IQ(0.00);        // Isd reference (pu), Flux Reference I Command
       VFD_Data.IsqRef = _IQ(0.00);        // Isq reference (pu), Torque Reference I Command

    //------------------------------------------------------------------------------------------------------
    // Initialise VFD Data Variables
    //------------------------------------------------------------------------------------------------------
        VFD_Data.IsAlpha    = VFD_Data.IsBeta   = 0;
        VFD_Data.Isd        = VFD_Data.Isq      = 0;

        VFD_Data.Imr    = 0;
        VFD_Data.VrRef  = VFD_Data.VrRef_   = 0;
        VFD_Data.VyRef  = VFD_Data.VyRef_   = 0;
        VFD_Data.VbRef  = VFD_Data.VbRef_   = 0;

        VFD_Data.VsdRef = VFD_Data.VsqRef   = 0;

        VFD_Data.VsAlphaRef     = VFD_Data.VsBetaRef = 0;

        VFD_Data.SineEpsilon    = _IQ(1.0);
        VFD_Data.CosEpsilon     = 0;
        VFD_Data.SineTheta      = _IQ(1.0);
        VFD_Data.CosTheta       = 0;

        VFD_Data.SpeedAct       = VFD_Data.SpeedRmp = 0;


    //------------------------------------------------------------------------------------------------------
    // Initialise Ramp Controller Structure
    //------------------------------------------------------------------------------------------------------
        MOT_RC.Freq             = 0;
        MOT_RC.RampOut          = 0;
        MOT_RC.Angle            = 0;
        MOT_RG.DelayCnt         = 0;
        MOT_RGI.DelayCnt        =0;
        MOT_RG.Tmp              = 0;
        MOT_RGI.Tmp             =0;
        MOT_RG.SetValue         = 0;
        MOT_RGI.SetValue        =0;
        MOT_RG.EqualFlag        = 0;
        MOT_RG.TargVal          = _IQ(0.0);
        MOT_RGI.EqualFlag        = 0;
        MOT_RGI.TargVal          = _IQ(0.0);


    //------------------------------------------------------------------------------------------------------
    // Initialise Flux Ramp Controller Structure
    //------------------------------------------------------------------------------------------------------
        MOT_RGQ.DelayCnt    = 0;
        MOT_RGQ.Tmp         = 0;
        MOT_RGQ.SetValue    = 0;
        MOT_RGQ.EqualFlag   = 0;
        MOT_RGQ.TargVal     = _IQ(0.0);


    //------------------------------------------------------------------------------------------------------
    //Initialise VbyF Profile Structure
    //------------------------------------------------------------------------------------------------------
        MOT_VF.Freq     = 0;
        MOT_VF.FreqOut  = 0;
        MOT_VF.VoltOut  = 0;
        MOT_VF.VfSlope  = 0;
        MOT_VF.AbsFreq  = 0;


    //------------------------------------------------------------------------------------------------------
    // Initialise Flux & Speed Estimation Variables
    //------------------------------------------------------------------------------------------------------
        MOT_FSE.CosThetaOld     = _IQ(1.0);
        MOT_FSE.SineThetaOld    = 0;
        MOT_FSE.CosTheta        = _IQ(1.0);
        MOT_FSE.SineTheta       = 0;

        MOT_FSE.VsAlpha  = MOT_FSE.VsBeta   = 0;
        MOT_FSE.EsAlpha  = MOT_FSE.EsBeta   = 0;
        MOT_FSE.IsAlpha  = MOT_FSE.IsBeta   = 0;
        MOT_FSE.Isd      = MOT_FSE.Isq      = 0;

        MOT_FSE.PsisAlpha   = MOT_FSE.PsisBeta      = 0;
        MOT_FSE.PsisAlphaOld= MOT_FSE.PsisBetaOld   = 0;
        MOT_FSE.PsirAlpha   = MOT_FSE.PsirBeta      = 0;

        MOT_FSE.WEst        = MOT_FSE.WSlip         = 0;
        MOT_FSE.WSyn        = MOT_FSE.WSyntemp      = 0;
        MOT_FSE.WEstFilt    = MOT_FSE.WEstFiltOld   = 0;
        MOT_FSE.WSlip       = MOT_FSE.WSyn          = 0;

    Init_PIcntrl_Vars();        // Initialise PI Control
    Init_RotPosEst_Vars();      // Initialise Rotor position Estimation
    Init_DynBrake_Vars();       // Initialise Dynamic Brake Control Variables

}
//------------------------------------------------------------------------------------------------------
void Init_FreqRamp_Vars()
{
//------------------------------------------------------------------------------------------------------
// Initialise Ramp Controller Structure
//------------------------------------------------------------------------------------------------------
        MOT_RC.Freq             = 0;
        MOT_RC.RampOut          = 0;
        MOT_RC.Angle            = 0;
        MOT_RG.DelayCnt         = 0;
        MOT_RG.Tmp              = 0;
        MOT_RG.SetValue         = 0;
        MOT_RG.EqualFlag        = 0;
        MOT_RGI.DelayCnt         = 0;
        MOT_RGI.Tmp              = 0;
        MOT_RGI.SetValue         = 0;
        MOT_RGI.EqualFlag        = 0;
//------------------------------------------------------------------------------------------------------
// Initialise Ramp Controller Parameters
//------------------------------------------------------------------------------------------------------
        MOT_RG.TargVal          = _IQ(0.0);         // (pu) Speed Reference Command = 1.0 * 1500 = 1500rpm (Base Speed = 1500rpm)
        MOT_RGI.TargVal         = _IQ(0.0);
        MOT_RC.StepAng          = _IQmpyIQX(_IQ20((float32)MOT_Data.Wb/PIx2), 20, VFD_Data.Ts, GLOBAL_Q);
                                                    // Base Frequency/Switching Frequency = 50Hz/3kHz
        MOT_RG.HighLim          = _IQ(1.1);           // Limited to +2, 2 times the Base Freq
        MOT_RG.LowLim           = _IQ(0.05);          // Limited to -2, 2 times the Base Freq
        MOT_RGI.HighLim         = _IQ(.5);          // Init Limited to .5, -.5 times the Base Freq
        MOT_RGI.LowLim          = _IQ(-.5);         // Limited to .5, -.5 times the Base Freq

	// Setting Ramp Time
	if ((User_Data.Frated / User_Data.Accel_Rate) < RmpTm_min)
	{
		MOT_RG.Delay            = RmpTm_min;            // Ramp Delay = 10 * 1s (Min Ramp time = 10s)
		User_Data.Accel_Rate    = User_Data.Frated / RmpTm_min;

	}
	else if ((User_Data.Frated / User_Data.Accel_Rate) > RmpTm_max)
	{
		MOT_RG.Delay            = User_Data.Frated / RmpTm_max;            // Ramp Delay = 120 * 1s (Max Ramp time = 120s)
		User_Data.Accel_Rate    = User_Data.Frated / RmpTm_max;
	}
	else
	{
		MOT_RG.Delay		= (User_Data.Frated / User_Data.Accel_Rate);
		MOT_RGI.Delay       = 25; // 1Hz/seconds fpr PMSM .5Hz/sec Delay
		if(MOT_RGI.Delay < RmpTm_min )
		MOT_RGI.Delay = RmpTm_min ;
	}
}
//------------------------------------------------------------------------------------------------------
void Init_VsdRamp_Vars()
{
//------------------------------------------------------------------------------------------------------
// Initialise Trq Ramp Controller Structure
//------------------------------------------------------------------------------------------------------
        MOT_RGD.DelayCnt    = 0;
        MOT_RGD.Tmp         = 0;
        MOT_RGD.SetValue    = 0;
        MOT_RGD.EqualFlag   = 0;

//------------------------------------------------------------------------------------------------------
// Initialise Trq Ramp Controller Parameters
//------------------------------------------------------------------------------------------------------
        MOT_RGD.TargVal     = _IQ(0.0);     // (pu) Flux Reference Command
        MOT_RGD.HighLim     = VFD_Data.VsqLim;
        MOT_RGD.LowLim      = _IQ(0.0);

}
void Init_VsqRamp_Vars()
{
//------------------------------------------------------------------------------------------------------
// Initialise Vsq Ramp Controller Structure
//------------------------------------------------------------------------------------------------------
        MOT_RGQ.DelayCnt    = 0;
        MOT_RGQ.Tmp         = 0;
        MOT_RGQ.SetValue    = 0;
        MOT_RGQ.EqualFlag   = 0;

//------------------------------------------------------------------------------------------------------
// Initialise Vsq Ramp Controller Parameters
//------------------------------------------------------------------------------------------------------
        MOT_RGQ.TargVal     = _IQ(0.0);     // (pu) Flux Reference Command
        if (User_Data.Control_Mode  == 12)
        {
            MOT_RGQ.HighLim = _IQ(0.5);     // Limited to +0.5, for V/F closed loop
            MOT_RGQ.LowLim  = _IQ(-0.5);    // Limited to -0.5, for V/F closed loop
        }
        else
        {
            MOT_RGQ.HighLim = VFD_Data.VsqLim;
            MOT_RGQ.LowLim  = -VFD_Data.VsqLim;
        }

}
void Init_VbyF_Vars()
{
//------------------------------------------------------------------------------------------------------
//Initialise VbyF Profile Structure
//------------------------------------------------------------------------------------------------------
        MOT_VF.Freq     = 0;
        MOT_VF.FreqOut  = 0;
        MOT_VF.VoltOut  = 0;
        MOT_VF.VfSlope  = 0;
        MOT_VF.AbsFreq  = 0;
//------------------------------------------------------------------------------------------------------
// Initialise VbyF Control Profile Parameters
//------------------------------------------------------------------------------------------------------
        MOT_VF.FreqMin  = _IQ(0.0);        // (pu) Minimum Frequency = 0.04   * 50  = 2Hz,  Base Frequency = 50Hz
        MOT_VF.FreqRated= _IQ(1.0);         // (pu) Rated Frequency   = 1.0    * 50  = 50Hz
        MOT_VF.FreqMax  = _IQ(1.5);         // (pu) Max Frequency     = 1.5    * 50  = 75Hz
        MOT_VF.VoltMin  = _IQ(0.0);        // (pu) Min Voltage       = 0.04   * 400 = 16V,  VoltMin = Is*Rs = 40*0.393 = 15.72V
        MOT_VF.VoltRated= _IQ(1.0);         // (pu) Rated Voltage     = 1.0    * 400 = 400V, Base Voltage  = 400V
        MOT_VF.VfSlope  = _IQdiv((MOT_VF.VoltRated - MOT_VF.VoltMin),(MOT_VF.FreqRated - MOT_VF.FreqMin));
                                            // (pu) Compute slope V/f profile
}
//------------------------------------------------------------------------------------------------------
void Init_PIcntrl_Vars()
{
//------------------------------------------------------------------------------------------------------
//  Initialise PIcontrol Structures
//------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------
// INDM Controller Limit Calculations
//------------------------------------------------------------------------------------------------------
    if (VFD_Data.Mot_Type == 1) // INDM
    {
    //------------------------------------------------------------------------------------------------------
    // VDC P CONTROLLER TUNING for V/F closed Loop
    //------------------------------------------------------------------------------------------------------
        VFD_Data.VdcKp      =_IQ(1.4);      // 2.4646, Cal check Excel sheet. (1.0) => 390V o/p for 405V i/p
                                            //                                (0.8) => 385V o/p for 405V i/p
        VFD_Data.VsqRefDelta= 0;

    //------------------------------------------------------------------------------------------------------
    // IsQ PI CONTROLLER TUNING
    //------------------------------------------------------------------------------------------------------
        PI_Isq.Ki   = _IQmpy(_IQ(MOT_Data.Sigma), _IQ(MOT_Data.Ls));    // Kp = (Sigma*Ls)/(TbwIs*Zb)
        PI_Isq.Ki   = _IQdiv(PI_Isq.Ki, VFD_Data.TbwIs);
        PI_Isq.Kp   = _IQmpy(PI_Isq.Ki, _IQ(MOT_Data.Zb_1));
        PI_Isq.Ki   = _IQmpy(PI_Isq.Ki, VFD_Data.Ts);                   // Ki = (Sigma*Ls*Ts)/(2*TbwIs*Tst*Zb)
        PI_Isq.Ki   = _IQmpy(PI_Isq.Ki, _IQ(MOT_Data.Zb_1));
        PI_Isq.Ki   = _IQdiv(PI_Isq.Ki, MOT_Data.Tst);
        PI_Isq.Ki   = _IQmpy(PI_Isq.Ki, _IQ(0.5));
        PI_Isq.KiInt= PI_Isq.Ki;

        PI_Isq.Umax = VFD_Data.VsqLim;      // PI Saturation Max
        PI_Isq.Umin = -VFD_Data.VsqLim;     // PI Saturation Min

      /*  PI_Isq.LoopCnt      = 1;            // For V/F Closed Loop
        PI_Isq.PreScaler    = 10;           // Slow down IsQ PI Control execution by Prescaler times*/

    //------------------------------------------------------------------------------------------------------
    // IsD PI CONTROLLER TUNING
    //------------------------------------------------------------------------------------------------------
        PI_Isd.Kp   = PI_Isq.Kp;
        PI_Isd.Ki   = PI_Isq.Ki;
        PI_Isd.KiInt= PI_Isd.Ki;

        PI_Isd.Umax = VFD_Data.VsdLim;      // PI Saturation Max
        PI_Isd.Umin = - VFD_Data.VsdLim;     // PI Saturation Min

      /*  PI_Isd.LoopCnt      = 1;            // For V/F Closed Loop
        PI_Isd.PreScaler    = 10;           // Slow down IsD PI Control execution by Prescaler times*/

    //------------------------------------------------------------------------------------------------------
    // SPEED PI CONTROLLER DESIGN

        PI_Spd.Kp   = _IQmpy(_IQ(MOT_Data.Kt), VFD_Data.TbwWm);         // Kp = (Jm/(Kt*IsdMax*TbwWm)) * (Wb/Ib)
        PI_Spd.Kp   = _IQmpy(PI_Spd.Kp, _IQ(VFD_Data.IsdNorm));
        PI_Spd.Kp   =_IQmpy(PI_Spd.Kp,_IQ(MOT_Data.Ib));
        PI_Spd.Kp   = _IQdiv(_IQ(1), PI_Spd.Kp);
        PI_Spd.Ki   = _IQmpyIQX(_IQ23(MOT_Data.Trated),23,PI_Spd.Kp, GLOBAL_Q);
        PI_Spd.Kp   = PI_Spd.KpInt = _IQmpy(_IQ(MOT_Data.Jm), PI_Spd.Kp);
        PI_Spd.Kp   = _IQmpyIQX(_IQ21(MOT_Data.Wb),21,PI_Spd.Kp, GLOBAL_Q);
        PI_Spd.Ki   = _IQmpy(PI_Spd.Ki, VFD_Data.Ts);            // (Poles/2) *(Ts/2)


        PI_Spd.KpInt = PI_Spd.Kp ;
        PI_Spd.KiInt = PI_Spd.Ki ;


        PI_Spd.Umax     = VFD_Data.IsqLim;                // PI Saturation Max, IsqLim
        PI_Spd.Umin     = -VFD_Data.IsqLim;                // PI Saturation Min

     /*   PI_Spd.LoopCnt  = 1;                        // For V/F Closed Loop
        PI_Spd.PreScaler= 1;                       // Slow down Speed PI Control execution by Prescaler times*/


        PI_Imr.Kp =  _IQdiv(_IQ(MOT_Data.Lr),VFD_Data.TbwImr ); // kp = Lr/TbwImr , (2*pi*f*Lr)
        PI_Imr.Ki =  _IQdiv(_IQ(MOT_Data.Rr), VFD_Data.TbwImr); // ki = Rr/TbwImr . (2*pi*f*Rr)
        PI_Imr.Ki =  _IQmpy(VFD_Data.Ts,PI_Imr.Ki);             // Multiplication with Sampling Time
        PI_Imr.KiInt= PI_Imr.Ki;
        VFD_Data.K1 = _IQmpy(_IQ(0.3),VFD_Data.FluxRef);       // 30% Flux Component
        VFD_Data.K2 = _IQmpy(_IQ(1.1),VFD_Data.FluxRef);       // 90% Torque Component

        PI_Imr.Umax = VFD_Data.IsdLim;                         // 1.25*Isd
        PI_Imr.Umin = _IQmpy(VFD_Data.FluxRef,_IQ(0.3));       // Minimum 30% of rated Flux.. 30% is flux required to run at full spped at no load
    }

//------------------------------------------------------------------------------------------------------
// PMSM Controller Limit Calculations
//------------------------------------------------------------------------------------------------------
    if (VFD_Data.Mot_Type == 2) // PMSM
    {
    //------------------------------------------------------------------------------------------------------
    //  IsQ PI CONTROLLER TUNING
    //------------------------------------------------------------------------------------------------------
        PI_Isq.Ki   = _IQmpy(_IQ(MOT_Data.Ld),_IQ(MOT_Data.Zb_1));  // CONTROLLER CONSTANTS CALCULATION BASED ON MACHINE PARAMETERS
        PI_Isq.Ki   = _IQdiv(PI_Isq.Ki, VFD_Data.TbwIs);
        PI_Isq.Kp   = PI_Isq.Ki ;
        PI_Isq.Ki   = _IQmpy(PI_Isq.Ki, VFD_Data.Ts);               // Ki = (Ld*Ts)/(2*TbwIs*Tst*Zb)
        PI_Isq.Ki   = _IQdiv(PI_Isq.Ki, MOT_Data.Tst);
        PI_Isq.Ki   = _IQmpy(PI_Isq.Ki, _IQ(0.5));

        PI_Isq.Umax = _IQ(1.3);       // TEMP giving  1.2 PU
        PI_Isq.Umin = _IQ(-1.3);      // TEMP giving -1.2 PU

    //------------------------------------------------------------------------------------------------------
    // IsD PI CONTROLLER TUNING
    //------------------------------------------------------------------------------------------------------
        PI_Isd.Kp   = PI_Isq.Kp;
        PI_Isd.Ki   = PI_Isq.Ki;

        PI_Isd.Umax = _IQ(1.2);       // TEMP giving  1.2 PU
        PI_Isd.Umin = _IQ(-1.2);      // TEMP giving -1.2 PU

    //------------------------------------------------------------------------------------------------------
    //  SPEED PI CONTROLLER TUNING
    //------------------------------------------------------------------------------------------------------
        PI_Spd.Kp   = _IQmpy(_IQ(MOT_Data.Kt), VFD_Data.TbwWm);     // Kp = (Jm/(Kt*TbwWm)) * (Wb/Ib)
        PI_Spd.Kp   =_IQmpy(PI_Spd.Kp,_IQ(MOT_Data.Ib));
        PI_Spd.Kp   = _IQdiv(_IQ(1), PI_Spd.Kp);
        PI_Spd.Ki   = _IQmpyIQX(_IQ23(MOT_Data.Trated),23,PI_Spd.Kp, GLOBAL_Q);
        PI_Spd.Kp   = PI_Spd.KpInt = _IQmpy(_IQ(MOT_Data.Jm), PI_Spd.Kp);
        PI_Spd.Kp   = _IQmpyIQX(_IQ21(MOT_Data.Wb),21,PI_Spd.Kp, GLOBAL_Q);
        PI_Spd.Ki   = _IQmpy(PI_Spd.Ki, VFD_Data.Ts);
        PI_Spd.Ki = _IQmpy(_IQ(2.0),PI_Spd.Ki); // Poles = (8/2)*(Ts/2)
   //     PI_Spd.Ki   = _IQdiv(PI_Spd.Ki, VFD_Data.TbwWm);
   //     PI_Spd.Kp = PI_Spd.KpInt    = _IQ(2.5);
   //     PI_Spd.Ki = PI_Spd.KiInt    = _IQ(0.00005);

        PI_Spd.KpInt = PI_Spd.Kp ;
        PI_Spd.KiInt = PI_Spd.Ki ;

        PI_Spd.Umax                 = _IQ(1.2);       // PI Saturation Max, IsqLim
        PI_Spd.Umin                 = _IQ(-1.2);      // PI Saturation Min

        Q_Decrement_Flag =0;  // In PMSM Motor to decrement the Q Axis Current

    }

    // Initialise PIcontrol Structures
    PI_Isd.Ref  = PI_Isq.Ref  = PI_Spd.Ref  = PI_Imr.Ref =_IQ(0.0);
    PI_Isd.Fbk  = PI_Isq.Fbk  = PI_Spd.Fbk  = PI_Imr.Fbk =_IQ(0.0);
    PI_Isd.Out  = PI_Isq.Out  = PI_Spd.Out  = PI_Imr.Out =_IQ(0.0);

    PI_Isd.up   = PI_Isq.up   = PI_Spd.up   = PI_Imr.up = _IQ(0.0);
    PI_Isd.ui   = PI_Isq.ui   = PI_Spd.ui   = PI_Imr.ui = _IQ(0.0);
    PI_Isd.u1   = PI_Isq.u1   = PI_Spd.u1   = PI_Imr.u1 = _IQ(0.0);
    PI_Isd.ut   = PI_Isq.ut   = PI_Spd.ut   = PI_Imr.ut = _IQ(0.0);
    PI_Isd.v1   = PI_Isq.v1   = PI_Spd.v1   = PI_Imr.v1 = _IQ(0.0);
   PI_Isd.i1   = PI_Isq.i1   = PI_Spd.i1   = PI_Imr.i1 = _IQ(0.0);

   VFD_Limits.AC_Current_Counter_Isc = 0 ;
   VFD_Limits.AC_Current_Counter_Isb = 0;
   VFD_Limits.AC_Current_Counter_Isa = 0 ;

   VFD_Limits.A_Phase_counter =  0;
   VFD_Limits.B_Phase_counter = 0;
   VFD_Limits.C_Phase_counter = 0;


   VFD_Limits.A_Phase_Flag = 0;
   VFD_Limits.B_Phase_Flag = 0;
   VFD_Limits.C_Phase_Flag = 0;
   VFD_Limits.A_Phase_Debug = 0;


   VFD_Limits.A_Phase_freq = _IQ(0.05);
   VFD_Limits.B_Phase_freq = _IQ(0.05);
   VFD_Limits.C_Phase_freq = _IQ(0.05);


   VFD_Limits.A_phase_temp = 0;
   VFD_Limits.B_phase_temp = 0;
   VFD_Limits.C_phase_temp = 0;


   VFD_Limits.A_Phase_Missing = 0;
   VFD_Limits.B_Phase_Missing = 0;
   VFD_Limits.C_Phase_Missing = 0;

   VFD_Limits.A_Phase_Freq_Miss = 0;
   VFD_Limits.B_Phase_Freq_Miss = 0;
   VFD_Limits.C_Phase_Freq_Miss = 0;

   VFD_Limits.Phase_Sequence_Flag = 0;
   VFD_Limits.Phase_sequence = 0;

   VFD_Limits.freq_A_Init_Flag_zero = 0;
   VFD_Limits.freq_B_Init_Flag_zero = 0;
   VFD_Limits.freq_C_Init_Flag_zero = 0;

   VFD_Limits.Isab_mirror = _IQ(0.0);
   VFD_Limits.Isbc_mirror = _IQ(0.0);
   VFD_Limits.Isac_mirror = _IQ(0.0);

   VFD_Limits.Low_freq = _IQ(0.0);
   VFD_Limits.Hig_freq = _IQ(0.0);

   VFD_Limits.zero_cross = 0;
   VFD_Limits.zero_cross_flag =0;

   VFD_Limits.Vgridmax = _IQ(0.0);
   VFD_Limits.Vgridmin = _IQ(0.0);
   VFD_Limits.Vtempgrid = _IQ(0.0);

   VFD_Limits.Inp_phase_unbalance_cnt = 0;

   VFD_Limits.Grid_Voltage_Flag = 0;

   VFD_Limits.DC_UV_counter  = 0;

   VFD_Limits.Heat_Sink_Fan_counter = 0;

   VFD_Limits.SMPS_Fan_counter = 0;

   VFD_Limits.Enclosure_Temp_counter = 0;






     // Initialising GUI PI Gains
        if (User_Data.Data_Srce == 01)
        {
            User_Data.SpeedKp = PI_Spd.Kp;
            User_Data.SpeedKi = PI_Spd.Ki;
            User_Data.CurrKp  = PI_Isd.Kp;
            User_Data.CurrKi  = PI_Isd.Ki;
        }

}
//------------------------------------------------------------------------------------------------------
void Init_FluxSpeedEst_Vars()
{
//------------------------------------------------------------------------------------------------------
// Initialise Flux & Speed Estimation Variables
//------------------------------------------------------------------------------------------------------

        MOT_FSE.FluxEstWc   = _IQmpy(VFD_Data.Ts, VFD_Data.FluxEstFc);
        MOT_FSE.K3          = _IQmpyIQX(_IQ21(MOT_Data.Wb), 21, VFD_Data.Ts, GLOBAL_Q);
        MOT_FSE.K4          = _IQ(1.0) - _IQmpy(_IQ(PIx2), MOT_FSE.FluxEstWc); // //



        MOT_FSE.SpeedEstTc  = _IQmpy(VFD_Data.Ts, VFD_Data.SpeedEstFc);
        MOT_FSE.SpeedEstTc  = _IQmpy(_IQ(PIx2), MOT_FSE.SpeedEstTc);

        MOT_FSE.CosThetaOld     = _IQ(1.0);
        MOT_FSE.SineThetaOld    = 0;
        MOT_FSE.CosTheta        = _IQ(1.0);
        MOT_FSE.SineTheta       = 0;

        MOT_FSE.VsAlpha  = MOT_FSE.VsBeta   = 0;
        MOT_FSE.EsAlpha  = MOT_FSE.EsBeta   = 0;
        MOT_FSE.IsAlpha  = MOT_FSE.IsBeta   = 0;
        MOT_FSE.Isd      = MOT_FSE.Isq      = 0;

        MOT_FSE.PsisAlpha   = MOT_FSE.PsisBeta      = 0;
        MOT_FSE.PsisAlphaOld= MOT_FSE.PsisBetaOld   = 0;
        MOT_FSE.PsirAlpha   = MOT_FSE.PsirBeta      = 0;

        MOT_FSE.WEst        = MOT_FSE.WSlip         = 0;
        MOT_FSE.WSyn        = MOT_FSE.WSyntemp      = 0;
        MOT_FSE.WEstFilt    = MOT_FSE.WEstFiltOld   = 0;
        MOT_FSE.WSlip       = MOT_FSE.WSyn          = 0;
}
//------------------------------------------------------------------------------------------------------
void Init_RotPosEst_Vars()
{
//------------------------------------------------------------------------------------------------------
// Initialise Rotor postion Estimaton Structure
//------------------------------------------------------------------------------------------------------

        PMSM_RPE.InjCnt             = 0;    // Counter for Current Injection
        PMSM_RPE.RotPosFlag         = 0;    // Set when Rotor Position Estimation is completed
        PMSM_RPE.RotPosCnt          = 0;    // Counter for the Estimation Function execution
        PMSM_RPE.RotPosCntLim       = 00;   // Limit for the Estimation Function execution

        VFD_Status.bits.ThetEpsFlag = 0;    // Initialise Change over Flag
        VFD_Status.bits.EpsThetFlag = 0;    // Initialise Change over Flag

        PMSM_RPE.Isq_Max            = 0;
        PMSM_RPE.Isd_Max            = 0;
}
//------------------------------------------------------------------------------------------------------
void Init_SVM_Vars()
{
//------------------------------------------------------------------------------------------------------
// Initialise SVPWM Generator structure
//------------------------------------------------------------------------------------------------------
        MOT_SVM.Alpha                   = 0;
        MOT_SVM.Beta                    = 0;
        MOT_SVM.VoltAmp                 = 0;
        MOT_SVM.Freq                    = 0;
        MOT_SVM.SecAngle                = 0;
        MOT_SVM.NewEntry                = 0;
        MOT_SVM.VecSector               = 0;
        MOT_SVM.Ta                      = 0;
        MOT_SVM.Tb                      = 0;
        MOT_SVM.Tc                      = 0;
        MOT_SVM.StepAngle               = 0;
        MOT_SVM.EntryOld                = 0;
        MOT_SVM.dx                      = 0;
        MOT_SVM.dy                      = 0;
        MOT_SVM.tmp1                    = 0;
        MOT_SVM.tmp2                    = 0;
        MOT_SVM.tmp3                    = 0;

        MOT_SVM.FreqMax                 = _IQmpyIQX(_IQ18((float32) 6 * MOT_Data.Wb / PIx2), 18, VFD_Data.Ts, GLOBAL_Q); // 1 pu. = 60 degree, dt = Sampling Freq
        MOT_SVM.Offset                  = 0;                            // No offset
}
//------------------------------------------------------------------------------------------------------
void Init_DynBrake_Vars()
{
//------------------------------------------------------------------------------------------------------
// Initialise Dynamic Brake Variables
//------------------------------------------------------------------------------------------------------

    VFD_Data.DynBrkFreq   = 10;       // Frequency of Dynamic Brake PWM (10 - Sw_Freq/2 Hz)
    VFD_Data.DynBrkDuty   = _IQ(0.1); // Duty cycle of Dynamic Brake PWM (0.1 - 0.4)

    if (VFD_Data.DynBrkFreq < 10)
        VFD_Data.DynBrkFreq = 10;
    if (VFD_Data.DynBrkFreq > VFD_Data.Sw_Freq * 500)
        VFD_Data.DynBrkFreq = VFD_Data.Sw_Freq * 500;
    if (VFD_Data.DynBrkDuty > _IQ(0.4))
        VFD_Data.DynBrkDuty = _IQ(0.4);

    Dyn_Con.Freq            = _IQ(1.0);
    Dyn_Con.RampOut         = 0;
    Dyn_Con.Angle           = 0;
    Dyn_Con.StepAng         = _IQmpyIQX(_IQ20((float32)VFD_Data.DynBrkFreq), 20, VFD_Data.Ts, GLOBAL_Q);
                                        // Base Frequency/Switching Frequency = 10Hz/3kHz
}

//------------------------------------------------------------------------------------------------------
void Init_DataLogger()
{
//------------------------------------------------------------------------------------------------------
// Initialise DATALOG module
//------------------------------------------------------------------------------------------------------
    DLOG_4CH_IQ_init(&dlog);
    dlog.input_ptr1 = &dval0;       // Input Data pointers
    dlog.input_ptr2 = &dval1;
    dlog.input_ptr3 = &dval2;
    dlog.input_ptr4 = &dval3;
    dlog.output_ptr1 = &DBUFF0[0];  // Output Data pointers => Import VFD_Graph.graphProp into Graph
    dlog.output_ptr2 = &DBUFF1[0];  // Address: "&DBUFF0", "&DBUFF1", "&DBUFF2", "&DBUFF3"
    dlog.output_ptr3 = &DBUFF2[0];
    dlog.output_ptr4 = &DBUFF3[0];
    dlog.size = DLOG_SIZE;          // Datalog size for each channel
    dlog.pre_scalar = 1;            // Prescalar count, typ = 1
    dlog.trig_value = 0;            // if input_ptr1 > trig_value, then data is logged
    dlog.status = 1;                // 0 => logs data as per trig_value
                                    // 1 => logs data instantaneously

    Dat_log.ch1Sel = 06;            // Initialise Datalog Channels
    Dat_log.ch2Sel = 10;
    Dat_log.ch3Sel = 18;
                                    // 01 = > VgRY (AC i/p)
                                    // 02 = > VgYB (AC i/p)
                                    // 03 = > Vdc  (DC Bus)

                                    // 04 = > Speed Refnce
                                    // 05 = > Speed Ramp
                                    // 06 = > Speed Actual

                                    // 07 = > Epsilon (gen)
                                    // 08 = > Sin Epsilon
                                    // 09 = > Cos Epsilon

                                    // 10 = > Theta (est)
                                    // 11 = > Sine Theta
                                    // 12 = > Cos Theta

                                    // 13 = > VsR (AC o/p)
                                    // 14 = > VsY (AC o/p)
                                    // 15 = > VsB (AC o/p)

                                    // 16 = > Ta (modulat)
                                    // 17 = > Tb (modulat)
                                    // 18 = > Tc (modulat)

                                    // 19 = > IsR (AC o/p)
                                    // 20 = > IsY (AC o/p)
                                    // 21 = > IsB (AC o/p)

                                    // 22 = > Isd Refnce
                                    // 23 = > Isd (AC o/p)
                                    // 24 = > Isq Refnce
                                    // 25 = > Isq (AC o/p)

                                    // 26 = > Vsd (AC o/p)
                                    // 27 = > Vsq (AC o/p)

                                    // 28 = > SpdRef (ana)
                                    // 29 = > Enclos Temp
                                    // 30 = > Ht Snk Temp
                                    // 31 = > Pt100  Temp
}
//------------------------------------------------------------------------------------------------------
void Data_Logger()
{
//-------------------------------------------------------------------------------------------------------------------
// DAC Write for debugging
//-------------------------------------------------------------------------------------------------------------------
//    EXT_DAC_Write(_IQtoF(VFD_Data.Vgry)* 0x3fff,_IQtoF(VFD_Data.Vgyb)* 0x3fff);
//    EXT_DAC_Write(VFD_ExtAdcData.Vgry, VFD_ExtAdcData.Vgyb);//_IQtoF(VFD_Data.Isb)* 0x3fff);
//    EXT_DAC_Write(_IQtoF(VFD_Data.SpeedRmp) * 0x3fff, _IQtoF(VFD_Data.SpeedAct)* 0x3fff);
//    EXT_DAC_Write((VFD_Status.bits.EpsThetFlag)* 0x3fff, MOT_RGQ.EqualFlag);

//    EXT_DAC_Write(Data_Selector(Dat_log.ch1Sel), Data_Selector(Dat_log.ch2Sel));  // DAC Write Ch1 & Ch2


//-------------------------------------------------------------------------------------------------------------------
// Data Log Module
//-------------------------------------------------------------------------------------------------------------------
    dval0 = _IQtoQ15(MOT_SVM.Ta);           // Pass values to be logged in DATALOG module
    dval1 = Data_Selector(Dat_log.ch1Sel);
    dval2 = Data_Selector(Dat_log.ch2Sel);
    dval3 = Data_Selector(Dat_log.ch3Sel);

    //dlog.status = 1;                // 0 => logs data as per trig_value
                                    // 1 => logs data instantaneously

    if (Dat_log.Mem_Type == 0)
    {
        DLOG_4CH_IQ_FUNC(&dlog);            // Call the DATALOG update function.

        SDRAMWrite();                       // Dat_log.Mem_Mode = 0 to Write
    }
    else if (Dat_log.Mem_Type == 1)
    {
        SDRAMRead();                        // Dat_log.Mem_Mode = 1 to Read
    }

    if (Dat_log.Mem_Mode == 10 || Dat_log.Mem_Mode == 20 )
    {
        SDChs       = Dat_log.Mem_Chns;
        SDMemTot    = Dat_log.Mem_Totl;
        SDMemInt    = Dat_log.Mem_Strt;
    }
}

signed int Data_Selector(int16 Channel)
{
//-------------------------------------------------------------------------------------------------------------------
// Data Log Channel Selector
//-------------------------------------------------------------------------------------------------------------------
    int16 RetVal = 0;
    switch (Channel)
    {
    case 0: RetVal = 0;
            break;
    case 1: RetVal = _IQtoQ15(_IQmpy(VFD_Data.Vgry, _IQ(0.5)));
        break;
    case 2: RetVal = _IQtoQ15(_IQmpy(VFD_Data.Vgyb, _IQ(0.5)));
        break;
    case 3: RetVal = _IQtoQ15(_IQmpy(VFD_Data.VdcFilt, _IQ(0.5)));
        break;
    case 4: RetVal = _IQtoQ15(_IQmpy(VFD_Data.SpeedRef, _IQ(0.5)));
        break;
    case 5: RetVal = _IQtoQ15(_IQmpy(VFD_Data.SpeedRmp, _IQ(0.5)));
        break;
    case 6: RetVal = _IQtoQ15(_IQmpy(VFD_Data.SpeedAct, _IQ(0.5)));
        break;
    case 7: RetVal = _IQtoQ15(_IQmpy(MOT_RC.RampOut, _IQ(0.5)));
        break;
    case 8: RetVal = _IQtoQ15(_IQmpy(VFD_Data.SineEpsilon, _IQ(0.5)));
        break;
    case 9: RetVal = _IQtoQ15(_IQmpy(VFD_Data.CosEpsilon, _IQ(0.5)));
        break;
    case 10: RetVal = _IQtoQ15(_IQmpy(MOT_FSE.ThetaFlux, _IQ(0.5)));
        break;
    case 11: RetVal = _IQtoQ15(_IQmpy(VFD_Data.SineTheta, _IQ(0.5)));
        break;
    case 12: RetVal = _IQtoQ15(_IQmpy(VFD_Data.CosTheta, _IQ(0.5)));
        break;
    case 13: RetVal = _IQtoQ15(_IQmpy(MOT_SVM.VaRef, _IQ(0.5)));
        break;
    case 14: RetVal = _IQtoQ15(_IQmpy(MOT_SVM.VbRef, _IQ(0.5)));
        break;
    case 15: RetVal = _IQtoQ15(_IQmpy(MOT_SVM.VcRef, _IQ(0.5)));
        break;
    case 16: RetVal = _IQtoQ15(_IQmpy(MOT_SVM.Ta, _IQ(0.5)));
        break;
    case 17: RetVal = _IQtoQ15(_IQmpy(MOT_SVM.Tb, _IQ(0.5)));
        break;
    case 18: RetVal = _IQtoQ15(_IQmpy(MOT_SVM.Tc, _IQ(0.5)));
        break;
    case 19: RetVal = _IQtoQ15(_IQmpy(VFD_Data.Isa, _IQ(0.5)));
        break;
    case 20: RetVal = _IQtoQ15(_IQmpy(VFD_Data.Isb, _IQ(0.5)));
        break;
    case 21: RetVal = _IQtoQ15(_IQmpy(VFD_Data.Isc, _IQ(0.5)));
        break;
    case 22: RetVal = _IQtoQ15(_IQmpy(PI_Isd.Ref, _IQ(0.5)));
        break;
    case 23: RetVal = _IQtoQ15(_IQmpy(PI_Isd.Fbk, _IQ(0.5)));
        break;
    case 24: RetVal = _IQtoQ15(_IQmpy(PI_Isq.Ref, _IQ(0.5)));
        break;
    case 25: RetVal = _IQtoQ15(_IQmpy(PI_Isq.Fbk, _IQ(0.5)));
        break;
    case 26: RetVal = _IQtoQ15(_IQmpy(VFD_Data.VsdRef, _IQ(0.5)));
        break;
    case 27: RetVal = _IQtoQ15(_IQmpy(VFD_Data.VsqRef, _IQ(0.5)));
        break;
    case 28: RetVal = _IQtoQ15(_IQmpy(VFD_Data.SpdRef, _IQ(0.5)));
        break;
    case 29: RetVal = _IQ15mpy(VFD_Data.EncTem, _IQ15((float32)1/Enc_max) );
        break;
    case 30: RetVal = _IQ15mpy(VFD_Data.HskTem, _IQ15((float32)1/Hsk_max) );
        break;
    case 31: RetVal = _IQ15mpy(VFD_Data.Pt100, _IQ15((float32)1/Pt_max));
        break;
    }
    return RetVal;

}

void Init_SDRAMLogger()
{
//------------------------------------------------------------------------------------------------------
// SDRAM Access initialisation
//------------------------------------------------------------------------------------------------------

    Dat_log.Mem_Type = 0;       // 0 => RAM
                                // 1 => SDRAM
    Dat_log.Mem_Mode = 0;       // Logging Resolution (ms)
    Dat_log.Mem_Chns = 3;       // No of Channels to Log (1 to 3)
    Dat_log.Mem_Totl = 1600;    // No of Bytes to be Written/Read
    Dat_log.Mem_Strt = 0;       // Initial Memory Location
    Dat_log.Mem_Resl = 10;      // Logging Resolution (ms)

    Dat_log.Mem_Mode   = 1; // 00 - Idle State
                            // 10 - Write Initialisation
                            // 01 - Write
                            // 20 - Read Initialisation
                            // 02 - Read
    SDDelCnt = 0;   // Delay Counter before each Read & Write
    SDMemCnt = 0;   // Processed Memory Count
}

void SDRAMWrite()
{
//------------------------------------------------------------------------------------------------------
// SDRAM Write
//------------------------------------------------------------------------------------------------------
                                                                                                                                                                                                                Dat_log.Mem_Strt = 0;
    Dat_log.Mem_Cont = SDMemCnt;

    if (Dat_log.Mem_Mode == 1)
        SDDelCnt++;     // Delay Count to Log only at Logging Rate
    if ((Dat_log.Mem_Mode == 1) && (SDDelCnt>(VFD_Data.Sw_Freq * Dat_log.Mem_Resl)))
    {
        SDDelCnt = 0;

        if (SDMemCnt < SDMemTot)
        {
            GpioCtrlRegs.GPBMUX1.bit.GPIO38     = 3;

            if (SDCh == 1)      // Log Channel 1
                *AddrPtr    = DBUFF0[SDBuffCnt];
            else if (SDCh == 2) // Log Channel 2
                *AddrPtr    = DBUFF1[SDBuffCnt];
            else if (SDCh == 3) // Log Channel 3
                *AddrPtr    = DBUFF2[SDBuffCnt];
            else if (SDCh == 4) // Log Channel 4
                *AddrPtr    = DBUFF3[SDBuffCnt];

            AddrPtr++;
            SDCh++;                 // Update Channel Num
            SDMemCnt++;     // Update Memory Counter

            if (SDCh > (SDChs+1))   // Upto No of Channels selected for logging
            {
                SDCh = 1;
                SDBuffCnt++;
                if (SDBuffCnt>(DLOG_SIZE-2))  // Buffer Limit
                    SDBuffCnt = 0;
            }
        }
        else
            Dat_log.Mem_Mode = 0;

    }
    else if (Dat_log.Mem_Mode == 10)
    {
        AddrPtr = SegmentStrtAddr;
        SDCh = 1;
        SDBuffCnt = 0;
        SDDelCnt = 0;
        SDMemCnt = 0;
    }
}

void SDRAMRead()
{
//------------------------------------------------------------------------------------------------------
// SDRAM Read
//------------------------------------------------------------------------------------------------------

    Dat_log.Mem_Strt = Dat_log.Mem_Totl - (400*(Dat_log.Mem_Chns+1));  // Read only 200 * 4Ch * 2 (8 bytes) Mem
    Dat_log.Mem_Cont = SDMemCnt;

    if (Dat_log.Mem_Mode == 2)
    {
        if (SDMemCnt < SDMemTot)
        {
            if (SDMemCnt>=SDMemInt)
            {
                if (SDCh == 1)      // Log Channel 1
                    DBUFF0[SDBuffCnt] = *AddrPtr;
                else if (SDCh == 2) // Log Channel 2
                    DBUFF1[SDBuffCnt] = *AddrPtr;
                else if (SDCh == 3) // Log Channel 3
                    DBUFF2[SDBuffCnt] = *AddrPtr;
                else if (SDCh == 4) // Log Channel 4
                    DBUFF3[SDBuffCnt] = *AddrPtr;

                SDCh++;         // Update Channel Num
                if (SDCh > (SDChs+1))   // Upto No of Channels selected for logging
                {
                    SDCh = 1;
                    SDBuffCnt++;
                    if (SDBuffCnt>(DLOG_SIZE-2))  // Buffer Limit
                        SDBuffCnt = 0;
                }
            }
            AddrPtr++;
            SDMemCnt++;     // Update Memory Counter

        }
        else
            Dat_log.Mem_Mode = 0;
    }
    else if (Dat_log.Mem_Mode == 20)
    {
        AddrPtr = SegmentStrtAddr;
        SDCh = 1;
        SDBuffCnt = 0;
        SDMemCnt = 0;
    }
}

void Init_EPROMAccess()
{

}

void EPROM128()
{

}



void EPROMRead()
{

}

//------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------
void Init_Modbus_Vars()
{
    VFD_ModbusMode.Mode   = 0;
    VFD_ModbusMode.Status = 0;
    VFD_ModbusMode.Index  = 0;
    TxCount     = 0;
    RxCount     = 0;
    timingCount = 0;

    StructAddrfloat[0]  = &User_Data.Prated;
    StructAddrint[0]    = &User_Data.Prated;
    StructAddr[0]       = &User_Data.Prated;

    StructAddr[1]       = &VFD_Limits.ABC_OC_Limit;

    StructAddrint[2]    = &VFD_Status;

    StructAddr[3]       = &VFD_Data.Isa;
    StructAddrint[3]    = &VFD_Data.Isa;

    StructAddr[4]       = &PI_Spd.Ref;
    StructAddr[5]       = &PI_Isq.Ref;
    StructAddr[6]       = &PI_Isd.Ref;

    StructAddrint[4]    = &DBUFF1[0];   // Start Address of Datalog ch 1
    StructAddrint[5]    = &DBUFF2[0];   // Start Address of Datalog ch 2
    StructAddrint[6]    = &DBUFF3[0];   // Start Address of Datalog ch 3

    StructAddrint[7]    = &Dat_log.ch1Sel;

}

void Offset_Reading()
{

    asm(" CLRC SXM");

    asm(" SETC SXM");


    if(VFD_Data.ADC_Flag==0 && VFD_Status.bits.StartFlag == 0)
    {
        AdcRegs.ADCTRL2.bit.RST_SEQ1        = 1;                // Reset Seq 1,             0=> No action
                                                                   //                          1=> Immediate Reset Seq 1 to initial state
        AdcRegs.ADCTRL2.bit.SOC_SEQ1        = 1;

        delay(50) ;
        while(!AdcRegs.ADCST.bit.SEQ1_BSY==0 ) ;

     adc_init_counter ++ ;
        if (VFD_Data.Mot_Rot == 0)    // AntiClockwise Motor Rotation (default)
        {
        VFD_Data.offset_Isa     = ( AdcRegs.ADCRESULT0>>4 ) +  VFD_Data.offset_Isa ; // Read the results, 4 bit shift for avoiding sign
        VFD_Data.offset_Isb     = ( AdcRegs.ADCRESULT1>>4 ) +  VFD_Data.offset_Isb;
        VFD_Data.offset_Isc      = ( AdcRegs.ADCRESULT2>>4 ) +  VFD_Data.offset_Isc;
        }
        if (VFD_Data.Mot_Rot == 1)    // Clockwise Motor Rotation
        {
            VFD_Data.offset_Isb     = (AdcRegs.ADCRESULT0>>4) +  VFD_Data.offset_Isb; // Read the results, 4 bit shift for avoiding sign
            VFD_Data.offset_Isa      = (AdcRegs.ADCRESULT1>>4 )+  VFD_Data.offset_Isa;
            VFD_Data.offset_Isc      = ( AdcRegs.ADCRESULT2>>4 ) +  VFD_Data.offset_Isc;
        }
        if(adc_init_counter > 63)
        {
        VFD_Data.ADC_Flag = 1;
        VFD_Data.offset_Isa = VFD_Data.offset_Isa >> 6 ;
        VFD_Data.offset_Isb = VFD_Data.offset_Isb >> 6 ;
        VFD_Data.offset_Isc = VFD_Data.offset_Isc >> 6 ;
        }
    }
 //   asm("NOP");

  //  AdcRegs.ADCTRL2.all = 0x4040 ;
 //

}

