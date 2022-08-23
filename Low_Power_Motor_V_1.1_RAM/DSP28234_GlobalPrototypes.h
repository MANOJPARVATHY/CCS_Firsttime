// FILE			: DSP28234_GlobalPrototypes.h
// TITLE		: DSP2833x Device Definitions.
// $TI Release	: DSP2833x/DSP2823x C/C++ Header Files V1.31 $
//####################################################################################################################
#ifndef DSP2833x_GLOBALPROTOTYPES_H
#define DSP2833x_GLOBALPROTOTYPES_H


#ifdef __cplusplus
extern "C" {
#endif
//--------------------------------------------------------------------------------------------------------------------
// Specify the PLL control register (PLLCR) and divide select (DIVSEL) value.
//--------------------------------------------------------------------------------------------------------------------
//#define DSP28_DIVSEL   0   	// Enable /4 for SYSCLKOUT
//#define DSP28_DIVSEL   1 		// Enable /4 for SYSCKOUT
#define DSP28_DIVSEL     2		// Enable /2 for SYSCLKOUT
//#define DSP28_DIVSEL   3 		// Enable /1 for SYSCLKOUT

//#define DSP28_PLLCR   10
//#define DSP28_PLLCR    9
#define DSP28_PLLCR    8
//#define DSP28_PLLCR    7
//#define DSP28_PLLCR    6
//#define DSP28_PLLCR    5
//#define DSP28_PLLCR    4
//#define DSP28_PLLCR    3
//#define DSP28_PLLCR    2
//#define DSP28_PLLCR    1
//#define DSP28_PLLCR    0  	// PLL is bypassed in this mode
//--------------------------------------------------------------------------------------------------------------------

//====================================================================================================================
// GPIO Definitions
//====================================================================================================================

#define ENABLE  1
#define DISABLE 0

#define GPIO	0
#define PERIPHERAL_1	1
#define PERIPHERAL_2	2
#define PERIPHERAL_3	3

#define INPUT	0
#define OUTPUT	1

#define GPIO_CLEAR	0
#define GPIO_SET	1
#define DIGHIGH		0		// Inversion in Hardware Circuit
#define DIGLOW		1		// Inversion in Hardware Circuit
//====================================================================================================================
// PWM Definitions
//====================================================================================================================
//-------------------------------------------------------------------------------------------------------------------
// TBCTL (Time-Base Control)
//-------------------------------------------------------------------------------------------------------------------
// FREE_SOFT bits
#define	TB_STP_CTR		0x0		// Emulation Halt Behaviour => Stop after next CTR Inc/Dec
#define	TB_STP_MODE		0x1		// Emulation Halt Behaviour => Stop when Up Mode     ; CTR=PRD,
								//										 Down Mode   ; CTR=0,
								//										 Up/Down Mode; CTR=0
#define	TB_FREE_RUN		0x1		// Emulation Halt Behaviour => Free Run
// PHSDIR bits
#define	TB_DOWN			0x0		// Phase Direction => Count Down after Sync
#define	TB_UP			0x1		// Phase Direction => Count Up after Sync
// CLKDIV and HSPCLKDIV bits
#define	TB_DIV1			0x0		// CLKDIV= /1 (default),	HSCLKDIV= /1			TBCLK=SYSCLKOUT/(HSPCLKDIV×CLKDIV)
#define	TB_DIV2			0x1		// CLKDIV= /2,				HSCLKDIV= /2 (default)	TBCLK=SYSCLKOUT/(HSPCLKDIV×CLKDIV)
#define	TB_DIV4			0x2		// CLKDIV= /4,				HSCLKDIV= /4			TBCLK=SYSCLKOUT/(HSPCLKDIV×CLKDIV)
// SYNCOSEL bits
#define	TB_SYNC_IN		0x0		// Source of the SYNC o/p signal => SYNC In
#define	TB_CTR_ZERO		0x1		// Source of the SYNC o/p signal => CTR=0
#define	TB_CTR_CMPB		0x2		// Source of the SYNC o/p signal => CTR=CMPB
#define	TB_SYNC_DISABLE	0x3		// Source of the SYNC o/p signal => Disable SYNCO
// PRDLD bit
#define	TB_SHADOW		0x0		// Period Shadow Load => Load on CTR=0
#define	TB_IMMEDIATE	0x1		// Period Shadow Load => Load Immediately
// PHSEN bit
#define	TB_DISABLE		0x0		// Phase Reg Disable
#define	TB_ENABLE		0x1		// Phase Reg Enable => CTR=TBPHS on EPWMxSYNCI
// CTRMODE bits
#define	TB_COUNT_UP		0x0		// Counter Mode => Count Up
#define	TB_COUNT_DOWN	0x1		// Counter Mode => Count Down
#define	TB_COUNT_UPDOWN	0x2		// Counter Mode => Count Up Down
#define	TB_FREEZE		0x3		// Counter Mode => Stop Freeze

//-------------------------------------------------------------------------------------------------------------------
// CMPCTL (Compare Control)
//-------------------------------------------------------------------------------------------------------------------
// SHDWAMODE and SHDWBMODE bits
#define	CC_SHADOW		0x0		// CMPB/CMPA Operating Mode => Shadow mode, Double Buffer w Shadow Register
#define	CC_IMMEDIATE	0x1		// CMPB/CMPA Operating Mode => Immediate mode, Shadow Register not used
// LOADAMODE and LOADBMODE bits
#define	CC_CTR_ZERO		0x0		// CMPB/CMPA Shadow Load Mode => Load on CTR=0
#define	CC_CTR_PRD		0x1		// CMPB/CMPA Shadow Load Mode => Load on CTR=PRD
#define	CC_CTR_ZERO_PRD	0x2		// CMPB/CMPA Shadow Load Mode => Load on CTR=0 or PRD
#define	CC_LD_DISABLE	0x3		// CMPB/CMPA Shadow Load Mode => Disabled

//-------------------------------------------------------------------------------------------------------------------
// AQCTLA and AQCTLB (Action Qualifier Control)
//-------------------------------------------------------------------------------------------------------------------
#define	AQ_NO_ACTION	0x0		// Action => Disabled
#define	AQ_CLEAR		0x1		// Action => Clear (low)
#define	AQ_SET			0x2		// Action => Set   (high)
#define	AQ_TOGGLE		0x3		// Action => Toggle

//-------------------------------------------------------------------------------------------------------------------
// AQSFRC (Action Qualifier S/W Control)
//-------------------------------------------------------------------------------------------------------------------
// RLDCSF bits
#define	AQ_CTR_ZERO		0x0		// AQSFRC Shadow Reload Options => Load on CTR = 0
#define	AQ_CTR_PRD		0x1		// AQSFRC Shadow Reload Options => Load on CTR = PRD
#define	AQ_CTR_ZERO_PRD	0x2		// AQSFRC Shadow Reload Options => Load on CTR = 0 or PRD
#define	AQ_IMMEDIATE	0x3		// AQSFRC Shadow Reload Options => Load Immediately
// OTSFB and OTSFA bits
#define	AQ_DISABLED		0x0		// One Time S/W Force on B/A => No Action
#define	AQ_SW_FORCE		0x1		// One Time S/W Force on B/A => Single S/W Force Event

//-------------------------------------------------------------------------------------------------------------------
// AQCSFRC (Action Qualifier Continuous S/W Control)
//-------------------------------------------------------------------------------------------------------------------
// CSFB and CSFA bits
#define	AQ_FRC_DISABLED	0x0		// Continuous S/W Force on Output B/A => Forcing Disabled
#define	AQ_FRC_CONT_LO	0x1		// Continuous S/W Force on Output B/A => Forcing Continuous low on output
#define	AQ_FRC_CONT_HI	0x2		// Continuous S/W Force on Output B/A => Forcing Continuous high on output

//-------------------------------------------------------------------------------------------------------------------
// DBCTL (Dead-Band Control)
//-------------------------------------------------------------------------------------------------------------------
// IN MODE
#define DBA_RED_FED     0x0		// In Mode Control => PWMxA is source for RED and FED
#define DBA_FED_DBB_RED 0x1		// In Mode Control => PWMxA is source for FED,
								// 					  PWMxB is source for RED
#define DBA_RED_DBB_FED 0x2		// In Mode Control => PWMxA is source for RED,
								// 					  PWMxB is source for FED
#define DBB_RED_FED     0x3		// In Mode Control => PWMxB is source for RED and FED
// POLSEL bits
#define	DB_ACTV_HI		0x0		// Polarity Select Control => Active High
#define	DB_ACTV_LOC		0x1		// Polarity Select Control => Active Low Complementary (RED)
#define	DB_ACTV_HIC		0x2		// Polarity Select Control => Active High Complementary (FED)
#define	DB_ACTV_LO		0x3		// Polarity Select Control => Active Low
// OUT MODE bits
#define	DB_DISABLE		0x0		// Out Mode Control => Disabled
#define	DBA_ENABLE		0x1		// Out Mode Control => PWMxA = No Delay
								// 					   PWMxB = FED
#define	DBB_ENABLE		0x2		// Out Mode Control => PWMxA = RED
								// 					   PWMxB = No Delay
#define	DB_FULL_ENABLE	0x3		// Out Mode Control => RED and RED, DBM fully enabled

//-------------------------------------------------------------------------------------------------------------------
// CHPCTL (chopper control)
//-------------------------------------------------------------------------------------------------------------------
// CHPEN bit
#define	CHP_DISABLE		0x0
#define	CHP_ENABLE		0x1
// CHPFREQ bits
#define	CHP_DIV1		0x0
#define	CHP_DIV2		0x1
#define	CHP_DIV3		0x2
#define	CHP_DIV4		0x3
#define	CHP_DIV5		0x4
#define	CHP_DIV6		0x5
#define	CHP_DIV7		0x6
#define	CHP_DIV8		0x7
// CHPDUTY bits
#define	CHP1_8TH		0x0
#define	CHP2_8TH		0x1
#define	CHP3_8TH		0x2
#define	CHP4_8TH		0x3
#define	CHP5_8TH		0x4
#define	CHP6_8TH		0x5
#define	CHP7_8TH		0x6

//-------------------------------------------------------------------------------------------------------------------
// TZSEL (Trip Zone Select)
//-------------------------------------------------------------------------------------------------------------------
// CBCn and OSHTn bits
#define	TZ_DISABLE		0x0		// Disable TZx as a => (OSHT) One-Shot Trip / (CBC) Cycle-by-Cycle Source for this ePWM module
#define	TZ_ENABLE		0x1		// Enable TZx as a  => (OSHT) One-Shot Trip / (CBC) Cycle-by-Cycle Source for this ePWM module

//-------------------------------------------------------------------------------------------------------------------
// TZCTL (Trip Zone Control)
//-------------------------------------------------------------------------------------------------------------------
// TZA and TZB bits
#define	TZ_HIZ			0x0		// Trip Event Action => EPWMx to a High impedance State
#define	TZ_FORCE_HI		0x1		// Trip Event Action => Force EPWMx to a High State
#define	TZ_FORCE_LO		0x2		// Trip Event Action => Force EPWMx to a Low State
#define	TZ_NO_CHANGE	0x3		// Trip Event Action => No Action on EPWMx

#define TZ_CLEAR		0x1		// Clears this Trip (set) condition.

//-------------------------------------------------------------------------------------------------------------------
// ETSEL (Event Trigger Select)
//-------------------------------------------------------------------------------------------------------------------
// SOCBEN, SOCAEN, INTEN bits
#define ET_DISABLE		0x0		// EPWMxSOCB and EPWMxSOCA Disable
#define ET_ENABLE		0x1		// EPWMxSOCB and EPWMxSOCA Enable
// SOCBSEL, SOCASEL, INTSEL bits
#define	ET_CTR_ZERO		0x1		// EPWMxSOCB and EPWMxSOCA Select => CTR=0
#define	ET_CTR_PRD		0x2		// EPWMxSOCB and EPWMxSOCA Select => CTR=PRD
#define	ET_CTRU_CMPA	0x4		// EPWMxSOCB and EPWMxSOCA Select => CTRU=CMPA
#define	ET_CTRD_CMPA	0x5		// EPWMxSOCB and EPWMxSOCA Select => CTRD=CMPA
#define	ET_CTRU_CMPB	0x6		// EPWMxSOCB and EPWMxSOCA Select => CTRU=CMPB
#define	ET_CTRD_CMPB	0x7		// EPWMxSOCB and EPWMxSOCA Select => CTRD=CMPB

//-------------------------------------------------------------------------------------------------------------------
// ETPS (Event Trigger Pre-scale)
//-------------------------------------------------------------------------------------------------------------------
// INTPRD, SOCAPRD, SOCBPRD, INTCNT, SOCACNT, SOCBCNT bits
#define	ET_DISABLE		0x0		// Period, Counter = Disabled
#define	ET_1ST			0x1		// Period, Counter = 1st Event, 1 Event
#define	ET_2ND			0x2		// Period, Counter = 2nd Event, 2 Events
#define	ET_3RD			0x3		// Period, Counter = 3rd Event, 3 Events

//====================================================================================================================
// HRPWM (High Resolution PWM)
//====================================================================================================================
// HRCNFG
#define	HR_Disable		0x0
#define	HR_REP			0x1
#define	HR_FEP			0x2
#define	HR_BEP			0x3

#define	HR_CMP			0x0
#define	HR_PHS			0x1

#define	HR_CTR_ZERO		0x0
#define	HR_CTR_PRD		0x1

//====================================================================================================================
// SCI Definitions
//====================================================================================================================
//-------------------------------------------------------------------------------------------------------------------
// SCICCR (SCI Communication Control Register)
//-------------------------------------------------------------------------------------------------------------------
// STOPBITS bit
#define SCI_1STOP_BIT	 0x0	// 1 Stop Bit
#define SCI_2STOP_BIT	 0x1	// 2 Stop Bits
// PARITY bit
#define SCI_ODD_PARITY	 0x0	// Odd Parity
#define SCI_EVEN_PARITY	 0x1	// Even Parity
// ADDRIDLEMODE bit
#define SCI_IDLE_LINE	 0x0	// Idle line Mode
#define SCI_ADDR_BIT	 0x1	// Address bit Mode

//-------------------------------------------------------------------------------------------------------------------
// SCICTL1 (SCI Control Register 1)
//-------------------------------------------------------------------------------------------------------------------
// SWRESET bit
#define SCI_SW_RESET	 0x0	// SCI-A S/W Reset
#define SCI_SW_RELEASE	 0x1	// SCI-A S/W Release from Reset

//-------------------------------------------------------------------------------------------------------------------
// SCICTL2 (SCI Control Register 2)
//-------------------------------------------------------------------------------------------------------------------
// TSRDY, TXEMPTY bits
#define SCI_TX_FULL  	 0x0	// SCI-A TX Ready => SCITXBUF is full (TSRDY), TXBUF or Shift Register loaded with data (TXEMPTY)
#define SCI_TX_EMPTY  	 0x1	// SCI-A TX Ready => SCITXBUF is empty (TSRDY), TXBUF and Shift Register empty (TXEMPTY)

// RXERRINTENA, TXWAKE, SLEEP, TXENA, RXENA, PARITYENA, LOOPBKENA, RXBKINTENA, TXINTENA, TXFFIENA, RXFFIENA, SCIFFCT bits
#define	SCI_DISABLED	 0x0	// SCI-A Disable
#define	SCI_ENABLED		 0x1	// SCI-A Enable

//====================================================================================================================
// Internal ADC Definitions
//====================================================================================================================
//-------------------------------------------------------------------------------------------------------------------
// ADCTRL1 (ADC Control Register 1)
//-------------------------------------------------------------------------------------------------------------------
// RESET bits
#define ADC_NO_EFFECT	0x0		// ADC Module Reset => No Effect
#define ADC_RESET		0x1		// ADC Module Reset => Reset, set back to 0 by ADC Logic
// SUSMOD bits
#define ADC_FREE_RUN	0x0		// Emulation Suspend Mode => Free Run, Don't Stop
#define ADC_CURR_SEQ	0x1		// Emulation Suspend Mode => Stop after Current Sequence
#define ADC_CURR_CONV	0x2		// Emulation Suspend Mode => Stop after Current Conversion
#define ADC_IMMEDIATE	0x3		// Emulation Suspend Mode => Stop Immediately
// SEQ_CASC bit
#define ADC_SEQ_DUAL	0x0		// Sequencer Mode => Dual Mode
#define ADC_SEQ_CASC	0x1		// Sequencer Mode => Cascaded Mode

//-------------------------------------------------------------------------------------------------------------------
// ADCTRL3 (ADC Control Register 3)
//-------------------------------------------------------------------------------------------------------------------
// ADCBGRFDN bit
#define ADC_POW_BGRF_DN	0x0		// ADC Bandgap & Ref Circuitry => Power Down
#define ADC_POW_BGRF_UP	0x3		// ADC Bandgap & Ref Circuitry => Power Up
// ADCPWDN bit
#define ADC_POW_DN		0x0		// ADC => Power Down
#define ADC_POW_UP		0x1		// ADC => Power Up
// ADCCLKPS bits
#define ADC_CLK_DIV0 	0x0		// ADC Clock Prescale => HSPCLK/{1x(ADCCTRL1.bit.CPS+1)}
#define ADC_CLK_DIV1 	0x1		// ADC Clock Prescale => HSPCLK/{2x(ADCCTRL1.bit.CPS+1)}
#define ADC_CLK_DIV2 	0x2		// ADC Clock Prescale => HSPCLK/{4x(ADCCTRL1.bit.CPS+1)}
#define ADC_CLK_DIV3 	0x3		// ADC Clock Prescale => HSPCLK/{6x(ADCCTRL1.bit.CPS+1)}
// SMODE_SEL bit
#define ADC_SEQ_MODE	0x0		// Sampling Mode Select => Sequential Sampling Mode
#define ADC_SIM_MODE	0x1		// Sampling Mode Select => Simultaneous Sampling Mode

//-------------------------------------------------------------------------------------------------------------------
// ADCREFSEL (ADC Reference Select Register )
//-------------------------------------------------------------------------------------------------------------------
//REF_SEL bits
#define ADC_REF_INT		0x0		// ADC Reference Voltage => Internal Reference
#define ADC_REF_EXTA	0x1		// ADC Reference Voltage => External Reference, 2.048 V on ADCREFIN
#define ADC_REF_EXTB	0x2		// ADC Reference Voltage => External Reference, 1.500 V on ADCREFIN
#define ADC_REF_EXTC	0x3		// ADC Reference Voltage => External Reference, 1.024 V on ADCREFIN


#define Root3by2	 0x376C			// <<2  

//====================================================================================================================
#define Stablisation_Time 0x2DF9 
//====================================================================================================================
extern void Initialise_DSP();									// Function to initialise the DSP Registers
extern void InitSysCtrl();										// Function to Initialise the System Control Registers
extern void DisableDog();										// Function to Disable the WatchDog
extern void InitPll();											// Function to Initialise the PLL
extern void InitPeripheralClocks();								// Function to Initialise the Clock for various Peripherls

extern void InitGpio();											// Function to initialise the General Purpose Port
extern void InitInverterEPwm();									// Function to initialise the EPWM Modules

extern void InitPieCtrl();										// Function to initialise the Peripheral Interrupt Control Registers
extern void InitPieVectTable();									// Funciton to initialise the Peripheral Vector Table
extern void InitPeripherals();									// Function to initialise the required peripherals like CAN & RS422

extern void	InitSCIComm();										// Function to initialise the SCI Registers

extern void InitADCint();										// Function to initialise the Internal ADC Registers

extern void	InitSPI();											// Function to initialise the SPI Registers
extern void InitTimer();                                        // Function to initialise Timer Registers

extern void Initialise_Variables();								// Function to initialise the variables

extern void Wait_for_Interrupt();								// Function to Wait for Interrupt

extern void GpioDelay();										// Function for delay to be added between GPIO Data Registers

extern void Start_Timer(); 										// Function to start the timer operation
extern void System_Faults();									// Function to Protect the System
extern int DAC_Display(int DacData);							// Function for Display in DAC

extern void DynamicBreakControl();								// Function to Control the Dynamic Breaking IGBT
extern void UserInterface();									// Function to check the user interface
extern void LowPassFilter();									// Function for low pass filter



#ifdef __cplusplus
}
#endif /* extern "C" */

#endif   // - end of DSP2833x_GLOBALPROTOTYPES_H
//===================================================================================================================
// End of file.
//===================================================================================================================

