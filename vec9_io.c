// VEC9 Input/Output Adapter
// Todd Michael Bailey

//=============================
// Atmel AVR ATXmega384d3u
// 20MHz Silicon Oscillator (LTC6905)
// GCC 4.8.2
//==============================


/*
Description:
==============================================================================
NOTE -- this was first based on the older PS2 adapter for VEC9 which used serial communications to a PS2 controller.
The newer unit uses the M1 Abrams Tank Gun Control Yoke and has generic IOs (reads resistances and switch closures, and sends out on/off bits)

The "start of message" bytes are PS2 related, and can (should) change.

Inputs:
----------------
Yoke Pitch (analog)
Yoke Roll (analog)
Yoke Triggers Right (3)
Yoke Triggers Left (3)
Safety Switches (4)
Illuminated Switches (2 -- NC (annoying))
Key Switch (1) (unused)
Coin Switch (1) (replaces key switch)
Test Mode switch (1)
Total: 14 + 2

Outputs:
-----------------
Flight Indicators (10, incandescent)
Illuminated Switches (2, incandescent)
Air Horn (1, motor-ish)
Red/Green LED Switch Panel Indicators (8)

Total: 21

Total IO = 37

In order to get all these, we drive the R/G LEDs with a serial-parallel converter (a 595 and inverters)

==================================================================================================
==================================================================================================

==================================================================================================
==================================================================================================

Serial Data Format:
---------------------
All serial exchanges are at 38400,8,1,N.

PC-To-Xmega (Poll Request / Set outputs):
--------------------------------------------
0x42 outputsMsb outputsMiddle outputsLsb


Reply from XMEGA:
------------------------
0x41 switchData0 switchlData1 0 0 analogRoll analogPitch


Timer Hardware Usage:
--------------------------


*/

#include	"includes.h"

#define		BUILD_DATE						__DATE__	// String we print when we want to know when we last updated this

// --------------------------
// Defines for this file:
// --------------------------

// --------------------------
// Globals for this file:
// --------------------------

// --------------------------
// Function Prototypes:
// --------------------------

static void CCPWrite( volatile uint8_t * address, uint8_t value );

//-----------------------------------------------------------------------
//-----------------------------------------------------------------------
// State Machine Functions.
//-----------------------------------------------------------------------
//-----------------------------------------------------------------------

void SetState(STATE_FUNC *newState)		// Sets the device to a new state, assumes it should begin at the first minor sub-state.
{
	State=newState;
	subState=SS_0;
}


// --------------------------
// Error Recovery:
// --------------------------

ISR(__vector_default)
{
    //  This means a bug happened.  Some interrupt that shouldn't have generated an interrupt went here, the default interrupt vector.
	//	printf("Buggy Interrupt Generated!  Flags = ");
	//  printf("*****put interrupt register values here****");

	// Hang, or reset part, or whatever

	asm("jmp 0000");			// head to normal reset vector, should never happen
}

//-----------------------------------------------------------------------
//-----------------------------------------------------------------------
// Software clock / PerIrq init
//-----------------------------------------------------------------------
//-----------------------------------------------------------------------

static void InitSoftclock(void)
// We're using PORTE's timer 0 for this.
// Look at the value of SECOND define, and set number of periodic interrupt requests per second.
// Affects watchdog, too.
{
	systemTicks=0;

	PR.PRPE&=~PR_TC0_bm;		// Turn on timer counter zero for porte

	TCE0.CCA=0;						// Compare match 0
	TCE0.CNT=0;						// Set Count Value to 0.
	TCE0.PER=(F_CPU/SECOND);		// Set period for this timer (max count)

	TCE0.CTRLB=0;					// No compares/captures enabled, and no waveforms

	TCE0.INTFLAGS=0xFF;						// Clear the interrupt flags
//	TCE0.INTCTRLA=TC_OVFINTLVL_HI_gc;		// Set wdt overflow interrupt (high priority)
	TCE0.INTCTRLA=TC_OVFINTLVL_LO_gc;		// Set wdt overflow interrupt (low priority)
	TCE0.INTCTRLB=0;						// No CC ints
	TCE0.CTRLA=TC_CLKSEL_DIV1_gc;			// Start the timer with a clock division of 1.
}

//-----------------------------------------------------------------------
//-----------------------------------------------------------------------
// Periodic Interrupt Request
//-----------------------------------------------------------------------
//-----------------------------------------------------------------------

ISR(TCE0_OVF_vect)
{
	systemTicks++;					// Increment the system ticks.
}

//-----------------------------------------------------------------------
//-----------------------------------------------------------------------
// Protected Register Access Functions
//-----------------------------------------------------------------------
//-----------------------------------------------------------------------

static void CCPWrite(volatile uint8_t * address, uint8_t value)
// Assembly helper function which writes the Config Change Protection register and immediately the passed protected register.
// NOTE -- this only writes protected IO registers, not SPM/LPM.
// Cribbed more or less from AVR1003, with the non-avrgcc #if statements taken out.
{
	unsigned char
		sreg;

	sreg=SREG;		// Pause interrupts
	cli();

	volatile uint8_t * tmpAddr = address;	// Redefine this locally (look this up, not sure why)
	RAMPZ = 0;								// Clear third Z indirect addressing reg

	asm volatile(
		"movw r30,  %0"	      "\n\t"		// Store our temp address
		"ldi  r16,  %2"	      "\n\t"		// Store our CCP signature
		"out   %3, r16"	      "\n\t"		// Write signature to CCP register
		"st     Z,  %1"       "\n\t"		// Put the passed value into the passed address (happens one cycle later)
		:
		: "r" (tmpAddr), "r" (value), "M" (CCP_IOREG_gc), "i" (&CCP)		// variables for above
		: "r16", "r30", "r31"												// Clobber list
		);

	SREG=sreg;		// Restore interrupts
}

//-----------------------------------------------------------------------
//-----------------------------------------------------------------------
// Printf support functions
//-----------------------------------------------------------------------
//-----------------------------------------------------------------------

int UartPutChar(char c, FILE *stream)		// Associating this with FILE makes this link to stdout and lets you use printf()
// Note -- this implementation pulls in ALL KINDS of garbage from standard C libraries and leads to lots of code bloat.
// Re-write it when you have time.
{
	if(c=='\n')
	{
		UartPutChar('\r', stream);				// Always follow a new line with a carriage return.
	}

	while(!(USARTF0.STATUS&USART_DREIF_bm))		// Hang here until there is room in the transmit buffer (bit is 1 when there's room)
	{
		;
	}
	USARTF0.DATA = c;							// Then xmit the character you've been passed.

	return(0);									// Returning an int makes this function play with printf() (no errors).
}

//-----------------------------------------------------------------------
//-----------------------------------------------------------------------
// Serial Functions
//-----------------------------------------------------------------------
//-----------------------------------------------------------------------
// NOTE:  It might be smart to dump anything partially received when the transmitter begins transmitting.
// These talk to the gateway.

#define		SERIAL_USART		USARTF0
#define		MAX_RX_FIFO_BYTES	64

static volatile unsigned char
	txBuffer[16],
	txBufferIndex,
	txBytesToSend,
	rxFifo[MAX_RX_FIFO_BYTES],
	fifoReadPointer,
	fifoWritePointer,
	dump,				// Keep me volatile
	bytesInRxFifo;

ISR(USARTF0_RXC_vect)
// When we receive a byte via serial, stick it in the FIFO.
{
	if(bytesInRxFifo<MAX_RX_FIFO_BYTES)
	{
		rxFifo[fifoWritePointer]=SERIAL_USART.DATA;	// Put data in fifo at current write pointer
		fifoWritePointer++;							// Move write pointer forward
		if(fifoWritePointer>=MAX_RX_FIFO_BYTES)		// Roll write pointer around end of ring buffer if needed
		{
			fifoWritePointer=0;
		}
		bytesInRxFifo++;							// One more byte in the fifo
	}
	else
	{
		dump=SERIAL_USART.DATA;		// For whatever reason, writing a one to the flag here doesn't seem to work.  But this does.
	}
}

static bool	RxFifoNotEmpty(void)
// Return true if there are unread data in the fifo
{
	unsigned char
		sreg;
	bool
		retVal;

	sreg=SREG;
	cli();

	retVal=false;
	if(bytesInRxFifo)	// Got anything in fifo?
	{
		retVal=true;
	}

	SREG=sreg;
	return(retVal);
}

unsigned char GetByteFromRxFifo(void)
// Returns bytes from the RS485 fifo in the order they were collected.
{
	unsigned char
		sreg,
		theByte;

	sreg=SREG;
	cli();

	if(bytesInRxFifo)	// Got anything in fifo?
	{
		theByte=rxFifo[fifoReadPointer];
		fifoReadPointer++;							// Move pointer forward
		if(fifoReadPointer>=MAX_RX_FIFO_BYTES)		// Roll pointer around end of ring buffer if needed
		{
			fifoReadPointer=0;
		}
		bytesInRxFifo--;							// One less byte in the fifo

		SREG=sreg;
		return(theByte);
	}
	else
	{
		SREG=sreg;
		return(0);		// Should not happen.  If we call this when there are no bytes in the fifo, return 0
	}
}

static void InitSerialFifos(void)
{
	unsigned char
		sreg;

	sreg=SREG;
	cli();

	bytesInRxFifo=0;					// Init ring buffer for RS485 byte reception
	fifoWritePointer=0;
	fifoReadPointer=0;
	txBufferIndex=0;
	txBytesToSend=0;
	SERIAL_USART.CTRLA=USART_RXCINTLVL_MED_gc;	// Interrupts enabled, medium priority

	SREG=sreg;
}

//-----------------------------------------------------------------------
//-----------------------------------------------------------------------
// Switch functions:
//-----------------------------------------------------------------------
//-----------------------------------------------------------------------

static unsigned int
	keyState,
	newKeys,
	newKeysReleased;

static void InitSwitches(void)
// Sets up user input -- any physical switches which need to be debounced.
// PB0-7 and PC0-4 are switch inputs
// PE2 is our test mode switch input
// NOTE -- we invert the data coming in from these pins, so a pressed switch is read as a ONE later.
// The pushbuttons (PC2 and PC3) are NC, so DON'T invert them.
// NOTE -- For PORTB to work right, JTAG gotta go.
{
	PORTB.DIRCLR=0xFF;									// All bits
	PORTCFG.MPCMASK=0xFF;								// Configure these pins on this port next time we write the config register
	PORTB.PIN0CTRL=PORT_INVEN_bm|PORT_OPC_PULLUP_gc;	// Set them to be pulled up and inverted

	PORTC.DIRCLR=0x1F;									// Bottom five bits
//	PORTCFG.MPCMASK=0x1F;								// Configure these pins on this port next time we write the config register
	PORTC.PIN0CTRL=PORT_INVEN_bm|PORT_OPC_PULLUP_gc;	// Set them to be pulled up and inverted
	PORTC.PIN1CTRL=PORT_INVEN_bm|PORT_OPC_PULLUP_gc;	// Set them to be pulled up and inverted
	PORTC.PIN2CTRL=PORT_OPC_PULLUP_gc;					// Set them to be pulled up
	PORTC.PIN3CTRL=PORT_OPC_PULLUP_gc;					// Set them to be pulled up
	PORTC.PIN4CTRL=PORT_INVEN_bm|PORT_OPC_PULLUP_gc;	// Set them to be pulled up and inverted

	PORTE.DIRCLR=(1<<2);								// PE2 to input
	PORTE.PIN2CTRL=PORT_INVEN_bm|PORT_OPC_PULLUP_gc;	// Set to be pulled up and inverted

	keyState=0;		// No keys pressed
	newKeys=0;		// No keys new

	SetTimer(TIMER_DEBOUNCE,(SECOND/64));
}


static void HandleSwitches(void)
// Read input pins, debounce, and flag newly-appeared keys.
// NOTE -- because the inputs have been inverted, they are already positive true
// NOTE -- For PORTB to work right, JTAG must be disabed.
{
	static unsigned int
		lastKeyState;

	lastKeyState=keyState;					// Record old keystate for comparison's sake

	if(CheckTimer(TIMER_DEBOUNCE))
	{
		keyState=PORTB.IN;								// Grab all PORTB inputs
		keyState|=((unsigned int)PORTC.IN&0x1F)<<8;		// Grab bottom 5 PORTC

		if(PORTE.IN&(1<<2))		// Check test switch, alone on this port
		{
			keyState|=Im_TEST;
		}

		SetTimer(TIMER_DEBOUNCE,(SECOND/64));
	}

	newKeys=((keyState^lastKeyState)&(keyState));			// Flag the keys which have been pressed since the last test.
	newKeysReleased=lastKeyState&(keyState^lastKeyState);	// And the ones immediately un-pressed
}

//-----------------------------------------------------------------------
//-----------------------------------------------------------------------
// ADC Handling:
//-----------------------------------------------------------------------
//-----------------------------------------------------------------------
// ADC has serious errata in this device -- see DS.
// We must used the signed mode (even though we are single ended) and our VREF is external and set to 2.5V
// This means that we have 11 bit accuracy and not 12, and there are some other weirdnesses too.
// For instance, since we are running differential, it is helpful to tie a PORT pin to ground and use it as a reference.

// Some helpful posts:
// http://blog.frankvh.com/2010/01/03/atmel-xmega-adc-problems-solutions/
// http://www.bostonandroid.com/manuals/xmega-precision-adc-howto.html

// Thu Nov 17 15:06:04 EST 2011
// As far as I can tell, a single conversion without gain takes 6 adc clock cycles
// So, at 125hKz adc clock, throwing out 4 samples and averaging 32, each source we read takes about 1.7mSecs, so 6.91mSecs for 4, or a full update 144 times a sec.  Plenty fast enough.

// NOTE -- decent stability at /64

// Sat Feb  2 16:34:39 EST 2013
// Switched to -AU series xmegas.  In theory these fix all the old ADC problems.  We'll see...
// Sample time is 7 adc clock cycles.  So at 20MHz / 512 (39kHz) we have 0.18mSec per sample.
// At 36 samples per source, we have 6.5mSec per source
// With 5 sources we'd have 32mSec to get everything (or 31Hz update rate).
// 2 sources is 77 times a second.

#define		SAMPLES_TO_ADD		8		// Don't exceed an int adding these up
#define		SAMPLES_TO_AVERAGE	8		// Then divide by this.  Can get oversampling if needed.
#define		SAMPLES_TO_TOSS		4		// Throw out this many every time we change the mux

enum									// Things our ADC is keeping track of
	{
		ADC_YOKE_PITCH=0,
		ADC_YOKE_ROLL,
		NUM_ADC,
	};

static unsigned int
	adcResults[NUM_ADC];	// Store processed results here
static bool
	newAdcResult[NUM_ADC];	// Flags which tell when an ADC result is new
static unsigned char
	currentAdcSource;			// Which ADC input are we looking at?

static unsigned long
	adcResultTemp;			// Accumulator for averaging/oversampling
static unsigned char
	adcSampleCount;			// How many samples have we pulled in for this particular measurement?

static void InitAdc(void)
// For ADCA.
{
	unsigned char
		i;

	// Init ADC hardware:

	PR.PRPA&=~PR_ADC_bm;									// Make sure ADC is on
	PORTA.DIRCLR=0x0F;										// PORTA analog inputs to inputs

//	ADCA.CTRLB=ADC_CONVMODE_bm|ADC_RESOLUTION_12BIT_gc;		// Set signed mode, 12 bit right justified, no free run
	ADCA.CTRLB=0x10;										// Same as above, no "convmode" bitmask
	ADCA.REFCTRL=ADC_REFSEL_AREFA_gc;						// External reference A, bandgap and temp disabled
//	ADCA.PRESCALER=ADC_PRESCALER_DIV256_gc;					// 16MHz divided by 256 = 62.5kHz  (see notes)	-- OG, helps with higher impedance sources
//	ADCA.PRESCALER=ADC_PRESCALER_DIV128_gc;					// 16MHz divided by 128 = 125kHz  (see notes)	-- this works pretty well, maybe better than above at low impedance
	ADCA.PRESCALER=ADC_PRESCALER_DIV512_gc;					// See notes	-- real slow for high impedance

	ADCA.CH0.CTRL=ADC_CH_INPUTMODE_DIFF_gc;							// Differential input, no gain.
	ADCA.CH0.MUXCTRL=ADC_CH_MUXPOS_PIN2_gc|ADC_CH_MUXNEG_PIN3_gc;	// Positive input is PIN2 (PA2), Neg is PA3	(first source we look at)
	ADCA.CH0.INTCTRL=0;												// No interrupts

	ADCA.CTRLA=ADC_ENABLE_bm;								// Enable the ADC

	// Init our application ADC variables:

	adcResultTemp=0;	// Zero our accumulator
	adcSampleCount=0;	// Zero oversample/average counter

	for(i=0;i<NUM_ADC;i++)	// Zero results
	{
		adcResults[i]=0;
		newAdcResult[i]=false;
	}

	currentAdcSource=0;	// Look at the first signal source

	ADCA.CH0.CTRL|=ADC_CH_START_bm; 	// Start conversion on channel 0

}

static void UpdateAdc(void)
// Scan through positioner, pressure, get the results and put them in their respective result registers
// Motor current monitor is done on its own time (should go fast, doesn't need accuracy)
{
	unsigned char
		i;
	int
		temp;
	static unsigned char
		tossCounter=SAMPLES_TO_TOSS;

	for(i=0;i<NUM_ADC;i++)		// New results only stay new for a loop
	{
		newAdcResult[i]=false;
	}

	if(ADCA.CH0.INTFLAGS&ADC_CH0IF_bm)	// Got a complete conversion on CH0?
	{
		temp=ADCA.CH0RES;			// Get result

		if(tossCounter)				// Throw out a couple results while we let mux settle (may not be necessary, but...)
		{
			tossCounter--;
		}
		else
		{
			if(temp<0)					// Pin results positive
			{
				temp=0;
			}

			adcResultTemp+=temp;		// Add to accumulator
			adcSampleCount++;			// One more sample gathered

			if(adcSampleCount>=SAMPLES_TO_ADD)	// Got enough samples?
			{
				adcResults[currentAdcSource]=(adcResultTemp/SAMPLES_TO_AVERAGE);	// export reading, with averaging
				newAdcResult[currentAdcSource]=true;								// Let program know we have a new ADC reading

				adcSampleCount=0;						// Restart accumulator
				tossCounter=SAMPLES_TO_TOSS;			// Ignore initial samples after switching the mux
				adcResultTemp=0;						// Clear temp register

				currentAdcSource++;						// Get our next signal source
				if(currentAdcSource>=NUM_ADC)			// Roll it around
				{
					currentAdcSource=0;
				}

				switch(currentAdcSource)		// Set mux to the next signal source
				{
					case ADC_YOKE_PITCH:											// PA2
					ADCA.CH0.CTRL=ADC_CH_INPUTMODE_DIFF_gc;							// Differential input, no gain.
					ADCA.CH0.MUXCTRL=ADC_CH_MUXPOS_PIN2_gc|ADC_CH_MUXNEG_PIN3_gc;	// Positive input is PIN2 (PA2), Neg is PA3
					break;

					case ADC_YOKE_ROLL:												// PA1
					ADCA.CH0.CTRL=ADC_CH_INPUTMODE_DIFF_gc;							// Differential input, no gain.
					ADCA.CH0.MUXCTRL=ADC_CH_MUXPOS_PIN1_gc|ADC_CH_MUXNEG_PIN3_gc;	// Positive input is PIN1 (PA1), Neg is PA3
					break;

					default:
					// should probably either wdr or at least reset ADC
					// *** Bad execution ***
					break;
				}
			}

			ADCA.CH0.INTFLAGS|=ADC_CH0IF_bm;	// Clear flag
			ADCA.CH0.CTRL|=ADC_CH_START_bm; 	// Start conversion on channel 0
		}
	}
}


//-----------------------------------------------------------------------
//-----------------------------------------------------------------------
// Output Handling
//-----------------------------------------------------------------------
//-----------------------------------------------------------------------
#define		SER_CLK_MASK	(1<<6)		// On PORTA
#define		SER_DATA_MASK	(1<<7)		// On PORTA

static unsigned char
	outputByteHigh,
	outputByteMiddle,
	outputByteLow;

static void InitSerialLeds(void)
// Clock "off" bits out to the leds on the serial-to-parallel latch, make sure we start with LEDs off.
{
	unsigned char
		i;

	PORTA.OUTCLR=SER_CLK_MASK;		// Clock starts low
	PORTA.OUTSET=SER_DATA_MASK;		// Inverted -- a set bit turns the LED off.
	MACRO_DoTenNops;
	
	for(i=0;i<20;i++)
	{
		PORTA.OUTCLR=SER_CLK_MASK;		// Bring clock low
		MACRO_DoTenNops;				// Wait
		PORTA.OUTSET=SER_DATA_MASK;		// Set data correctly
		MACRO_DoTenNops;				// Wait
		PORTA.OUTSET=SER_CLK_MASK;		// Bring clock high (latch in data)
		MACRO_DoTenNops;				// Wait
	}

	PORTA.OUTCLR=SER_CLK_MASK;		// Clock ends low
	PORTA.OUTSET=SER_DATA_MASK;		// Data ends high
}

static void InitOutputs(void)
// Turn all output pins to drivers and set them low
{
	PORTC.OUTCLR=0xE0;
	PORTC.DIRSET=0xE0;		

	PORTD.OUTCLR=0xFF;		
	PORTD.DIRSET=0xFF;		

	PORTE.OUTCLR=0x03;		
	PORTE.DIRSET=0x03;		

	PORTA.OUTCLR=0xF0;		
	PORTA.DIRSET=0xF0;		

	// Initialize 595 outputs:
	InitSerialLeds();	
}

static void SetSerialLeds(void)
// Sets the LEDs on the 595 based on the status of our output mask
// Outputs of 595:
// ------------------
//	Qa		Green 0
//	Qb		Green 1
//	Qc		Green 2
//	Qd		Green 3
//	Qe		Red 0
//	Qf		Red 1
//	Qg		Red 2
//	Qh		Red 3
// -------------------
// Data propogates from Qa to Qh.  So if you clock in 8 bits, the first bit in ends up in Qh.
// Per the DS:
// "If both clocks are connected together, the shift register is always one clock pulse ahead of the storage register"
// Pretty sure this means we need to toggle the clock once more when we're done.
{
	unsigned char
		i,
		serOutputMask;
		
	serOutputMask=0xFF;				// Start with mask set to off

	// Put bits into mask in the correct order
	if(outputByteLow&Om_RED_LED_3)
	{
		serOutputMask&=~(1<<0);
	}
	if(outputByteLow&Om_RED_LED_2)
	{
		serOutputMask&=~(1<<1);
	}
	if(outputByteLow&Om_RED_LED_1)
	{
		serOutputMask&=~(1<<2);
	}
	if(outputByteLow&Om_RED_LED_0)
	{
		serOutputMask&=~(1<<3);
	}

	if(outputByteLow&Om_GREEN_LED_3)
	{
		serOutputMask&=~(1<<4);
	}
	if(outputByteLow&Om_GREEN_LED_2)
	{
		serOutputMask&=~(1<<5);
	}
	if(outputByteLow&Om_GREEN_LED_1)
	{
		serOutputMask&=~(1<<6);
	}
	if(outputByteLow&Om_GREEN_LED_0)
	{
		serOutputMask&=~(1<<7);
	}

	for(i=0;i<8;i++)	// Clock out one byte (clock should always start low)
	{
		if(serOutputMask&(1<<i))			// Set data correctly		
		{
			PORTA.OUTSET=SER_DATA_MASK;		
		}
		else
		{
			PORTA.OUTCLR=SER_DATA_MASK;				
		}
		PORTA.OUTSET=SER_CLK_MASK;		// Bring clock high (latch in data)
		PORTA.OUTCLR=SER_CLK_MASK;		// Bring clock low
	}

	// Clock out one remaining bit to get 595 registers in sync

	PORTA.OUTSET=SER_DATA_MASK;		// Don't care (off)
	PORTA.OUTSET=SER_CLK_MASK;		// Bring clock high (latch in data)
	PORTA.OUTCLR=SER_CLK_MASK;		// Bring clock low
}

static void SetOutputs(void)
// Sets output bits based on the value of the output masks
{
	// PORTC
	// -----------------------------------------
	if(outputByteHigh&Om_FLIGHT_IND_0)
	{
		PORTC.OUTSET=(1<<5);
	}
	else
	{
		PORTC.OUTCLR=(1<<5);	
	}
	if(outputByteHigh&Om_FLIGHT_IND_1)
	{
		PORTC.OUTSET=(1<<6);
	}
	else
	{
		PORTC.OUTCLR=(1<<6);	
	}
	if(outputByteHigh&Om_FLIGHT_IND_2)
	{
		PORTC.OUTSET=(1<<7);
	}
	else
	{
		PORTC.OUTCLR=(1<<7);	
	}

	// PORTD
	// -----------------------------------------
	if(outputByteHigh&Om_FLIGHT_IND_3)
	{
		PORTD.OUTSET=(1<<0);
	}
	else
	{
		PORTD.OUTCLR=(1<<0);	
	}
	if(outputByteHigh&Om_FLIGHT_IND_4)
	{
		PORTD.OUTSET=(1<<1);
	}
	else
	{
		PORTD.OUTCLR=(1<<1);	
	}
	if(outputByteMiddle&Om_FLIGHT_IND_5)
	{
		PORTD.OUTSET=(1<<2);
	}
	else
	{
		PORTD.OUTCLR=(1<<2);	
	}
	if(outputByteMiddle&Om_FLIGHT_IND_6)
	{
		PORTD.OUTSET=(1<<3);
	}
	else
	{
		PORTD.OUTCLR=(1<<3);	
	}
	if(outputByteMiddle&Om_FLIGHT_IND_7)
	{
		PORTD.OUTSET=(1<<4);
	}
	else
	{
		PORTD.OUTCLR=(1<<4);	
	}
	if(outputByteMiddle&Om_FLIGHT_IND_8)
	{
		PORTD.OUTSET=(1<<5);
	}
	else
	{
		PORTD.OUTCLR=(1<<5);	
	}
	if(outputByteMiddle&Om_FLIGHT_IND_9)
	{
		PORTD.OUTSET=(1<<6);
	}
	else
	{
		PORTD.OUTCLR=(1<<6);	
	}
	if(outputByteMiddle&Om_AIR_HORN)
	{
		PORTD.OUTSET=(1<<7);
	}
	else
	{
		PORTD.OUTCLR=(1<<7);	
	}

	// PORTE
	// -----------------------------------------
	if(outputByteMiddle&Om_PUSHBUTTON_LAMP_0)
	{
		PORTE.OUTSET=(1<<0);
	}
	else
	{
		PORTE.OUTCLR=(1<<0);	
	}
	if(outputByteMiddle&Om_PUSHBUTTON_LAMP_1)
	{
		PORTE.OUTSET=(1<<1);
	}
	else
	{
		PORTE.OUTCLR=(1<<1);	
	}

	SetSerialLeds();		// Clock data out to 595	
}

//-----------------------------------------------------------------------
//-----------------------------------------------------------------------
// Serial
//-----------------------------------------------------------------------
//-----------------------------------------------------------------------

#define		INCOMING_MESSAGE_TIMEOUT			(SECOND/8)

static unsigned char
	analogRoll,
	analogPitch;

ISR(PORTD_INT0_vect)	
{
}


static void UpdateIncomingSerial(void)
// Inhales incoming serial from the PC and puts it out to the program when the entire message comes in.
{
	unsigned char
		byte;

	static unsigned char
		rxMessageState;
	bool
		gotMessage;
	
	gotMessage=false;	// No complete message yet

	if(CheckTimer(TIMER_INCOMING_MESSAGE_TIMEOUT))		// If we don't get a byte for some period of time, reset our collection state machine
	{
		rxMessageState=0;
	}
	
	while((RxFifoNotEmpty())&&(gotMessage==false))	// Loop here handling bytes we've gotten from the cpu until the fifo is empty OR we have a message to send to the program
	{
		SetTimer(TIMER_INCOMING_MESSAGE_TIMEOUT,INCOMING_MESSAGE_TIMEOUT);	// Got new bytes in the fifo, so don't time out.
		byte=GetByteFromRxFifo();											// Inhale it
		switch(rxMessageState)
		{
			case 0:
				if(byte==0x42)			// Correct poll byte
				{
					rxMessageState=1;
				}
				break;
			case 1:
				outputByteHigh=byte;
				rxMessageState=2;
				break;
			case 2:
				outputByteMiddle=byte;
				rxMessageState=3;
				break;
			case 3:
				outputByteLow=byte;
				rxMessageState=0;
				gotMessage=true;
				break;
		}
	}

	if(gotMessage)
	{
		SetOutputs();

		// Now send out our reply

		if(txBytesToSend==0)				// Are we done sending the last message to the host?
		{
			txBuffer[0]=0x41;
			txBuffer[1]=(unsigned char)((keyState>>8)&0xFF);
			txBuffer[2]=(unsigned char)(keyState&0xFF);
			txBuffer[3]=0;
			txBuffer[4]=0;
			txBuffer[5]=analogRoll;
			txBuffer[6]=analogPitch;
				
			txBytesToSend=7;
			txBufferIndex=0;
		}
	}
}

static void UpdateOutgoingSerial(void)
// Keep bytes going out to the host if necessary
{
	if(txBytesToSend)	// Anything to go out?
	{
		if(UartTxBufferReady())		// Room in the Uart TX fifo?
		{
			UartSendByte(txBuffer[txBufferIndex++]);
			txBytesToSend--;
		}
	}
}

//-----------------------------------------------------------------------
//-----------------------------------------------------------------------
// High Level
//-----------------------------------------------------------------------
//-----------------------------------------------------------------------

static void DoUpdateIo(void)
// Keep reading the switches, and update indicators when told
{

	// If new ADC reading, put it in the mask
	if(newAdcResult[ADC_YOKE_PITCH]==true)
	{
		analogPitch=(unsigned char)(adcResults[ADC_YOKE_PITCH]/8);		// Get back to 8 bits
	}
	if(newAdcResult[ADC_YOKE_ROLL]==true)
	{
		analogRoll=(unsigned char)(adcResults[ADC_YOKE_ROLL]/8);		// Get back to 8 bits
	}

	UpdateIncomingSerial();		// If we get a full message from the PC, do what it says and send a reply
	if(txBytesToSend)
	{
		UpdateOutgoingSerial();
	}
}

#define		HORN_INTRO_TIME		(SECOND/4)
#define		CHASE_TIME			(SECOND/8)
#define		LAMP_TIME			((SECOND*4)/3)


static void DoStartupTest(void)
// At power on, go here and make pretty / diagnostic light chases until the game boots.
// Leave this state when we start receiving bytes over serial.
{
	static unsigned char
		lastOutputByteHigh,
		lastOutputByteMiddle,
		lastOutputByteLow;

	static unsigned char
		shiftIndex,
		lampCounter;
	
	if(subState==SS_0)					// Horn intro
	{
		SetTimer(TIMER_1,(HORN_INTRO_TIME));
		outputByteMiddle=Om_AIR_HORN;
		outputByteHigh=(Om_FLIGHT_IND_0|Om_FLIGHT_IND_1);
		outputByteMiddle|=Om_PUSHBUTTON_LAMP_1;
		SetOutputs();
		subState=SS_1;
	}
	else if(subState==SS_1)
	{
		if(CheckTimer(TIMER_1))
		{
			SetTimer(TIMER_1,(HORN_INTRO_TIME));
			outputByteMiddle=0;
			outputByteHigh=(Om_FLIGHT_IND_0|Om_FLIGHT_IND_1|Om_FLIGHT_IND_2|Om_FLIGHT_IND_3);
			outputByteMiddle|=Om_PUSHBUTTON_LAMP_0;
			SetOutputs();
			subState=SS_2;	
		}
	}
	else if(subState==SS_2)
	{
		if(CheckTimer(TIMER_1))
		{
			SetTimer(TIMER_1,(HORN_INTRO_TIME));
			outputByteMiddle=Om_AIR_HORN;
			outputByteHigh=(Om_FLIGHT_IND_2|Om_FLIGHT_IND_3|Om_FLIGHT_IND_4);
			outputByteMiddle|=Om_FLIGHT_IND_5;
			outputByteLow=Om_RED_LED_3;
			SetOutputs();
			subState=SS_3;	
		}
	}
	else if(subState==SS_3)
	{
		if(CheckTimer(TIMER_1))
		{
			SetTimer(TIMER_1,(HORN_INTRO_TIME));
			outputByteMiddle=0;
			outputByteHigh=Om_FLIGHT_IND_4;
			outputByteMiddle|=(Om_FLIGHT_IND_5|Om_FLIGHT_IND_6|Om_FLIGHT_IND_7);
			outputByteLow=Om_RED_LED_2;
			SetOutputs();
			subState=SS_4;	
		}
	}
	else if(subState==SS_4)
	{
		if(CheckTimer(TIMER_1))
		{
			SetTimer(TIMER_1,(HORN_INTRO_TIME));
			outputByteMiddle=Om_AIR_HORN;
			outputByteHigh=0;
			outputByteMiddle|=(Om_FLIGHT_IND_6|Om_FLIGHT_IND_7|Om_FLIGHT_IND_8|Om_FLIGHT_IND_9);
			outputByteLow=Om_RED_LED_1;
			SetOutputs();
			subState=SS_5;	
		}
	}
	else if(subState==SS_5)
	{
		if(CheckTimer(TIMER_1))
		{
			SetTimer(TIMER_1,(HORN_INTRO_TIME));
			outputByteMiddle=0;
			outputByteMiddle|=(Om_FLIGHT_IND_8|Om_FLIGHT_IND_9);
			outputByteLow=Om_RED_LED_0;
			SetOutputs();
			subState=SS_6;	
		}
	}
	else if(subState==SS_6)
	{
		if(CheckTimer(TIMER_1))
		{
			SetTimer(TIMER_1,(HORN_INTRO_TIME));
			outputByteMiddle=Om_AIR_HORN;
			outputByteLow=(Om_GREEN_LED_0|Om_GREEN_LED_1|Om_GREEN_LED_2|Om_GREEN_LED_3);

			SetOutputs();
			subState=SS_7;	
		}
	}
	else if(subState==SS_7)
	{
		if(CheckTimer(TIMER_1))
		{
			SetTimer(TIMER_1,(CHASE_TIME));
			SetTimer(TIMER_2,(LAMP_TIME));

			outputByteHigh=0;
			outputByteMiddle=0;
			outputByteLow=0;
			SetOutputs();

			shiftIndex=0;
			lampCounter=0;

			subState=SS_8;	
		}
	}	
	// LIGHT CHASE -----------------------------
	else if(subState==SS_8)
	{
		lastOutputByteHigh=outputByteHigh;
		lastOutputByteMiddle=outputByteMiddle;
		lastOutputByteLow=outputByteLow;
	
		if(CheckTimer(TIMER_1))		// Update Green LEDs
		{
			shiftIndex++;

			if(shiftIndex<5)	// LEDs marching up
			{
				// Clear Green LEDs
				outputByteLow&=~(Om_GREEN_LED_0|Om_GREEN_LED_1|Om_GREEN_LED_2|Om_GREEN_LED_3);

				if(shiftIndex>=1)
				{
					outputByteLow|=Om_GREEN_LED_0;
				}
				if(shiftIndex>=2)
				{
					outputByteLow|=Om_GREEN_LED_1;
				}
				if(shiftIndex>=3)
				{
					outputByteLow|=Om_GREEN_LED_2;
				}
				if(shiftIndex>=4)
				{
					outputByteLow|=Om_GREEN_LED_3;
				}
			}
			else if(shiftIndex<9)	// Leds marching off
			{
				// Set Green LEDs
				outputByteLow|=(Om_GREEN_LED_0|Om_GREEN_LED_1|Om_GREEN_LED_2|Om_GREEN_LED_3);

				if(shiftIndex>=5)
				{
					outputByteLow&=~Om_GREEN_LED_0;
				}
				if(shiftIndex>=6)
				{
					outputByteLow&=~Om_GREEN_LED_1;
				}
				if(shiftIndex>=7)
				{
					outputByteLow&=~Om_GREEN_LED_2;
				}
				if(shiftIndex>=8)
				{
					outputByteLow&=~Om_GREEN_LED_3;
				}			
			}
			else if(shiftIndex<13)	// No green LEDs, pause
			{
				outputByteLow&=~(Om_GREEN_LED_0|Om_GREEN_LED_1|Om_GREEN_LED_2|Om_GREEN_LED_3);
			}
			else
			{
				shiftIndex=0;
			}

			SetTimer(TIMER_1,(CHASE_TIME));
		}
		
		if(CheckTimer(TIMER_2))		// Chase pushbutton lamps if we aren't fussing with them
		{
			lampCounter++;
			if(lampCounter>=4)
			{
				lampCounter=0;
			}

			outputByteMiddle&=~Om_PUSHBUTTON_LAMP_0;		
			outputByteMiddle&=~Om_PUSHBUTTON_LAMP_1;		

			if(lampCounter&0x01)
			{
				outputByteMiddle|=Om_PUSHBUTTON_LAMP_0;
			}
			if(lampCounter&0x02)
			{
				outputByteMiddle|=Om_PUSHBUTTON_LAMP_1;
			}

			SetTimer(TIMER_2,(LAMP_TIME));
		}

		// Set pushbutton lamps if we're holding them, clear on release
		if(keyState&Im_PUSHBUTTON_0)		
		{
			outputByteMiddle|=Om_PUSHBUTTON_LAMP_0;
		}
		else if(newKeysReleased&Im_PUSHBUTTON_0)
		{
			outputByteMiddle&=~Om_PUSHBUTTON_LAMP_0;		
		}

		if(keyState&Im_PUSHBUTTON_1)		
		{
			outputByteMiddle|=Om_PUSHBUTTON_LAMP_1;
		}
		else if(newKeysReleased&Im_PUSHBUTTON_1)
		{
			outputByteMiddle&=~Om_PUSHBUTTON_LAMP_1;		
		}

		// Clear flight indicators
		outputByteHigh=0;
		outputByteMiddle&=~(Om_FLIGHT_IND_5|Om_FLIGHT_IND_6|Om_FLIGHT_IND_7|Om_FLIGHT_IND_8|Om_FLIGHT_IND_9);
		
		// Clear Red LEDs
		outputByteLow&=~(Om_RED_LED_0|Om_RED_LED_1|Om_RED_LED_2|Om_RED_LED_3);

		// Put toggle switches directly on red LEDs
		if(keyState&Im_TOGGLE_0)		
		{
			outputByteLow|=Om_RED_LED_0;
		}
		if(keyState&Im_TOGGLE_1)		
		{
			outputByteLow|=Om_RED_LED_1;
		}
		if(keyState&Im_TOGGLE_2)		
		{
			outputByteLow|=Om_RED_LED_2;
		}
		if(keyState&Im_TOGGLE_3)		
		{
			outputByteLow|=Om_RED_LED_3;
		}

		// Triggers to flight indicators

		if(keyState&Im_R_MAIN_TRIG)
		{
			outputByteHigh|=Om_FLIGHT_IND_0;
		}
		if(keyState&Im_R_THUMB_TRIG)
		{
			outputByteHigh|=Om_FLIGHT_IND_1;
		}
		if(keyState&Im_R_GRIP_TRIG)
		{
			outputByteHigh|=Om_FLIGHT_IND_2;
		}
		if(keyState&Im_L_MAIN_TRIG)
		{
			outputByteHigh|=Om_FLIGHT_IND_3;
		}
		if(keyState&Im_L_THUMB_TRIG)
		{
			outputByteHigh|=Om_FLIGHT_IND_4;
		}
		if(keyState&Im_L_GRIP_TRIG)
		{
			outputByteMiddle|=Om_FLIGHT_IND_5;
		}

		// Analog to flight indicators

		if(newAdcResult[ADC_YOKE_PITCH]==true)
		{
			analogPitch=(unsigned char)(adcResults[ADC_YOKE_PITCH]/8);		// Get back to 8 bits
		}
		if(newAdcResult[ADC_YOKE_ROLL]==true)
		{
			analogRoll=(unsigned char)(adcResults[ADC_YOKE_ROLL]/8);		// Get back to 8 bits
		}

		if(analogPitch<70)
		{
			outputByteMiddle|=Om_FLIGHT_IND_6;
		}
		if(analogPitch>140)
		{
			outputByteMiddle|=Om_FLIGHT_IND_7;
		}
		if(analogRoll<100)
		{
			outputByteMiddle|=Om_FLIGHT_IND_8;
		}
		if(analogRoll>130)
		{
			outputByteMiddle|=Om_FLIGHT_IND_9;
		}

		// Sound of the police
		if(keyState&Im_COIN)
		{
			SetTimer(TIMER_3,(SECOND));
			outputByteMiddle|=Om_AIR_HORN;
		}

		if(CheckTimer(TIMER_3))
		{
			outputByteMiddle&=~Om_AIR_HORN;			
		}

		// Test button resets chases
		if(keyState&Im_TEST)
		{
			subState=SS_0;
		}

		if((lastOutputByteHigh!=outputByteHigh)||(lastOutputByteMiddle!=outputByteMiddle)||(lastOutputByteLow!=outputByteLow))		// Update outputs if anything changed this time around.
		{
			SetOutputs();
		}
	}

	if(RxFifoNotEmpty())		// Once the game takes control, stop messing with outputs.
	{
		outputByteHigh=0;
		outputByteMiddle=0;
		outputByteLow=0;
		SetOutputs();
		SetState(DoUpdateIo);
	}
}

//-----------------------------------------------------------------------
//-----------------------------------------------------------------------
// Program main loop:
//-----------------------------------------------------------------------
//-----------------------------------------------------------------------

int main(void)
// Application main loop.
{
	cli();			// No interrupts until we're ready.

	InitOutputs();			// Turn off all outputs
	
	// Xmega parts start running an internal clock at 2MHZ -- we change that in software.
	// This board has a 20MHz silicon oscillator hooked up to the MCU which we use as the CPU clock

	OSC.XOSCCTRL=OSC_XOSCSEL_EXTCLK_gc;				// No crystal oscillator, set up for external clock in (this is really just setting this register to 0 -- this also starts up fast)
	OSC.PLLCTRL=(0b00<<OSC_PLLSRC_gp)|(0<<OSC_PLLFAC_gp);	// no PLL
	OSC.DFLLCTRL=0;						// no calibration
	OSC.XOSCFAIL=0;						// no failure monitoring

	OSC.CTRL=(0<<OSC_PLLEN_bp)|(1<<OSC_XOSCEN_bp)|(0<<OSC_RC32KEN_bp)|(0<<OSC_RC32MEN_bp)|(1<<OSC_RC2MEN_bp);	// turn on external clock, (leave internal 2MHz running)

	while(!(OSC.STATUS&(1<<OSC_XOSCRDY_bp)))		// sit here until external clock is alive
		;

	CCPWrite(&CLK.PSCTRL,0);						//	No prescaling
	CCPWrite(&CLK.CTRL,CLK_SCLKSEL_XOSC_gc);		//	Set ext oscillator as the clock source
	CLK.RTCCTRL=CLK_RTCSRC_RCOSC_gc|0;				// select internal 32.748KHz RC, but leave it disabled

	CCPWrite(&OSC.XOSCFAIL,OSC_XOSCFDEN_bm);		//  Enable failure detection on internal oscillator (reset if it gets real weird)
	OSC.CTRL&=~(1<<OSC_RC2MEN_bp);					// turn off 2MHz clock now that we're running from the PLL/external OSC

	// Ports and INIT

//	PR.PRGEN=0x1F;									// Power off AES, EBI, RTC, event system, and DMA
//  @@@ power off that stuff once you're sure you aren't using it.
//  Power off the port peripherals too.

	CCPWrite(&MCU.MCUCR,MCU_JTAGD_bm);				//  Disable JTAG (screws up PORTB)

	InitUart();
	InitSerialFifos();
//	InitSpi();
	InitAdc();
	InitSwitches();
	InitSoftclock();
	fdevopen(UartPutChar, NULL);	// Associate stdout with out putchar routine (enable printf)  --- This is REALLY hoggy.

	PMIC.CTRL=PMIC_HILVLEN_bm|PMIC_MEDLVLEN_bm|PMIC_LOLVLEN_bm;		// Enable all interrupt levels
	sei();															// Global interrupt enable

	RST.STATUS=0x3F;				// Clear reset cause flags	
	
	printf("\nI live! %s\n",BUILD_DATE);

	SetState(DoStartupTest);

	while(1)
	{
		UpdateAdc();
		HandleSwitches();
		State();	// Execute the current program state
	}
	return(0);	// Keep GCC from complaining.
}
