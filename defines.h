// Application Definitions and Typedefs go here, but not variable declarations:

#define	F_CPU	(20000000UL)
#define	SECOND	(3000)			// Softclock ticks in a second.  Dependent on the system clock frequency divided by the TIMER prescaler.

// Make a C version of "bool"
//----------------------------
typedef unsigned char bool;
#define		false			(0)
#define		true			(!(false))

typedef void				// Creates a datatype, here a void function called STATE_FUNC().
	STATE_FUNC(void);

enum	// A list of the sub states.  Propers to Todd Squires for this state machine stuff.
	{
		SS_0=0,
		SS_1,
		SS_2,
		SS_3,
		SS_4,
		SS_5,
		SS_6,
		SS_7,
		SS_8,
		SS_9,
		SS_10,
		SS_11,
		SS_12,
		SS_13,
		SS_14,
		SS_15,
		SS_16,
		SS_17,
		SS_18,
		SS_19,
		SS_20,
		SS_21,
		SS_22,
		SS_23,
		SS_24,
		SS_25,
		SS_26,
		SS_27,
		SS_28,
		SS_29,
		SS_30,
		SS_31,
		SS_32,
		SS_33,
		SS_34,
		SS_35,
		SS_36,
		SS_37,
		SS_38,
		SS_39,
		SS_40,
	};


// Fixed Delay Macro:
//----------------------------------------------

#define		MACRO_DoTenNops	asm volatile("nop"::); asm volatile("nop"::); asm volatile("nop"::); asm volatile("nop"::); asm volatile("nop"::); asm volatile("nop"::); asm volatile("nop"::); asm volatile("nop"::); asm volatile("nop"::); asm volatile("nop"::);

// Timers:
//-----------------------------------------------------------------------
// Software timer variables:
enum								// Add more timers here if you need them, but don't get greedy.
{
	TIMER_1=0,
	TIMER_2,
	TIMER_3,
	TIMER_DEBOUNCE,
	TIMER_INCOMING_MESSAGE_TIMEOUT,
	NUM_TIMERS,
};


//-----------------------------------------------------------------------
//-----------------------------------------------------------------------
//Application Defines:
//-----------------------------------------------------------------------
//-----------------------------------------------------------------------

// Input and output bit masks:

// Inputs (in keystate variable):
#define		I_R_MAIN_TRIG		0
#define		I_R_THUMB_TRIG		1
#define		I_R_GRIP_TRIG		2
#define		I_L_MAIN_TRIG		3
#define		I_L_THUMB_TRIG		4
#define		I_L_GRIP_TRIG		5
#define		I_TOGGLE_0			6
#define		I_TOGGLE_1			7
#define		I_TOGGLE_2			8
#define		I_TOGGLE_3			9
#define		I_PUSHBUTTON_0		10
#define		I_PUSHBUTTON_1		11
#define		I_COIN				12
#define		I_TEST				13

// Masks
#define		Im_R_MAIN_TRIG		(1<<I_R_MAIN_TRIG)
#define		Im_R_THUMB_TRIG		(1<<I_R_THUMB_TRIG)	
#define		Im_R_GRIP_TRIG		(1<<I_R_GRIP_TRIG)
#define		Im_L_MAIN_TRIG		(1<<I_L_MAIN_TRIG)	
#define		Im_L_THUMB_TRIG		(1<<I_L_THUMB_TRIG)	
#define		Im_L_GRIP_TRIG		(1<<I_L_GRIP_TRIG)
#define		Im_TOGGLE_0			(1<<I_TOGGLE_0)		
#define		Im_TOGGLE_1			(1<<I_TOGGLE_1)		
#define		Im_TOGGLE_2			(1<<I_TOGGLE_2)		
#define		Im_TOGGLE_3			(1<<I_TOGGLE_3)		
#define		Im_PUSHBUTTON_0		(1<<I_PUSHBUTTON_0)	
#define		Im_PUSHBUTTON_1		(1<<I_PUSHBUTTON_1)	
#define		Im_COIN				(1<<I_COIN)
#define		Im_TEST				(1<<I_TEST)


// Output bits (see wiring diagram for where these go in hardware)
// These are broken into 3 bytes rather than one long int.
// There are outputs on PORTs C,D,E, and F.

// outputByteHigh
#define		O_FLIGHT_IND_0			4		// PE0
#define		O_FLIGHT_IND_1			3		// PE1
#define		O_FLIGHT_IND_2			2		// PE2
#define		O_FLIGHT_IND_3			1		// PE3
#define		O_FLIGHT_IND_4			0		// PE4

// outputByteMid
#define		O_FLIGHT_IND_5			7		// PE5
#define		O_FLIGHT_IND_6			6		// PE6
#define		O_FLIGHT_IND_7			5		// PE7
#define		O_FLIGHT_IND_8			4		// PF0
#define		O_FLIGHT_IND_9			3		// PF1
#define		O_AIR_HORN				2		// PC5
#define		O_PUSHBUTTON_LAMP_0		1		// PC3
#define		O_PUSHBUTTON_LAMP_1		0		// PC4

// outputByteLow (NOTE, the LED bit polarities are inverted)
#define		O_GREEN_LED_0			7		// PD4
#define		O_GREEN_LED_1			6		// PD5
#define		O_GREEN_LED_2			5		// PD6
#define		O_GREEN_LED_3			4		// PD7
#define		O_RED_LED_0				3		// PD0
#define		O_RED_LED_1				2		// PD1
#define		O_RED_LED_2				1		// PD2
#define		O_RED_LED_3				0		// PD3

// Output Mask

#define		Om_FLIGHT_IND_0			(1<<O_FLIGHT_IND_0)
#define		Om_FLIGHT_IND_1			(1<<O_FLIGHT_IND_1)		
#define		Om_FLIGHT_IND_2			(1<<O_FLIGHT_IND_2)		
#define		Om_FLIGHT_IND_3			(1<<O_FLIGHT_IND_3)		
#define		Om_FLIGHT_IND_4			(1<<O_FLIGHT_IND_4)		
#define		Om_FLIGHT_IND_5			(1<<O_FLIGHT_IND_5)		
#define		Om_FLIGHT_IND_6			(1<<O_FLIGHT_IND_6)		
#define		Om_FLIGHT_IND_7			(1<<O_FLIGHT_IND_7)		
#define		Om_FLIGHT_IND_8			(1<<O_FLIGHT_IND_8)		
#define		Om_FLIGHT_IND_9			(1<<O_FLIGHT_IND_9)		
#define		Om_AIR_HORN				(1<<O_AIR_HORN)	
#define		Om_PUSHBUTTON_LAMP_0	(1<<O_PUSHBUTTON_LAMP_0)		
#define		Om_PUSHBUTTON_LAMP_1	(1<<O_PUSHBUTTON_LAMP_1)
#define		Om_GREEN_LED_0			(1<<O_GREEN_LED_0)
#define		Om_GREEN_LED_1			(1<<O_GREEN_LED_1)		
#define		Om_GREEN_LED_2			(1<<O_GREEN_LED_2)		
#define		Om_GREEN_LED_3			(1<<O_GREEN_LED_3)		
#define		Om_RED_LED_0			(1<<O_RED_LED_0)		
#define		Om_RED_LED_1			(1<<O_RED_LED_1)		
#define		Om_RED_LED_2			(1<<O_RED_LED_2)		
#define		Om_RED_LED_3			(1<<O_RED_LED_3)		
