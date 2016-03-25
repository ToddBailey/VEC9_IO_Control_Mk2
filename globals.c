// DEFINITIONS of all global variables (anything you want to declare "extern") should go here and only here just to keep things straight.
//--------------------------------------------------------------------------------------

#include "includes.h"

STATE_FUNC				//  Creates a pointer called State to an instance of STATE_FUNC().
	*State;
unsigned char
	subState;			//  Keeps track of the minor states (sub states) the device can be in.
volatile unsigned int	// This counter keeps track of timing ticks in the ISR and is referenced by the Timer routines.
	systemTicks;
