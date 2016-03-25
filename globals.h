// Declarations of variables as external (stuff we use across several .c files) go here.
// The INSTANTIATION of the variable should go in globals.c
//--------------------------------------------------------------------------------------


extern STATE_FUNC		//  Creates a pointer called State to an instance of STATE_FUNC().
	*State;
extern unsigned char
	subState;						//  Keeps track of the minor states (sub states) the device can be in.
extern volatile unsigned int		// This counter keeps track of timing ticks in the ISR and is referenced by the Timer routines.
	systemTicks;
