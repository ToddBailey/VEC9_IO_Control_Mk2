//-----------------------------------------------------------------------
//-----------------------------------------------------------------------
// Software Clock Functions:
//-----------------------------------------------------------------------
//-----------------------------------------------------------------------
// Using these routines requires that we've set up some way of incrementing "systemTicks".  This is usually through a hardware timer which is setting off a Periodic IRQ.
// In some apps we don't want an interrupt and we get ticks some other way.
// Remember: variables, bits changed inside ISRs and monitored inside the program should be declared volatile.

#include "includes.h"


static volatile unsigned int			// Local variables which keep track of timer stuff. 
	entryTime[NUM_TIMERS],
	delayTime[NUM_TIMERS];

void ResetTimer(unsigned char timerNum)
// Starts a given timer counting again from the time this function is called (resets the entryTime) using the last value of ticksToWait passed to that timer.
{
	unsigned char 
		sreg;

	sreg=SREG;	// Keep operations atomic -- ISR changes systemTicks.
	cli();
	entryTime[timerNum]=systemTicks;
	SREG=sreg;
}

void SetTimer(unsigned char timerNum, unsigned int ticks_to_wait)
// Sets a software timer with an entry time and an amount of time before it expires.
{
	unsigned char 
		sreg;

	sreg=SREG;	// Keep operations atomic -- ISR changes systemTicks.
	cli();

	entryTime[timerNum]=systemTicks;
	delayTime[timerNum]=ticks_to_wait;

	SREG=sreg;
}

unsigned char CheckTimer(unsigned char timerNum)
// If the current system time MINUS the entry time is greater than (or equal to) the amount of ticks we're supposed to wait, we've waited long enough.  Return true.
// Ie, return true if the time is up, and false if it isn't.
{
	unsigned char 
		sreg;

	sreg=SREG;	// Keep operations atomic -- ISR changes systemTicks.
	cli();
	
	if((systemTicks-entryTime[timerNum])>=delayTime[timerNum])
	{
		SREG=sreg;		// Restore interrupt state.
		return(true);
	}
	else
	{
		SREG=sreg;		// Restore interrupt state.
		return(false);
	}
}

void ExpireTimer(unsigned char timerNum)
// Sets a timer check to return false the next time it is checked.  IE, "runs out" the passed timer.
{
	delayTime[timerNum]=0;		// Zero ticks until we're expired.  
}

