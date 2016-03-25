// SPI library for Xmegas.
// Fri Jul 19 17:42:41 EDT 2013
// TMB

// Pins:
// PD4		Chip Select
// PD5		MOSI
// PD6		MISO
// PD7		SCK


#include	"includes.h"

// -------------------------
// Control Line Macros
// -------------------------
// See SPI.h
// -------------------------
// Low Level Functions:
// -------------------------

bool SpiTransferComplete(void)
// Returns true if the SPI transfer interrupt complete flag is SET
{
	if(SPID.STATUS&SPI_IF_bm)
	{
		return(true);
	}
	return(false);
}

unsigned char TransferSpiByte(unsigned char theByte)
// Loads a byte into the SPI transmit shift register, waits until the transfer is complete, and then returns the byte it's gotten from the slave.
// Checks to make sure the transmitter is ready first.
{

	SPID.DATA=theByte;				// Load the xmit buffer and start the transfer.

	while(!(SPID.STATUS&SPI_IF_bm))	// Spin until the transfer is complete
		;
	return(SPID.DATA);
}

void DoBusyWait(volatile unsigned int loops)
// Pull ~10 operations per loop plus overhead (probably more like 15)
{
	while(loops--)
	{
		MACRO_DoTenNops;
	}
}


void InitSpi(void)
{
	PORTD.OUTCLR=0xF0;		// All lines low to start
	PORTD.DIRSET=0xB0;		// SCK, MOSI, SS to output
	
	PORTD.DIRCLR=(1<<6);	// MISO to input
	
	PR.PRPD&=~PR_SPI_bm;				// Turn SPI module on (on port D)
	PORTD.PIN6CTRL=PORT_OPC_PULLUP_gc;	// Pull up inputs so they don't flop around

	PORTD.OUTSET=0x10;		// PD4 high (chip select)

	SPID.INTCTRL=0;											// No interrupts

	// Examples:
//	SPID.CTRL=SPI_ENABLE_bm|SPI_MASTER_bm|SPI_PRESCALER_DIV64_gc|SPI_MODE_3_gc;	// Enable SPI, Master, clk/64, MSb first, SPI mode 3
//	SPID.CTRL=SPI_CLK2X_bm|SPI_ENABLE_bm|SPI_MASTER_bm|SPI_PRESCALER_DIV16_gc|SPI_MODE_3_gc;	// Enable SPI, Master, clk/8, MSb first, SPI mode 3
//	SPID.CTRL=SPI_CLK2X_bm|SPI_ENABLE_bm|SPI_MASTER_bm|SPI_PRESCALER_DIV4_gc|SPI_MODE_3_gc;	// Enable SPI, Master, clk/2, MSb first, SPI mode 3

//	SPID.CTRL=SPI_ENABLE_bm|SPI_MASTER_bm|SPI_PRESCALER_DIV16_gc|SPI_MODE_0_gc;	// Enable SPI, Master, clk/16, MSb first, SPI mode 0

	SPID.CTRL=SPI_ENABLE_bm|SPI_DORD_bm|SPI_MASTER_bm|SPI_PRESCALER_DIV64_gc|SPI_MODE_3_gc;	// Enable SPI, LSb first, Master, clk/64, SPI mode 3
}