//-----------------------------------------------------------------------
//-----------------------------------------------------------------------
// UART functions.
//-----------------------------------------------------------------------
//-----------------------------------------------------------------------
// For XMEGA MCUs.
// NOTE -- we are associated with one uart here only, this is not great use of the xmega register-as-struct concept.
// Mon Oct 31 16:06:56 EDT 2011

#include "includes.h"

#define		USART		USARTF0

bool UartTxBufferReady(void)
{
	if(USART.STATUS&USART_DREIF_bm)		// Room in the transmit buffer? (bit is 1 when there's room)
	{
		return(true);
	}
	return(false);
}

void UartSendByte(unsigned char byte)
// Waits (forever if necessary) until the the send buffer is ready, then sends a byte out over the UART.
// NOTE -- this doesn't check whether the output shift register is still clocking out data (ie, whether transmission is complete) just whether the buffer is ready to get a new byte to transmit.
{
	while(!(USART.STATUS&USART_DREIF_bm))		// Hang here until there is room in the transmit buffer (bit is 1 when there's room)
		;
	USART.DATA=byte;							// Load the TX buffer.  The byte will clock out automagically.
}

unsigned char UartGetByte()
// Gets the first byte in the UART's receive buffer.
{
	return(USART.DATA);		// Get one byte back from the receive buffer.  Note that there may be (one) more in the FIFO.
}

bool UartGotByte()
// Returns true when there is unread data in the UART's receive buffer.
{
	if(USART.STATUS&USART_RXCIF_bm)		// Flag is set when there are unread data in the buffer
	{
		return(true);
	}
	return(false);
}

void UartWaitForByte()
// Hang out here (maybe forever) until we get a byte.
{
	while(!UartGotByte())				// If there's not new data in the buffer, wait here until there is.
		;
}

void UartFlushBuffer()
// Empties the serial buffer.
{
	while(UartGotByte())
	{
		UartGetByte();
	}
}

void UnInitUart()
// Undo what InitUart did.
{
	USART.CTRLB=0;		// Disable transmitter and receiver
}

void InitUart()
// This UART setup is for 38400 baud, 8 data bits, one stop bit, no parity, no flow control.
// Interrupts are disabled.
{
	// Port specific stuff:

	PR.PRPF&=~PR_USART0_bm;	// USART0 power on for this port

	PORTF.OUTSET=PIN3_bm;
	PORTF.DIRSET=PIN3_bm;	// PF3 (TXD0) as output driving high

	PORTF.DIRCLR=PIN2_bm;	// PF2 (RXD0) as input
	USART.CTRLB=0;			// Tx / Rx off for now, normal rate

	// Baud:
	#if F_CPU==16000000UL 
	// 16MHz peripheral clock, normal speed, 38.4kbaud: BSEL=25.042 (bscale=0) error == 0.16%
	USART.BAUDCTRLB=0;			// No bscale (no fractional rate)
	USART.BAUDCTRLA=25;			// 38,400 at 16MHz per clock (see above)
	#warning "Uart assuming 16Mhz F_CPU..."
	#elif F_CPU==20000000UL
	// 20MHz peripheral clock, double speed, 38.4kbaud: BSEL=64.10 (bscale=0) error == 0.16%
	USART.BAUDCTRLB=0;			// No bscale (no fractional rate)
	USART.BAUDCTRLA=64;			// 38,400 at 20MHz per clock (see above)
	USART.CTRLB|=(USART_CLK2X_bm);	// Set double rate
	#warning "Uart assuming 20Mhz F_CPU..."
	#else
	#error "Can't generate a baud rate from the current F_CPU."
	#endif

	USART.CTRLA=0;						// No interrupts
	USART.CTRLC=USART_CHSIZE_8BIT_gc;	// Set 8,N,1

	USART.CTRLB|=(USART_RXEN_bm|USART_TXEN_bm);	// Enable transmitter and reciever at normal rate
	UartFlushBuffer();							// Get rid of any poo poo hanging out in the input buffer.
}
