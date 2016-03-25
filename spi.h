#define		SET_CS	(PORTD.OUTSET=0x10)
#define		CLR_CS	(PORTD.OUTCLR=0x10)

bool SpiTransferComplete(void);
unsigned char TransferSpiByte(unsigned char theByte);
void DoBusyWait(volatile unsigned int loops);
void InitSpi(void);
