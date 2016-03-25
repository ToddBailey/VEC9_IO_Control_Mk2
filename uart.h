bool UartTxBufferReady(void);
void UartSendByte(unsigned char byte);
unsigned char UartGetByte(void);
bool UartGotByte(void);
void UartWaitForByte(void);
void UartFlushBuffer(void);
void UnInitUart(void);
void InitUart(void);
