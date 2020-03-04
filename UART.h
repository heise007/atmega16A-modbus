#define __DIS_UART_INT	{UCSRB &= 0x3f;}
#define __EN_UART_INT	{UCSRB |= 0xc0;}

extern char UART_Init(long int Baud);
extern char UART_PutChar(unsigned char Chr);
extern char UART_GetChar(unsigned char *Chr);
extern void UART_ClrRxBuf();
extern unsigned int UART_GetCharsInRxBuf();
extern unsigned char UART_IsTxEmpty();


