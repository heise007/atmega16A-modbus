/*
               
                     51单片机串口通讯程序库(带有收发缓冲区)

  说明：该程序库包括串口初始化、串口单字节收/发函数，以及串口终端设备接口
        函数。可以作为各种程序的底层驱动使用。
            要使用该库函数，需要将本文件(UART.c)添加进工程，并在需要调用
        串口函数的文件开头处包含"UART.h"。另外注意本程序开头的宏定义要根据
        实际的硬件进行设置，否则波特率不对。 

            程序后半部分是标准终端输入输出函数，对标准终端设备输入的数据流进
        行解析。通过串口与PC机相连;在PC机运行超级终端程序，从而将PC的屏幕和键
        盘映射成单片机的输入/输出终端。这样在单片机程序中可以使用格式化输入/
        输出函数，如printf/scanf等。详细的原理请参考《超级终端人机对话范例》
        程序。终端输入函数带有退格功能，因此有输入缓冲区。请根据需要自行修改
        输入行缓冲区大小(LINE_LENGTH)，该值限定了终端设备每行最大能输入的字符数。
            本程序的优点是串口发送及等待接收过程中，并不直接操作硬件，而是对收发
        缓冲区（环形队列）进行读写。串口的硬件收发在中断内完成。串口的收发函数不
        会阻塞后续代码的执行。缺点是收发缓存占用了较多内存。
            该程序库与精简版（无缓冲区）串口程序的函数接口完全兼容，可以直接替换
        使用（只有UART_GetChar 用法略有不同）。运行效果看似完全一样，但是串口数据
        发送过程非常快，CPU不用等待每一个字节数据发完，接收程序也不阻塞CPU的运行，
        不用等待接收到一个字符才返回数据。在主循环内大量收发数据也不会使得后续程
        序失去响应。
         
        
     
  (C)西安电子科技大学 测控技术与仪器教研中心 编写：谢楷 2009/11/4 
  
*/
//******************************************************************************
//                        
//            
//                                 软件结构
//            
//                  
//            +------------+   +-------------+   +-------------+
//            |  串    口  |   |  RX_BUFF[]  |   |UART_GetChar |
// RXD引脚 -->|  接收中断  |-->|  接收缓冲   |-->|串口读取函数 |-->应用程序  
//            |            |   |  队列(FIFO) |   |             |
//            +------------+   +-------------+   +-------------+
//
//            +------------+   +-------------+   +-------------+
//            |  串    口  |   |  TX_BUFF[]  |   |UART_PutChar |
// TXD引脚 <--|  发送中断  |<--|  发送缓冲   |<--|串口发送函数 |<--应用程序  
//            |            |   |  队列(FIFO) |   |             |
//            +------------+   +-------------+   +-------------+
////
//  本程序硬件连接：
//                89x51/52
//           +-----------------+
//           |              XIN|---
//           |                 |   11.0592MHz/22.1184MHz
//           |             XOUT|---
//           |                 |
//           |        (TXD)P3.1|----------------> //  ----> RXD(2)
//           |                 | 300~115200 bps    TTL/232        
//           |        (RXD)P3.0|<---------------  //  <---- TXD(3)  PC(DB9)
//           |                 |
//           |             GND |----------------------------GND(5)
//
//******************************************************************************
//#include <reg52.h>        /*单片机寄存器头文件*/
#include <avr/io.h>
#include "avr/interrupt.h"
#include "UART.h"         /* Modbus协议栈需要调用串口收发函数作为链路层*/


/* 使用前请根据实际硬件设置下面8行                                 */
#define FOSC (11059200L)  /* 晶体频率,使用前必须设置正确                                     */
#define FCLK (FOSC/12)    /* 机器周期，根据单片机的实际分频系数设置                          */ 
#define BaudTimer   2     /* 波特率定时器 1=Timer1    2=Timer2                               */
#define UART_PS     1     /* 串口优先级0/1，设为1可以打断其他中断，高波特率及系统中具有长时间中断时应设为1  */
#define TXBUF_SIZE  128   /* 发送FIFO的最大容量，必须大于一次连续最多可能的发送字节数        */
#define RXBUF_SIZE  64    /* 接收FIFO的最大容量，必须大于一次连续最多可能的接收字节数        */
#define BUF_LOCATE  //xdata /* 接收/发送缓冲区放在内存的位置 idata:内部RAM xdata:外部RAM       */
                          /* 对于STC单片机有256字节额外的外部RAM，串口缓冲区建议放在xdata中  */
                          /* 对于没有外部RAM的单片机，串口缓冲区只能放在idata中，不能太大    */
#define LINE_LENGTH 8     /* 行缓冲区大小，决定scanf函数每行最多输入的字符数*/

/*根据需要设置下列函数编译开关*/
#define UART_INIT       1    /*是否编译UART_Init函数   */
#define UART_GETCHAR    1    /*是否编译UART_GetChar函数*/
#define UART_PUTCHAR    1    /*是否编译UART_PutChar函数*/
#define UART_GET_CHARS  1    /*是否编译UART_GetCharsInRxBuf函数*/
#define UART_CLR_RXBUF  0    /*是否编译UART_ClrRxBuf函数*/
#define UART_TX_EMPTY   1    /*是否编译UART_IsTxEmpty函数*/
#define HT_GETKEY       0    /*是否编译_getkey函数，该函数被scanf函数调用，如果开启，UART_GetChar也必须开启*/
#define HT_PUTCHAR      0    /*是否编译_putchar函数，该函数被printf函数调用，如果开启，Uart_PutChar也必须开启*/





unsigned char BUF_LOCATE TX_BUFF[TXBUF_SIZE]; /*发送FIFO缓冲区数组*/
unsigned int  UART_OutLen=0;                  /*发送FIFO内待发出的字节数*/
unsigned int  TX_IndexR=0;                    /*发送FIFO内的读指针*/
unsigned int  TX_IndexW=0;                    /*发送FIFO内的写指针*/

unsigned char BUF_LOCATE RX_BUFF[RXBUF_SIZE]; /*接收FIFO缓冲区数组*/
unsigned int  UART_InpLen=0;                  /*接收FIFO内待读取的字节数*/
unsigned int  RX_IndexR=0;                    /*接收FIFO的读指针*/
unsigned int  RX_IndexW=0;                    /*接收FIFO的写指针*/
unsigned char  Tx_FirstFlag=1;
unsigned char  UART_EA;
#define DISABLE_INT  cli();//UART_EA=EA;   EA=0;     /*关闭中断的宏定义*/
#define RESTORE_INT  sei();//EA=UART_EA;             /*恢复中断的宏定义*/

/*****************************************************************
* 名    称：UART_ISR()
* 功    能：串口中断
* 入口参数：无
* 出口参数：无    
* 说    明: 
*****************************************************************/

 ISR(USARTRXC_vect){
	 asm("nop");
	if(UART_InpLen >= RXBUF_SIZE) return; //如果FIFO已满，丢弃刚接收的字符
	UART_InpLen++;                  // 接收字节计数加1
	RX_BUFF[RX_IndexW]=UDR;        // 串口接收数据通过写指针写入FIFO
	if (++RX_IndexW >= RXBUF_SIZE)  // 写指针递增，且判断是否下标越界
	{
		RX_IndexW = 0;                  // 如果越界则写指针归零(循环队列)
	}
 }
 ISR(USARTTXC_vect){
    if(UART_OutLen>0)                   // FIFO内是否有待发送的数据？
	{                                // 有，按照顺序逐个发送。
		UART_OutLen--;                // 待发送数据字节数减1
		UDR=TX_BUFF[TX_IndexR];      // 从尾指针读取一个字节并发送
		if (++TX_IndexR >= TXBUF_SIZE)// 读指针递增，且判断是否下标越界
		{
			TX_IndexR = 0;             // 如果越界则写指针归零(循环队列)
		}
	}
    else Tx_FirstFlag=1;    //所有数据已发完，下一个字节将要直接送入SBUF
}
/****************************************************************************
* 名    称：UART_Init()
* 功    能：初始化串口。设置其工作模式及波特率。
* 入口参数：
*           Baud         波特率    (1200~115200)         
* 出口参数：返回值为1时表示初化成功，为0表示参数出错
* 范    例: UART_Init(9600) //设成9600bps
* 说    明: 使用该函数之前，先要在宏定义中设定晶振频率以及单片机的机器周期
            推荐使用定时器T2作为波特率发生器，能产生更为广泛的波特率。
****************************************************************************/
#if(UART_INIT)
char UART_Init(long int Baud)
{
	UCSRB = 0x00; //disable while setting baud rate
	UCSRA = 0x00;
	UCSRC = 0x06;
	UBRRL = 0x2F; //set baud rate lo
	UBRRH = 0x00; //set baud rate hi
	UCSRB = 0xD8;
}
#endif 
/*****************************************************************
* 名    称：UART_PutChar()
* 功    能：从串口发送1字节数据(向缓冲队列内填入1字节待发送数据)
* 入口参数：Chr：待发送的字节
* 出口参数：返回1表示发送成功，
            返回0表示发送失败。     
* 说    明: 发送过程中，不阻塞CPU运行
*****************************************************************/
#if (UART_PUTCHAR)
char UART_PutChar(unsigned char Chr)
{
  if(UART_OutLen == TXBUF_SIZE) //如果FIFO已满
  { 
    return (0);                 // 不能再发送数据，返回发送失败标志
  }
  DISABLE_INT; 
  if(Tx_FirstFlag==1)            // 如果是第一个字节，即当前没有数据等待被发送
  {
    UDR=Chr;                    // 第一个字节直接送入SBUF内，在发完时会触发中断    
    Tx_FirstFlag=0;
    RESTORE_INT;
    return(1);
  }                             // 若不是第一字节，说明还有数据在排队等待发送
  UART_OutLen++;                // 将该字节压入发送队列，待发送字节数加1
  TX_BUFF[TX_IndexW] = Chr;     // 待发送数据通过写指针写入FIFO
  if (++TX_IndexW >= TXBUF_SIZE)// 写指针递增，且判断是否下标越界
   { 
    TX_IndexW = 0;              // 如果越界则写指针归零(循环队列)       
   }   
  RESTORE_INT;                  // FIFO操作完毕，恢复中断允许
  return (1);                   // 返回发送成功标志    
}
#endif                       
/*****************************************************************
* 名    称：UART_GetChar()
* 功    能：从串口读取1字节数据(从缓冲队列内读取1字节已接收的数据)
* 入口参数：*Chr：读取数据所存放的地址指针
* 出口参数：返回1表示读取成功，返回0表示读取失败。     
* 说    明: 读取过程中，不阻塞CPU运行
*****************************************************************/
#if(UART_GETCHAR)
char UART_GetChar(unsigned char *Chr)
{
  if(UART_InpLen==0) return(0);  	// 如果FIFO内无数据，返回0
  DISABLE_INT;                     	// 涉及FIFO操作时不允许中断，以免数据错乱                               
  UART_InpLen--;                  	// 待读取数据字节数减1  
  *Chr=RX_BUFF[RX_IndexR];        	// 从尾指针读取一个字节作为返回值
  if (++RX_IndexR >= RXBUF_SIZE)   	// 读指针递增，且判断是否下标越界
     {                                   
       RX_IndexR = 0;             	// 如果越界则写指针归零(循环队列)
     }
  RESTORE_INT;                  	// FIFO操作完毕，恢复中断允许
  return (1);                     	// 返回发送成功标志    
}                       
#endif

/*****************************************************************
* 名    称：UART_GetCharsInRxBuf()
* 功    能：获取FIFO内已接收的数据字节数
* 入口参数：无
* 出口参数：待读取的字节数     
*****************************************************************/
#if(UART_GET_CHARS)
unsigned int UART_GetCharsInRxBuf()
{
  unsigned int Temp;
  __DIS_UART_INT;//ES=0;
  Temp=UART_InpLen;
  __EN_UART_INT;//ES=1;
  return (Temp);                 // 返回FIFO内数据的字节数    
}
#endif 

/*****************************************************************
* 名    称：UART_IsTxEmpty()
* 功    能：查询一个数据帧发送是否结束
* 入口参数：无
* 出口参数：1：当前缓冲区内没有字符要发送，SBUF内的数据也发完了
            0：当前串口正在发送数据过程中
* 说    明: 该函数常用于485通讯中检查一个数据帧是否发送完毕，进行收发换向     
*****************************************************************/
#if(UART_TX_EMPTY)
unsigned char UART_IsTxEmpty()
{
  if(Tx_FirstFlag) return(1);
  else             return(0);  
}
#endif                       
/*****************************************************************
* 名    称：UART_ClrRxBuf()
* 功    能：清除接收FIFO区
* 入口参数：无
* 出口参数：无     
*****************************************************************/
#if(UART_CLR_RXBUF)
void UART_ClrRxBuf()
{
  DISABLE_INT;                          // 涉及FIFO操作时不允许中断，以免数据错乱
  UART_InpLen=0;                        // 接收的数据清空
  RX_IndexR=0;    
  RX_IndexW=0;                          // 头尾指针复位
  RESTORE_INT;                          // FIFO操作完毕，恢复中断允许
} 
#endif
/*===================以下是串口终端设备接口函数库==========================*/



/*标准终端设备中，特殊ASCII码定义，请勿修改*/
#define In_BACKSP 0x08          /* ASCII  <--  (退格键)  */
#define In_DELETE 0x7F          /* ASCII <DEL> (DEL 键)  */
#define In_EOL '\r'             /* ASCII <CR>  (回车键)  */
#define In_SKIP '\3'            /* ASCII control-C */
#define In_EOF '\x1A'           /* ASCII control-Z */

#define Out_DELETE "\x8 \x8"    /* VT100 backspace and clear */
#define Out_SKIP "^C\n"         /* ^C and new line */
#define Out_EOF "^Z"            /* ^Z and return EOF */
#include "stdio.h"

/****************************************************************************
* 名    称：putchar()
* 功    能：向标准终端设备发送一字节数据(1个字符)
* 入口参数：ch: 待发送的字符  
* 出口参数：发出的字符
* 说    明: printf函数会调用该函数作为底层输出。这里从串口输出字符到PC机的超
            级终端软件上，printf的结果将打印到超级终端上。若修改该函数，将字
            符以其他方式输出，如显示到LCD上，printf的结果将显示在LCD上。
****************************************************************************/
#if(HT_PUTCHAR)
char putchar(char ch)
{
  if (ch == '\n')        //  '\n'(回车)扩展成 '\n''\r' (回车+换行) 
  {
    UART_PutChar(0x0d) ; //'\r'
  }
  UART_PutChar(ch);      //从串口发出数据  
  return (ch);
}
#endif
/****************************************************************************
* 名    称：put_message()
* 功    能：向标准终端设备发送一个字符串
* 入口参数：*s: 字符串(数组)头指针(数组名)
* 出口参数：无
****************************************************************************/
#if(HT_GETKEY)
#if(LINE_LENGTH>0)
static void put_message(char *s)
{
  while (*s)        //当前字符不为空 (字符串以0x00结尾)
    putchar(*s++);  //输出一个字符，指针指向下一字符
}
#endif
#endif
/****************************************************************************
* 名    称：_getkey()
* 功    能：从标准终端设备接收一字节数据(1个字符)
* 入口参数：无
* 出口参数：收到的字符
* 说    明: scanf函数会调用该函数作为底层输入。这里从PC机键盘借助超级终端软
            件通过串口输入字符到单片机上。scanf函数的输入即源自PC机键盘。若
            修改该函数，将字符以其他方式输入，如单片机IO口，可用按钮向scanf
            函数输入数据。本函数带有缓存，能够处理退格等删除操作。若不需要删
            除操作，可直接调用UART_GetChar()函数。
****************************************************************************/
#if(HT_GETKEY)
char _getkey (void)
{
  char c;
#if(LINE_LENGTH==0)
    while(UART_GetChar(&c)==0);       //等待串口接收一个字符
    return(c);
#endif
#if(LINE_LENGTH>0)
  static char io_buffer[LINE_LENGTH + 2];     /* Where to put chars */
  static int ptr;                             /* Pointer in buffer */
  for (;;)
  {
    if (io_buffer[ptr])                 //如果缓冲区有字符
      return (io_buffer[ptr++]);        //则逐个返回字符
    ptr = 0;                            //直到发送完毕为止，清空缓冲区指针
  while(1)                              //缓冲区没有字符，才会执行到这里，开始等待字符输入
    { while(UART_GetChar(&c)==0);       //等待串口接收一个字符
      if (c == In_EOF && !ptr)          //----EOF键（Ctrl+Z）----
      {                                 //EOF符只能在未输入其他字符时才有效
        put_message(Out_EOF);           //让终端显示EOF符
        return EOF;                     //返回EOF符
      }
      if ((c == In_DELETE)||(c==In_BACKSP)) //----退格或DEL键----
      {
        if (ptr)
        {
          ptr--;                        //从缓冲区删掉一个字符
          put_message(Out_DELETE);      //让终端显示也删掉一个字符
        }
      }
      else if (c == In_SKIP)            //-----取消键 Ctrl + C----
      {
        put_message(Out_SKIP);          //让终端显示放弃本行跳到下一行
        ptr = LINE_LENGTH + 1;          /* 这里永远是0(结束符) */
        break;
      }
      else if (c == In_EOL)             //--------遇到回车键------
      {
        putchar(io_buffer[ptr++] = '\n');   //让终端显示换行
        io_buffer[ptr] = 0;             //末尾增添字符串结束字符NULL
        ptr = 0;                        //指针清空
        break;                          //跳出后开始返回数据
      }
      else if (ptr < LINE_LENGTH)       //----正常ASCII码字符----
      {
        if (c >= ' ')                   //删除0x20以下的其他ASCII码
        {
          putchar(io_buffer[ptr++] = c);//存入缓冲区
        }
      }
      else                  //--------冲区已满--------
      {
        putchar('\7');      //向终端发送鸣响符，PC会响一声，提示已满
      }
    }
  }
#endif
}
#endif