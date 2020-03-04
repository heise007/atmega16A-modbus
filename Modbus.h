#define __RS485_CTL_PORT	PORTD//配置项
#define __RS485_CTL_DDR	DDRD//配置项
#define __RS485_CTL_MSK	0x10//配置项
#define __RS485_RX_MODE	{__RS485_CTL_PORT &= ~__RS485_CTL_MSK;}
#define __RS485_TX_MODE	{__RS485_CTL_PORT |= __RS485_CTL_MSK;}

/*根据Modbus协议所需，设置下面9行*/
#define REG_NUM         40      /*Modbus总寄存器数，根据寄存器列表数量设置，所有寄存器均可读*/
#define WR_START        0       /*可写寄存器起始地址*/
#define WR_END          39       /*可写寄存器终止地址*/
#define DIR_DELAY       5       /*收到请求帧到回应数据帧之间的延迟，单位是Modbus_Tick函数被调用的周期*/
#define CMD_BUF_LOCATE //xdata    /*请求帧暂缓冲区位于内存中的位置         idata:内部RAM xdata:外部RAM */
#define REG_BUF_LOCATE //idata    /*Modbus寄存器缓冲区存放于内存中的位置   idata:内部RAM xdata:外部RAM */
//sbit    DIR=P1^0;               /*485方向切换引脚定义*/
#define DIR_TX  __RS485_TX_MODE           /*485方向设为输出[电平定义]，根据实际485芯片所需的电平*/
#define DIR_RX  __RS485_RX_MODE           /*485方向设为输入[电平定义]，根据实际485芯片所需的电平*/

/*根据需要设置下列函数编译开关*/
#define MB_SET_REG      1       /*是否编译Modbus_SetReg函数*/
#define MB_GET_REG      1       /*是否编译Modbus_GetReg函数*/
#define MB_SET_ADDR     1       /*是否编译Modbus_SetAddr函数*/
#define MB_GET_ADDR     1       /*是否编译Modbus_SetAddr函数*/


extern void Modbus_Tick();
extern void Modbus_Process();
extern void Modbus_SetReg(unsigned int Addr,unsigned int Data);
extern unsigned int Modbus_GetReg(unsigned int Addr);
extern void Modbus_SetAddr(unsigned char Addr);
extern unsigned char Modbus_GetAddr();
extern void Send_Addr(void);
extern unsigned int Caculate_CRC16_BUF(unsigned char *DAT,unsigned char Lenth);
//寄存器地址说明：
//设备地址寄存器(W/R)：
#define __DEVICE_ADDR	0
//AD采样寄存器(R)
#define __CUR_AD0_ADDR		1
#define __CUR_AD1_ADDR		2
#define __CUR_AD2_ADDR		3
#define __CUR_AD3_ADDR		4
#define __CUR_AD4_ADDR		5
#define __CUR_AD5_ADDR		6
#define __CUR_AD6_ADDR		7
#define __CUR_AD7_ADDR		8
//继电器(W/R)
#define __RELAY_ADDR	10


/*

寄存器说明：


aa 06 00 00 00 55 50 2e
55 06 00 00 00 aa 04 61


55 05 00 00 FF 00 81 ee
55 05 00 00 00 00 c0 1e

55 05 00 01 ff 00 d0 2e
55 05 00 01 00 00 91 de

55 05 00 02 ff 00 20 2e
55 05 00 02 00 00 61 de

55 05 00 03 ff 00 71 ee
55 05 00 03 00 00 30 1e

55 05 00 04 ff 00 c0 2f
55 05 00 04 00 00 81 df

55 05 00 05 ff 00 91 ef
55 05 00 05 00 00 d0 1f

55 05 00 06 ff 00 61 ef
55 05 00 06 00 00 20 1f

55 05 00 07 ff 00 30 2f
55 05 00 07 00 00 71 df

*/



/*
移植时的注意
在定时器里面加入modbus时基
void timer1_init(void)
{
	TCCR1B = 0x00; //stop
 TCNT1H = 0x8C; //setup
 TCNT1L = 0xCD;
 OCR1AH = 0x73;
 OCR1AL = 0x33;
 OCR1BH = 0x73;
 OCR1BL = 0x33;
 ICR1H  = 0x73;
 ICR1L  = 0x33;
 TCCR1A = 0x00;
 TCCR1B = 0x01; //start Timer

	TIMSK |= 0x05; //timer interrupt sources
}
ISR(TIMER1_OVF_vect)
{
	//TIMER1 has overflowed
	TCNT1H = 0x8c; //reload counter high value
	TCNT1L = 0xcd; //reload counter low value
	Modbus_Tick();                      //每2ms处理一次Modbus协议中的时基部分	
}

定义485发送接收使能端口
主程序初始化串口
*/