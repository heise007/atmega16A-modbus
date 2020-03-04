#define __RS485_CTL_PORT	PORTD//������
#define __RS485_CTL_DDR	DDRD//������
#define __RS485_CTL_MSK	0x10//������
#define __RS485_RX_MODE	{__RS485_CTL_PORT &= ~__RS485_CTL_MSK;}
#define __RS485_TX_MODE	{__RS485_CTL_PORT |= __RS485_CTL_MSK;}

/*����ModbusЭ�����裬��������9��*/
#define REG_NUM         40      /*Modbus�ܼĴ����������ݼĴ����б��������ã����мĴ������ɶ�*/
#define WR_START        0       /*��д�Ĵ�����ʼ��ַ*/
#define WR_END          39       /*��д�Ĵ�����ֹ��ַ*/
#define DIR_DELAY       5       /*�յ�����֡����Ӧ����֮֡����ӳ٣���λ��Modbus_Tick���������õ�����*/
#define CMD_BUF_LOCATE //xdata    /*����֡�ݻ�����λ���ڴ��е�λ��         idata:�ڲ�RAM xdata:�ⲿRAM */
#define REG_BUF_LOCATE //idata    /*Modbus�Ĵ���������������ڴ��е�λ��   idata:�ڲ�RAM xdata:�ⲿRAM */
//sbit    DIR=P1^0;               /*485�����л����Ŷ���*/
#define DIR_TX  __RS485_TX_MODE           /*485������Ϊ���[��ƽ����]������ʵ��485оƬ����ĵ�ƽ*/
#define DIR_RX  __RS485_RX_MODE           /*485������Ϊ����[��ƽ����]������ʵ��485оƬ����ĵ�ƽ*/

/*������Ҫ�������к������뿪��*/
#define MB_SET_REG      1       /*�Ƿ����Modbus_SetReg����*/
#define MB_GET_REG      1       /*�Ƿ����Modbus_GetReg����*/
#define MB_SET_ADDR     1       /*�Ƿ����Modbus_SetAddr����*/
#define MB_GET_ADDR     1       /*�Ƿ����Modbus_SetAddr����*/


extern void Modbus_Tick();
extern void Modbus_Process();
extern void Modbus_SetReg(unsigned int Addr,unsigned int Data);
extern unsigned int Modbus_GetReg(unsigned int Addr);
extern void Modbus_SetAddr(unsigned char Addr);
extern unsigned char Modbus_GetAddr();
extern void Send_Addr(void);
extern unsigned int Caculate_CRC16_BUF(unsigned char *DAT,unsigned char Lenth);
//�Ĵ�����ַ˵����
//�豸��ַ�Ĵ���(W/R)��
#define __DEVICE_ADDR	0
//AD�����Ĵ���(R)
#define __CUR_AD0_ADDR		1
#define __CUR_AD1_ADDR		2
#define __CUR_AD2_ADDR		3
#define __CUR_AD3_ADDR		4
#define __CUR_AD4_ADDR		5
#define __CUR_AD5_ADDR		6
#define __CUR_AD6_ADDR		7
#define __CUR_AD7_ADDR		8
//�̵���(W/R)
#define __RELAY_ADDR	10


/*

�Ĵ���˵����


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
��ֲʱ��ע��
�ڶ�ʱ���������modbusʱ��
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
	Modbus_Tick();                      //ÿ2ms����һ��ModbusЭ���е�ʱ������	
}

����485���ͽ���ʹ�ܶ˿�
�������ʼ������
*/