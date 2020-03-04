/*
               
                         Modbus-RTUЭ��[�ӻ�]ͨѶ�����

  ˵����Modbus-RTU��ʹ����Ϊ�㷺�Ĺ�ҵ�ֳ�����Э��֮һ���Դ�1970�걻Modicon��˾�������
        ����40�����Ӧ�þ��ò�˥��Modbus-RTUЭ���̾��ɣ���7��ģ����ֻռ�м򵥵�2�㣨��
        ·���Э��㣩�������Ը����������Ϊͨ�Ž���(RS-485�����ˡ����ߡ����ߡ�TCP/IP)��
        Ҳ�ɺ͸���Ӧ�ò�ӿڡ�������ʹ��RS-485������Ϊ����㣬���ṩ�˻����Ĵ�����д����
        ��ΪӦ�ò�ӿڡ�
            Modbus-RTUЭ����һ�ּĴ�������ͨѶЭ�顣�������е�����������ɾ�̬�ļĴ�����
        ���е����ݶ�д����������˼Ĵ����Ķ�д������ͨ��˫��Լ��0�żĴ�������"�¶�ֵ"��
        1�żĴ������С��¶����ޡ�����Ƭ�����º��¶Ȳ����������0�żĴ���������������ʱ
        ͨ�������ȡ0�żĴ������Ͷ�ȡ���¶�ֵ����Ƭ����ȡ�Ĵ���1��������Ϊ��������ֵ������
        Ҳ������ʱͨ��д�������1�żĴ�����ʵ�ָ����¶�����ֵ�Ĳ�����
           
           ����������˼Ĵ�����������ΪЭ����Ӧ�ò�֮��Ľӿڣ��üĴ������������ʱ��
        ModbusЭ��ջ������ʣ�Ҳ������ʱ��Ӧ�ó�����ʡ�Զ������ͨ��Modbusָ���޸��˼Ĵ�����
        Ӧ�ó�����Է��ʼĴ�����������֪��Ӧ�ó�������ݣ�����ֲ��������Ҳ����д��Ĵ�
        �����������Ӷ����Ա�Զ��������ȡ���������ܹ�����ModbusЭ����3�����
            03�����������ȡ����Ĵ���
            06�����д�����Ĵ���
            16���������д����Ĵ���

  ׼��������
   1.   �������ԡ����������Ĵ���ģ�黯����⡷�еĴ����շ�������Ϊ�ײ�ͨ����·�����Ҫ
        ʹ�ñ������⣬��Ҫ��UART.c��UART.h �Լ����ļ�(Modbus.c)��Modbus.h��4���ļ���ӽ����̣�
        ��������Ҫ���üĴ�����д�������ļ���ͷ����Modbus.h����ʹ�ñ�������֮ǰ���������ô���
        ������UART.c����֤����4��������������
            1.UART_GetChar
            2.UART_PutChar
            3.UART_GetCharsInRxBuf
            4.UART_IsTxEmpty  
   2.   ���ô����շ���������С��ԭ���ǣ�
            1.TXBUF_SIZE�Ĵ�С�������03ָ������ܷ��͵��ֽ���(������������ȡ�ļĴ�����*2+5)
            2.RXBUF_SIZE�Ĵ�С�������16ָ������ܽ��յ��ֽ���(����������д��ļĴ�����*2+9)  

   3.   �ڱ��ļ���ͷ������ModbusЭ����ز���

   4.   �������У�����һ����ʱ�жϣ����ж��ڵ���Modbus_Tick()��������ʱ����1-10ms����

   5.   Modbus_Process()����Ҳ��Ҫ�������Եص��ã������ڶ�ʱ�ж��ڵ��ã����Ǹú���ִ��ʱ��ϳ�
        (Ҫ����Э�顢��������)����ִ��ʱ��������Ҫ��д���ֽ��������ȣ�����ж��ڵ��ã���ʹ�ж�
        ִ��ʱ��䳤���������ڶ�ʱ���ڣ����������ж���Ӧ�ӳ٣���ʹ����ѭ��ִ�б������ú���Ҳ��
        ������ѭ���ڵ��ã���ʱ��Ӧ����֡���ᱻ�ӳ����1����ѭ������ʱ�䡣
        

ModbusЭ���飺ModbusЭ���У����ݴ洢���������"�Ĵ���"Ϊ��λ��ÿ���Ĵ�������2�ֽ����ݣ����ֽ�
       ���ȡ�ͨѶ֡�У���CRCУ����ֽ��������⣬����˫�ֽ����ݾ����ֽ����ȡ�
       ModbusЭ���ÿ��ͨѶ�����������������������������ʽ����ͨѶ����֡���������֡�ṹ���£�

   03������(���Ĵ���):
       �����ʽ:�豸��ַ[1�ֽ�]+������[03H]+��ʼ�Ĵ���[2�ֽ�]+��ȡ�Ĵ�������N[2�ֽ�]+CRC[2�ֽ�]
       ���ظ�ʽ:�豸��ַ[1�ֽ�]+������[03H]+�ֽ���[1�ֽ�]+����N���Ĵ���ֵ[2N�ֽ�]+CRC[2�ֽ�]

   06������(д�����Ĵ���):
       �����ʽ: �豸��ַ[1�ֽ�]+������[06H]+д��Ĵ�����ַ[2�ֽ�]+д������[2�ֽ�]+CRC[2�ֽ�]
       ���ظ�ʽ��ԭ���ظ�һ����������

   16��(10H��)����(����д����Ĵ���)
       �����ʽ: �豸��ַ[1�ֽ�]+������[10H]+д����ʼ�Ĵ�����ַ[2�ֽ�]+д��Ĵ�������N[2�ֽ�]
                 +д�������ֽ�����(2N)[1�ֽ�]+����д��ļĴ�������[2N�ֽ�]+CRC[2�ֽ�]
       ���ظ�ʽ���豸��ַ[1�ֽ�]+������[10H]+д����ʼ�Ĵ�����ַ[2�ֽ�]+д��Ĵ�������N[2�ֽ�]+CRC[2B]

ͨѶ������
����->�ӻ�
�ֽ�:   �ֽ�1	�ֽ�2	�ֽ�3	�ֽ�4   �ֽ�5	�ֽ�6	    �ֽ�7	�ֽ�8
����:   0x01	0x03	0x00    0x00    0x00    0x04	     0x44	0x09
����:   ��ַ    ��	  �ӼĴ���0000��ʼ  ����ȡ0004���Ĵ���	   CRCУ��

1�Ŵӻ���Ӧ���������ݣ�
�ֽ�:	�ֽ�1	�ֽ�2	�ֽ�3	   �ֽ�4	�ֽ�5	�ֽ�6	�ֽ�7	�ֽ�8	�ֽ�9   �ֽ�10	�ֽ�11	�ֽ�13	�ֽ�14
����:	0x01	0x03	0x08	   0x13     0x88	0x00    0x04	0x00     0x14   0x6D    0x41	 XX      XX
����	��ַ   ���ɹ�  ����8�ֽ�    �Ĵ���0=0x1348  �Ĵ���1=0x0400  �Ĵ���2=0x0014   �Ĵ���2=0x6D41    CRCУ��
                        	

                             
     
  (C)�������ӿƼ���ѧ ��ؼ����������������� ��д��л�� 2009/11/11 
  
*/                        
//            
//                                               ����ṹ
//|           |                                                  |                           |                            |
//|<--�����->|<------------------��·��------------------------>|<----------Э���--------->|<----------Ӧ�ò�---------->|
//|           |                                                  |                           |                            |
//
//            +------------+   +-------------+   +-------------+   +---------+   +---------+   +--------------+
//            |  ��    ��  |   |  RX_BUFF[]  |   |UART_GetChar |   |         |   |         |   |Modbus_GetReg |
// RXD���� -->|  �����ж�  |-->|  ���ջ���   |-->|���ڶ�ȡ���� |-->|Modbus_  |-->|         |-->|�Ĵ�����ȡ����|-->Ӧ�ó���  
//            |            |   |  ����(FIFO) |   |             |   |Process()|   |         |   |              |
//            +------------+   +-------------+   +-------------+   |  ����   |   |REG_BUF[]|   +--------------+
//                                                                 |         |   | �Ĵ���  |                  
//            +------------+   +-------------+   +-------------+   |         |   | ������  |   +--------------+
//            |  ��    ��  |   |  TX_BUFF[]  |   |UART_PutChar |   |(Modbus  |   |         |   |Modbus_SetReg |
// TXD���� <--|  �����ж�  |<--|  ���ͻ���   |<--|���ڷ��ͺ��� |<--| Э��ջ) |<--|         |<--|�Ĵ������ú���|<--Ӧ�ó���  
//            |            |   |  ����(FIFO) |   |             |   |         |   |         |   |              |
//            +------------+   +-------------+   +-------------+   +---+-----+   +---------+   +--------------+
//                                                                     |    
//                                                              +------+-------+
//                                                              |��ʱ�ж��ڵ���|
/// DIR���� <---------------------------------------------------|Modbus_Tick() |
//                                                              |�ṩ��ʱ����  |
//                                                              +--------------+
//

//������Ӳ�����ӣ�
//                STC89C52                                      
//           +-----------------+                                
//           |              XIN|---
//           |                 |   11.0592MHz/22.1184MHz
//           |             XOUT|---
//           |                 |           +---------+
//           |        (TXD)P3.1|---------->| DI      |<--------->A(D+)
//           |                 | 300~115200|         |         
//           |        (RXD)P3.0|<----------| DO      |<--------->B(D-)  
//           |                 |           |  MAX485 |
//           |        (DIR)P1.0|---------->| DIR     |
//           |                 |           |         |
//           |                 |           |         |
//           |                 |           +---------+
//           |                 |



//******************************************************************************
#include <avr/io.h>
#include <stdio.h>
#include "UART.h"         /* ModbusЭ��ջ��Ҫ���ô����շ�������Ϊ��·��*/
#include "Modbus.h"
//#include "bsp.h"
//#include "config.h"

/*���������Ҫ��������4�����ں�������UART.c�У����ǵı��뿪�ر��뱻�򿪡�
UART_GetChar��UART_PutChar��UART_GetCharsInRxBuf��UART_IsTxEmpty      

TXBUF_SIZE�Ĵ�С�������03ָ������ܷ��͵��ֽ���
RXBUF_SIZE�Ĵ�С�������16ָ������ܽ��յ��ֽ���                     */




/*-------------------�����������������޸�---------------------*/

unsigned char SlaveAddr=1;        /*ͨѶ��ַ������Ĭ��ͨѶ��ַ=1 */
unsigned int  REG_BUF_LOCATE  REG_BUF[REG_NUM];    /*Modbus�Ĵ���������*/
unsigned int  CRCREG=0xffff;     /*CRC���������ݴ����*/
unsigned char RxFrameFlag=0;               /*�յ�����֡�ı�־*/


/*****************************************************************
 
                   Layer1.  ����㣺TTL/485оƬ

*****************************************************************/

/*****************************************************************
 
                   Layer2.  ��·�㣺UART.c���ṩ�Ĵ����շ�����

*****************************************************************/

/*****************************************************************
 
                   Layer3.  ModbusЭ��ջ�㣺

*****************************************************************/
/****************************************************************************
* ��    �ƣ�Caculate_CRC16_Byte()
* ��    �ܣ�����CRCУ�飬�Ե�����ʽ����1�ֽڵ�CRCУ��ֵ
* ��ڲ�����DAT:��������CRC��1�ֽ�����
            CRC:ǰһ�εĵ�CRC�������
* ���ڲ�������ǰ��CRCֵ��
* ˵    ��: �ú����ʺϴ���ÿ����һ�ֽ�ʱ��������CRCУ�飬�Խ�ʡʱ��
****************************************************************************/
unsigned int Caculate_CRC16_Byte(unsigned char DAT,unsigned int CRC)
{  unsigned char j;
         CRC=CRC^(unsigned int)DAT;
        for(j=0;j<8;j++)
          { 
             if(CRC&0x01)
             {
               CRC=CRC>>1;
               CRC=CRC^0xA001;       //һ�������CRC�����㷨
             }
             else
             {
               CRC=CRC>>1;           
             } 
          } 
   return(CRC);               
}
/****************************************************************************
* ��    �ƣ�Caculate_CRC16_BUF()
* ��    �ܣ�����һ�������CRCУ��
* ��ڲ�����*DAT:������ݵ�������
            Lenth:��������Ҫ����CRC���ֽ���
* ���ڲ�����CRC��������
* ˵    ��: �ú����ʺ϶Խ��ջ����������ͻ�����������ݼ���CRCУ��ֵ
****************************************************************************/
unsigned int Caculate_CRC16_BUF(unsigned char *DAT,unsigned char Lenth)
{
   unsigned int CRC=0xffff;                 //CRC��ʼֵ=ȫ1
   unsigned char i;
   for(i=0;i<Lenth;i++)                     //��������Lenth��������CRCУ��
      {
        CRC=Caculate_CRC16_Byte(DAT[i],CRC);//���ε�������
      }
  return(CRC);                              //���ؽ��
}    
/****************************************************************************
* ��    �ƣ�SendByte_CRC()
* ��    �ܣ�����һ���ֽڣ�ͬʱ��������CRCУ��ֵ
* ��ڲ�����DAT:�����͵�1�ֽ�����
* ���ڲ������ޡ�
* ˵    ��: �ú������ô��ڷ���1�ֽں���
****************************************************************************/
void SendByte_CRC(unsigned char DAT)
 {
   CRCREG=Caculate_CRC16_Byte(DAT,CRCREG);//�ȵ�������CRC
   UART_PutChar(DAT);                     //�ٴӴ��ڷ���
 }                                        //�޷���ֵ����������CRCREGȫ�ֱ�����

/****************************************************************************
* ��    �ƣ�SendWord_CRC()
* ��    �ܣ�����һ��16λ���ݣ�ͬʱ��������CRCУ��ֵ
* ��ڲ�����WORD:�����͵�2�ֽ���������
* ���ڲ������ޡ�
****************************************************************************/
void SendWord_CRC(unsigned int WORD)
{
   SendByte_CRC(WORD/256);
   SendByte_CRC(WORD%256); 
}
/****************************************************************************
* ��    �ƣ�ModBus_Command03()
* ��    �ܣ�����������[03]
* ��ڲ�����StartAddr:��ȡ����ʼ�Ĵ���
            RegNum   :��ȡ�ļĴ�������
* ���ڲ������ޡ�
* ˵    ��: ModbusЭ���У�������[03]:������ȡ����Ĵ���
            �����ʽ: �豸��ַ[1B]+������[1B]+��ʼ�Ĵ���[2B]+��ȡ�Ĵ�������N[2B]+CRC[2B]
            ���ظ�ʽ���豸��ַ[1B]+������[1B]+�ֽ���[1B]+DATA[2NB]+CRC[2B]
****************************************************************************/
void ModBus_Command03(StartAddr,RegNum)
{
  unsigned int EndAddr=StartAddr+RegNum;      //�������һ������ȡ�ļĴ�����ַ
  unsigned int Addr;

  SendByte_CRC(SlaveAddr);                    //���ص�ַ
  SendByte_CRC(0x03);                         //���ع����� 03            
  SendByte_CRC(RegNum*2);                     //�����������ֽ���(2*�Ĵ�����)
  for(Addr=StartAddr;Addr<EndAddr;Addr++)     //��������Ҫ�����η�������
    {
      SendWord_CRC(REG_BUF[Addr]);            //RegNum���Ĵ��������η���
    }
  UART_PutChar(CRCREG%256);                   //CRC,��λ����
  UART_PutChar(CRCREG/256); 
}
/****************************************************************************
* ��    �ƣ�ModBus_Command06()
* ��    �ܣ�����������[06]
* ��ڲ�����WR_Addr:д��ļĴ�����ַ
            WR_Data:д�������
            *Ptr:ָ����������֡��ָ��
* ���ڲ������ޡ�
* ˵    ��: ModbusЭ���У�������[06]:д�����Ĵ���
            �����ʽ: �豸��ַ[1B]+������[1B]+�Ĵ�����ַ[2B]+д������[2B]+CRC[2B]
            ���ظ�ʽ��ԭ���ظ�һ����������
****************************************************************************/
void ModBus_Command06(unsigned int WR_Addr,unsigned int WR_Data,unsigned char CMD_BUF_LOCATE *Ptr)
{           
    unsigned char i;
    REG_BUF[WR_Addr]=WR_Data;                //�����ݴ�����Ӧ�ļĴ�����
    for(i=0;i<8;i++)  UART_PutChar(Ptr[i]);  //ԭ���ظ�һ������֡��Ϊ�ظ�
}
/****************************************************************************
* ��    �ƣ�ModBus_Command16()
* ��    �ܣ�����������[16]
* ��ڲ�����StartAddr:д�����ʼ�Ĵ���
            RegNum   :д��ļĴ�������
            *Ptr:ָ����������֡��ָ��
* ���ڲ������ޡ�
* ˵    ��: ModbusЭ���У�������[16]:����д����Ĵ���
            �����ʽ: �豸��ַ[1B]+������[1B]+д����ʼ�Ĵ�����ַ[2B]++д��Ĵ�������N[2B]+�ֽ���(=2N)[1B]+д������[2NB]+CRC[2B]
            ���ظ�ʽ���豸��ַ[1B]+������[1B]+д����ʼ�Ĵ�����ַ[2B]++д��Ĵ�������N[2B]+CRC[2B]
****************************************************************************/
void ModBus_Command16(unsigned char StartAddr,unsigned char RegNum,unsigned char CMD_BUF_LOCATE *Ptr)
{
  unsigned int EndAddr=StartAddr+RegNum;   //�������һ����д��ļĴ�����ַ
  unsigned int Addr,Data,i=0;
  for(Addr=StartAddr;Addr<EndAddr;Addr++)  //����ʼ��ַ��ʼ������д��
    {
       Data=256*Ptr[2*i+7]+Ptr[2*i+8];     //������֡�����λ�ȡ��д�������
       REG_BUF[Addr]=Data;                 //д����Ӧ�ļĴ���
       i++;                                //��һ��
    } 
  SendByte_CRC(SlaveAddr);                 //���ص�ַ
  SendByte_CRC(0x10);                      //���ع����� 16            
  SendWord_CRC(StartAddr);                 //����д�����ʼ�Ĵ���
  SendWord_CRC(RegNum);                    //����д��ļĴ�����
  UART_PutChar(CRCREG%256);
  UART_PutChar(CRCREG/256);                //CRC
}
void  ModBus_Command05_WriteRelay(unsigned int StartAddr, unsigned int WR_Data){
	/*
	switch (StartAddr){
		case 0:
			if (WR_Data==0x0000){__RELAY_0_DIS;} else if(WR_Data==0xFF00){__RELAY_0_EN;}break;
		case 1:
			if (WR_Data==0x0000){__RELAY_1_DIS;} else if(WR_Data==0xFF00){__RELAY_1_EN;}break;
		case 2:
			if (WR_Data==0x0000){__RELAY_2_DIS;} else if(WR_Data==0xFF00){__RELAY_2_EN;}break;
		case 3:
			if (WR_Data==0x0000){__RELAY_3_DIS;} else if(WR_Data==0xFF00){__RELAY_3_EN;}break;
		case 4:
			if (WR_Data==0x0000){__RELAY_4_DIS;} else if(WR_Data==0xFF00){__RELAY_4_EN;}break;
		case 5:
			if (WR_Data==0x0000){__RELAY_5_DIS;} else if(WR_Data==0xFF00){__RELAY_5_EN;}break;
		case 6:
			if (WR_Data==0x0000){__RELAY_6_DIS;} else if(WR_Data==0xFF00){__RELAY_6_EN;}break;
		case 7:
			if (WR_Data==0x0000){__RELAY_7_DIS;} else if(WR_Data==0xFF00){__RELAY_7_EN;}break;
		default:break;
	}
	if (WR_Data==0x0000){
		RelayStatus[StartAddr]=0;
	}else if(WR_Data==0xFF00){
		RelayStatus[StartAddr]=1;
	}
	
	SendByte_CRC(SlaveAddr);                 //���ص�ַ
	SendByte_CRC(0x05);                      //���ع����� 16            
	SendWord_CRC(StartAddr);                 //����д�����ʼ�Ĵ���
	SendWord_CRC(WR_Data);                    //����д�����
	UART_PutChar(CRCREG%256);
	UART_PutChar(CRCREG/256);                //CRC
	*/
}
ModBus_Command01_ReadRelay(unsigned int StartAddr,unsigned int RegNum){
	/*
	unsigned char reg_group=0,status_byte_temp=0;//
	unsigned char temp=0;
	unsigned char status_group_temp[__RELAY_GROUP];
	unsigned char count=0;//��¼�Ѷ�ȡ��Ȧ״̬������
	if (RegNum%8==0){
		reg_group = RegNum/8;
	}else{
		reg_group = RegNum/8+1;
	}
	
	SendByte_CRC(SlaveAddr);                 //���ص�ַ
	SendByte_CRC(0x01);                      //���ع����� 16            
	SendByte_CRC(reg_group);                //���ؼ̵���״̬���ֽ���
	for (unsigned char i=0;i<reg_group;i++)
	{
		status_group_temp[i]=0;
		status_byte_temp=0;
		for (unsigned char j=0;j<8;j++){
			if (count>RegNum){//������Ȧ��״̬��������
				count=0;
				break;
			}
			if (RelayStatus[StartAddr+count]==1){
				status_byte_temp |= (0x01<<j);
			}else{
				status_byte_temp &= ~(0x01<<j);
			}			
			count++;
			status_group_temp[i]=status_byte_temp;
			//status_byte_temp<<=1;
			
		}
		SendByte_CRC(status_group_temp[i]);
	}
	
		
	

	UART_PutChar(CRCREG%256);
	UART_PutChar(CRCREG/256);                //CRC
	*/
}
/****************************************************************************
* ��    �ƣ�ModBus_CommandErr()
* ��    �ܣ����ظ��ִ������
* ��ڲ�����Command:�ں�������ʱ�����˴���
            ExceptionCode:�������
* ���ڲ������ޡ�
* ˵    ��: ModbusЭ���У��������ʽ��
            �豸��ַ[1B]+(0x80+������)[1B]+�������[1B]+CRC[2B]
****************************************************************************/
void ModBus_CommandErr(unsigned char Command,unsigned char ExceptionCode)
{
  SendByte_CRC(SlaveAddr); 
  SendByte_CRC(0x80+Command);    //�������
  SendByte_CRC(ExceptionCode);   //���ش������
  UART_PutChar(CRCREG%256);
  UART_PutChar(CRCREG/256); 
}
void My_ModBus_Command01_ReadRelay(unsigned char offset,unsigned char reg_num){
	/*
	unsigned char relay_status = *(volatile unsigned char *)Reg_Data[offset].addr_l;
	SendByte_CRC(SlaveAddr);                 //���ص�ַ
	SendByte_CRC(0x01);                      //���ع����� 01
	SendByte_CRC(1);                //���ؼ̵���״̬���ֽ���
	SendByte_CRC(relay_status);
	UART_PutChar(CRCREG%256);
	UART_PutChar(CRCREG/256);                //CRC
	*/
}
void My_ModBus_Command05_WriteRelay(unsigned char offset, unsigned int WR_Data){
	/*
	if (WR_Data==0xff00){
		*(volatile unsigned char *)Reg_Data[offset].addr_l |= Reg_Data[offset].msk_l;
	} else if (WR_Data==0x0000){
		*(volatile unsigned char *)Reg_Data[offset].addr_l &= ~Reg_Data[offset].msk_l;
	}
	
	SendByte_CRC(SlaveAddr);                 //���ص�ַ
	SendByte_CRC(0x05);                      //���ع����� 16
	SendWord_CRC(offset+__RELAY_REG_SATART_ADDR);                 //����д�����ʼ�Ĵ���
	SendWord_CRC(WR_Data);                    //����д�����
	UART_PutChar(CRCREG%256);
	UART_PutChar(CRCREG/256);                //CRC
*/
}
void My_ModBus_Command15_WriteRelay(unsigned char offset , unsigned int reg_num,unsigned char CMD_BUF_LOCATE *Ptr ){
	/*
	unsigned int i=0;
	unsigned char ii=0,j=0;
	unsigned char relay_status_temp=0;
	unsigned char reg_group=0;
	reg_group = reg_num/8;
	if (reg_num%8){
		reg_group+=1;
	}
	for (i=0;i<reg_group;i++){//ѭ���Ĵ�������
		asm("nop");
		if((reg_num-i)<8){//ѭ�������һ���ֽ���ʣ��Ĵ�����������8����
			ii=reg_num-i;
		}else{
			ii=8;
		}
		for (j=0;j<ii;j++){
			if ( *(Ptr+7+i) & (0x01<<j))
			{
				*(volatile unsigned char *)Reg_Data[offset+j].addr_l |= Reg_Data[offset+j].msk_l;
			}else{
				*(volatile unsigned char *)Reg_Data[offset+j].addr_l &= ~Reg_Data[offset+j].msk_l;
			}
		}
	}
	SendByte_CRC(SlaveAddr);                 //���ص�ַ
	SendByte_CRC(0x0f);                      //���ع����� 16
	SendWord_CRC(offset+__RELAY_REG_SATART_ADDR);                 //����д�����ʼ�Ĵ���
	SendWord_CRC(reg_num);                    //����д�����
	UART_PutChar(CRCREG%256);
	UART_PutChar(CRCREG/256);                //CRC
	*/
}
void My_ModBus_Command06(unsigned int WR_Addr,unsigned int WR_Data,unsigned char CMD_BUF_LOCATE *Ptr){
	/*
    unsigned char i,data=0;
	unsigned int eeprom_addr=0;

	eeprom_addr=Reg_Data[__HOLDING_REG_OFFSET + (WR_Addr-__HOLDING_REG_START_ADDR)].addr_l;

    for(i=0;i<8;i++)  {
		data=*(Ptr+i);
		UART_PutChar(data);  //ԭ���ظ�һ������֡��Ϊ�ظ�
	}

	if (eeprom_addr==2){
		Modbus_SetAddr((unsigned char)(WR_Data&0x00ff));
	}

	eeprom_write_byte(eeprom_addr,(unsigned char)(WR_Data&0x00ff));
	eeprom_write_byte(eeprom_addr+1,(unsigned char)((WR_Data&0xff00)>>8));

	_delay_ms(200);
	//���¶�ȡeeprom��ȡ���ڴ�
	*/
}
void My_ModBus_Command03(unsigned int StartAddr,unsigned char RegNum){
	/*
	unsigned int EndAddr=StartAddr+RegNum-__HOLDING_REG_START_ADDR;      //�������һ������ȡ�ļĴ�����ַ
	unsigned int start_addr=StartAddr-__HOLDING_REG_START_ADDR; 
	unsigned int Addr;
	unsigned int i=0,data=0;
	unsigned int data_temp[200];
	unsigned char reg_num=0;
	unsigned int data_len=0;
	
	unsigned int tt=0;
	reg_num=EndAddr-start_addr;
	for (i=0;i<reg_num;i++){//ѭ���Ĵ�������
		tt=__HOLDING_REG_OFFSET +start_addr+i;
		//ȥeeprom��ȡ
		if ((Reg_Data[__HOLDING_REG_OFFSET +start_addr+i].addr_l==0)&&(Reg_Data[__HOLDING_REG_OFFSET +start_addr+i].addr_h==0)){

			break;
		}
		data = eeprom_read_byte(Reg_Data[__HOLDING_REG_OFFSET +start_addr+i].addr_l+1);
		data <<= 8;
		data |= eeprom_read_byte(Reg_Data[__HOLDING_REG_OFFSET +start_addr+i].addr_l);

		data_temp[i]=data;
	}
	if (i<reg_num){
		 ModBus_CommandErr(0x10,3);    //����д�����ش�����03:�Ĵ�����ַ����Χ,�Ƿ�����ֵ
		 return;
	}
	SendByte_CRC(SlaveAddr);                    //���ص�ַ
	SendByte_CRC(0x03);                         //���ع����� 03
	SendByte_CRC(i*2);                     //�����������ֽ���(2*�Ĵ�����)
	data_len=i;
	for (i=0;i<data_len;i++){
	 	SendWord_CRC(data_temp[i]);
		
	}

	UART_PutChar(CRCREG%256);                   //CRC,��λ����
	UART_PutChar(CRCREG/256);	
	*/
}
 void My_ModBus_Command16(unsigned int start_addr,unsigned int reg_num,unsigned char CMD_BUF_LOCATE *Ptr){
	 /*
	unsigned int start_addr_temp=start_addr-__HOLDING_REG_START_ADDR;
	unsigned int endaddr_temp=start_addr+reg_num-__HOLDING_REG_START_ADDR;      //�������һ������ȡ�ļĴ�����ַ
	unsigned int data=0;
	unsigned char i=0;
	unsigned int data_temp[200];
	unsigned int tt=0;
	//*Ptr[7]��ʼ���������Ĵ�д������
	for (i=0;i<reg_num;i++){//ѭ���Ĵ�������,����Χ����û��
		tt = __HOLDING_REG_OFFSET +start_addr_temp+i;
		if ((Reg_Data[__HOLDING_REG_OFFSET +start_addr_temp+i].addr_l==0)&&(Reg_Data[__HOLDING_REG_OFFSET +start_addr_temp+i].addr_h==0)){			
			ModBus_CommandErr(0x10,3);    //����д�����ش�����03:�Ĵ�����ַ����Χ
			return;
		}
	}
	//��Χû����
	for (i=0;i<reg_num;i++){
		tt=Ptr[7+i*2];
		tt=Ptr[7+i*2+1];
		eeprom_write_byte(Reg_Data[__HOLDING_REG_OFFSET +start_addr_temp+i].addr_l+1,Ptr[7+i*2]);
		eeprom_write_byte(Reg_Data[__HOLDING_REG_OFFSET +start_addr_temp+i].addr_l,Ptr[7+i*2+1]);
	}
	SendByte_CRC(SlaveAddr);                    //���ص�ַ
	SendByte_CRC(0x10);                         //���ع����� 03
	SendWord_CRC(reg_num);
	SendByte_CRC(i*2);                     //�����������ֽ���(2*�Ĵ�����)

	UART_PutChar(CRCREG%256);                   //CRC,��λ����
	UART_PutChar(CRCREG/256);

	if (Modbus_GetAddr()!=eeprom_read_byte(2)){
		Modbus_SetAddr(eeprom_read_byte(2));
	}
*/
 }
/****************************************************************************
* ��    �ƣ�Modbus_Process()
* ��    �ܣ�ModbusЭ������봦��
* ��ڲ�������
* ���ڲ������ޡ�
* ˵    ��: ���������Եص��øú����������ڶ�ʱ�ж��ڵ��ã����Ǹú�����ִ��
            ʱ��ϳ������ж��ڵ��ÿ��ܻ���������ж�ʵʱ���½���Ҳ���Է���
            ��ѭ����ִ��,��ʱ���ݷ������ᱻ�ӳ�1����ѭ������ʱ�䡣
****************************************************************************/
void Modbus_Process()
{ 
  unsigned char CMD_BUF_LOCATE  CMD_BUF[REG_NUM*2+10]; //Modbus����֡����
  unsigned int StartAddr,EndAddr,RegNum,WR_Addr,WR_Data;
  unsigned char Modbus_FrameLenth;          //����֡���ȱ���
  unsigned char i,Chr,offset;
  if(RxFrameFlag!=1)  return;               //û�յ�����֡��ֱ�ӷ���
  __DIS_UART_INT;//ES=0;                                     //���������б��ڼ�رմ����շ��������ڼ��յ����ַ����»���
  Modbus_FrameLenth=UART_GetCharsInRxBuf(); //����֡����=��ǰ���ջ���������
  for(i=0;i<Modbus_FrameLenth;i++)          
      {
        UART_GetChar(&Chr);                 //����ȡ�����ڻ����������е��ַ���
        CMD_BUF[i]=Chr;                     //���������󻺳�����
      }
  asm("nop");
  if(CMD_BUF[0]!=SlaveAddr)  goto END;      //��ַ�������Ǳ������󲻴�������
  if(Caculate_CRC16_BUF(CMD_BUF,Modbus_FrameLenth)!=0) goto END;//У�����,����������   
  CRCREG=0xffff;
  DIR_TX;                                   //485��Ϊ��������濪ʼ��������
  switch (CMD_BUF[1])                       //�ж��յ�����ʲô����
        {
		/*
          case 0x03:    //������03 [����������Ĵ���]
             {
               StartAddr=CMD_BUF[2]*256+CMD_BUF[3];                     //��ȡ�Ĵ�����ʼ��ַ
               RegNum=(CMD_BUF[4]*256+CMD_BUF[5]);                      //��ȡ�Ĵ�������
               EndAddr  =StartAddr+RegNum;                              //�������һ���Ĵ����ĵ�ַ
               if     (Modbus_FrameLenth!=8) ModBus_CommandErr(0x03,1); //���ݰ���ȱ�����ش�����01����Чָ��
               else if(EndAddr>REG_NUM) ModBus_CommandErr(0x03,2);      //�����Ĵ�����ַ��Χ�����ش�����02:��Ч��ַ
               else   ModBus_Command03(StartAddr,RegNum);               //��ȷ��03ָ��������ݰ���
               break;
             }
          case 0x06:   //������06 [д�����Ĵ���]
             {                                                          
               WR_Addr=CMD_BUF[2]*256+CMD_BUF[3];                        //д��Ĵ����ĵ�ַ
               WR_Data=(CMD_BUF[4]*256+CMD_BUF[5]);                      //д�������
               if      (Modbus_FrameLenth!=8) ModBus_CommandErr(0x06,1); //���ݰ���ȱ��������01����Чָ��
               else if((WR_Addr<WR_START)||(WR_Addr>WR_END)) ModBus_CommandErr(0x06,2);//��ַ�����������02:��Ч��ַ
               else ModBus_Command06(WR_Addr,WR_Data,CMD_BUF);                   //��ȷָ��������ݰ���
               break;                      
             }
          case 0x10:   //������16 (0x10) [����д����Ĵ���]
            {
               StartAddr=CMD_BUF[2]*256+CMD_BUF[3];                      //����д��Ĵ�����ʼ��ַ
               RegNum=(CMD_BUF[4]*256+CMD_BUF[5]);                       //����д��Ĵ�������
               EndAddr  =StartAddr+RegNum;                               //�������һ���Ĵ����ĵ�ַ
               if(StartAddr<WR_START)      ModBus_CommandErr(0x10,2);    //����д�����ش�����02:��Ч��ַ
               else if(EndAddr>WR_END)     ModBus_CommandErr(0x10,2);    //����д�����ش�����02:��Ч��ַ
               else if(Modbus_FrameLenth!=RegNum*2+9) ModBus_CommandErr( 0x10,1);//֡�����������ش�����01:��Ч����
               else ModBus_Command16(StartAddr,RegNum,CMD_BUF);          //��ȷ������֮
			   break;
            }
		  case 0x01:
		   {
			   StartAddr=CMD_BUF[2]*256+CMD_BUF[3];                      //�Ĵ�����ַ
			   RegNum=(CMD_BUF[4]*256+CMD_BUF[5]);                       //�Ĵ�������
			   if (RegNum>(__RELAY_NUM-StartAddr))	ModBus_CommandErr(0x01,1);//��������Χ
			   else if(StartAddr>(__RELAY_NUM-1))	ModBus_CommandErr(0x01,2);//��ַ����Χ
			   else ModBus_Command01_ReadRelay(StartAddr,RegNum);
			   break;
		   }
		  case 0x05:   //������5 (0x05) [д��һ��Ȧ]
		   {
			   StartAddr=CMD_BUF[2]*256+CMD_BUF[3];                      //�Ĵ�����ַ
			   WR_Data=(CMD_BUF[4]*256+CMD_BUF[5]);                      //д�������
			   ModBus_Command05_WriteRelay(StartAddr,WR_Data);
			   break;
		   }
	
		  case 0x01://WR
		  
			StartAddr=CMD_BUF[2]*256+CMD_BUF[3];                      //�Ĵ�����ַ
			offset = StartAddr-__RELAY_REG_SATART_ADDR;
			RegNum=(CMD_BUF[4]*256+CMD_BUF[5]);                       //�Ĵ�������
			if ((StartAddr>9999)||(StartAddr<1)) ModBus_CommandErr(0x01,2);//ָ���ַ����Χ
			else if (RegNum>(__RELAY_NUM-offset))	ModBus_CommandErr(0x01,1);//��������Χ
			else if(offset>(__RELAY_NUM-1))	ModBus_CommandErr(0x01,3);//��ַ����Χ
			else My_ModBus_Command01_ReadRelay(offset,RegNum);
			
			break;
		  case 0x05://WR
		  
			StartAddr=CMD_BUF[2]*256+CMD_BUF[3];                      //�Ĵ�����ַ
			offset = StartAddr-__RELAY_REG_SATART_ADDR;
			WR_Data=(CMD_BUF[4]*256+CMD_BUF[5]);                      //д�������
			if ((StartAddr>9999)||(StartAddr<1)) ModBus_CommandErr(0x01,2);//ָ���ַ����Χ
			else if (offset >(__RELAY_NUM-1))ModBus_CommandErr(0x05,3);//��ַ����Χ
			else My_ModBus_Command05_WriteRelay(offset,WR_Data);
			
			break;
		  case 0x0f://WR
		  
			StartAddr=CMD_BUF[2]*256+CMD_BUF[3];                      //�Ĵ�����ַ
			offset = StartAddr-__RELAY_REG_SATART_ADDR;
			RegNum=(CMD_BUF[4]*256+CMD_BUF[5]);                      //д�������
			if(RegNum>(__RELAY_NUM-offset))ModBus_CommandErr(0x0f,1);//��������Χ
			else if (offset >(__RELAY_NUM-1))ModBus_CommandErr(0x05,2);//��ַ����Χ
			else My_ModBus_Command15_WriteRelay(offset,RegNum,CMD_BUF);
			*/
			break;
		  case 0x02://R ��ɢ�Ĵ���
			ModBus_CommandErr(0x02,4);//δʵ��ָ��
			break;
		  case 0x04://R AD
			ModBus_CommandErr(0x04,4);//δʵ��ָ��
			break;
		  case 0x03://WR
		  /*
            StartAddr=CMD_BUF[2]*256+CMD_BUF[3];                     //��ȡ�Ĵ�����ʼ��ַ
            RegNum=(CMD_BUF[4]*256+CMD_BUF[5]);                      //��ȡ�Ĵ�������
            EndAddr  =StartAddr+RegNum;                              //�������һ���Ĵ����ĵ�ַ
            if     (Modbus_FrameLenth!=8) ModBus_CommandErr(0x03,1); //���ݰ���ȱ�����ش�����01����Чָ��
            else if((StartAddr<__HOLDING_REG_START_ADDR)||(StartAddr>49999)) ModBus_CommandErr(0x03,2);      //�����Ĵ�����ַ��Χ�����ش�����02:��Ч��ַ
            else   My_ModBus_Command03(StartAddr,RegNum);               //��ȷ��03ָ��������ݰ���
			
            break;
		  case 0x06://WR
		  
            WR_Addr=CMD_BUF[2]*256+CMD_BUF[3];                        //д��Ĵ����ĵ�ַ
            WR_Data=(CMD_BUF[4]*256+CMD_BUF[5]);                      //д�������
            if      (Modbus_FrameLenth!=8) ModBus_CommandErr(0x06,1); //���ݰ���ȱ��������01����Чָ��
            else if((WR_Addr<__HOLDING_REG_START_ADDR)||(WR_Addr>49999)) ModBus_CommandErr(0x06,2);//��ַ�����������02:��Ч��ַ
            else My_ModBus_Command06(WR_Addr,WR_Data,CMD_BUF);                   //��ȷָ��������ݰ���
			
			break;
		  case 0x10://WR
		  
            StartAddr=CMD_BUF[2]*256+CMD_BUF[3];                        //д��Ĵ����ĵ�ַ
            RegNum=(CMD_BUF[4]*256+CMD_BUF[5]);                      //д�������         
            if((StartAddr<__HOLDING_REG_START_ADDR)||(WR_Addr>49999)) ModBus_CommandErr(0x06,2);//��ַ�����������02:��Ч��ַ		
            else My_ModBus_Command16(StartAddr,RegNum,CMD_BUF);                   //��ȷָ��������ݰ���
			*/
			break;
          default:
             {
               ModBus_CommandErr(CMD_BUF[1],1);         //�޷�������������ش�����01: ��Ч����
               break;
             }               
        }
END:
  RxFrameFlag=0;        //Modbus����֡�������
  __EN_UART_INT;//ES=1;                 //�����жϣ���������֡��ʱ�ſ�ʼ����
}


/****************************************************************************
* ��    �ƣ�Modbus_Tick()
* ��    �ܣ�ModbusЭ��������֡�Ĳ����Լ���ʱ����صĴ���
* ��ڲ�������
* ���ڲ������ޡ�
* ˵    ��: ���������Եص��øú������Ƽ���ʱ�ж��ڵ��á���ʱ�ж�����1-10ms
****************************************************************************/
void Modbus_Tick()
{
  unsigned char CharsCnt_N;    //��Ž��ջ������ڵ�ǰ�ַ����ı���  (_N=Now)
  static unsigned char CharsCnt_P;    //��Ž��ջ�������ǰһ���ַ����ı���(_P=Previous)
  static unsigned char IdleLineTimer; //���ڿ��У������ݣ���ʱ���������ж�֡�����á�
  if(UART_IsTxEmpty()) DIR_RX;       //��������Ѿ�����������֡��485�����л��ؽ��ա�
  CharsCnt_N= UART_GetCharsInRxBuf();  		   //��ȡ��ǰ���ջ������ڵ��ַ���
  if(CharsCnt_P!=CharsCnt_N) IdleLineTimer=0;  //����������λ������ڵ��ַ������ȣ�˵������֡���ڱ�����
  if((CharsCnt_P==CharsCnt_N)&&(CharsCnt_N>0)) //����������λ��������ַ������,�Ҳ�Ϊ0,˵����·����
     {                                         
          IdleLineTimer++;                     //�ۼӼ�����·���е�ʱ��
          if(IdleLineTimer>DIR_DELAY)           //����·�����ۼ�ʱ�䵽���趨ʱ�䣬���ж�Ϊ����֡����
          {
             IdleLineTimer=0;                                           
             RxFrameFlag=1;                    //������֡���ճɹ���־λ
             CharsCnt_N=0;
             CharsCnt_P=0;                     //����ַ�����¼
          }
     }
  CharsCnt_P= CharsCnt_N;                      //���浱ǰ���ַ�����Ϊ��һ�αȽϵ��о�
}

/****************************************************************************
 
                       Layer4.  Ӧ�ò�ӿ�:

****************************************************************************/

/****************************************************************************
* ��    �ƣ�Modbus_SetReg()
* ��    �ܣ�����1���Ĵ�����ֵ
* ��ڲ�����Addr:�����õļĴ�����ַ
            Data:���õ���ֵ
* ���ڲ������ޡ�
* ˵    ��: 
****************************************************************************/
#if(MB_SET_REG)
void Modbus_SetReg(unsigned int Addr,unsigned int Data)
{
   if(Addr>REG_NUM)  return;   //�����˵�ַ��Χ��ֱ�ӷ���
   REG_BUF[Addr]=Data;         //д������
}
#endif
/****************************************************************************
* ��    �ƣ�Modbus_GetReg()
* ��    �ܣ���ȡ1���Ĵ�����ֵ
* ��ڲ�����Addr:����ȡ�ļĴ�����ַ
* ���ڲ������Ĵ�������ֵ��
* ˵    ��: 
****************************************************************************/
#if(MB_GET_REG)
unsigned int Modbus_GetReg(unsigned int Addr)
{
   return(REG_BUF[Addr]); 
}
#endif
/****************************************************************************
* ��    �ƣ�Modbus_SetAddr()
* ��    �ܣ����ñ������豸��ַ
* ��ڲ�����Addr:�豸��ַ
* ���ڲ�������
* ˵    ��: ���������Խ��豸��ַ��Ϊ0-255����Modbus�涨����Ч�豸��ַ��Χ
            ��1-247��
****************************************************************************/
#if(MB_SET_ADDR)
void Modbus_SetAddr(unsigned char Addr)
{
   SlaveAddr=Addr;
}
#endif
/****************************************************************************
* ��    �ƣ�Modbus_GetAddr()
* ��    �ܣ���ȡ�������豸��ַ
* ��ڲ�������
* ���ڲ������������豸��ַ
****************************************************************************/
#if(MB_GET_ADDR)
unsigned char Modbus_GetAddr()
{
   return(SlaveAddr);
}
void Send_Addr(void){

  SendByte_CRC(Modbus_GetAddr());   //���ش������
  UART_PutChar(CRCREG%256);
  UART_PutChar(CRCREG/256); 
}
#endif