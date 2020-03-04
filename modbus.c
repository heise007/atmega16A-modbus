/*
               
                         Modbus-RTU协议[从机]通讯程序库

  说明：Modbus-RTU是使用最为广泛的工业现场总线协议之一，自从1970年被Modicon公司提出以来
        至今40多年的应用经久不衰。Modbus-RTU协议简短精巧，在7层模型中只占有简单的2层（链
        路层和协议层），可以以各种物理层作为通信介质(RS-485、光纤、无线、网线、TCP/IP)，
        也可和各种应用层接口。本程序使用RS-485总线作为物理层，并提供了基本寄存器读写操作
        作为应用层接口。
            Modbus-RTU协议是一种寄存器基的通讯协议。它将串行的数据流虚拟成静态的寄存器，
        所有的数据读写都被虚拟成了寄存器的读写。例如通信双方约定0号寄存器存有"温度值"，
        1号寄存器存有“温度上限”。单片机测温后将温度测量结果存入0号寄存器，主机可以随时
        通过命令读取0号寄存器，就读取了温度值，单片机读取寄存器1的数据作为报警上限值，主机
        也可以随时通过写命令更改1号寄存器，实现更改温度上限值的操作。
           
           本程序采用了寄存器缓冲区作为协议层和应用层之间的接口，该寄存器缓存可以随时被
        Modbus协议栈程序访问，也可以随时被应用程序访问。远方主机通过Modbus指令修改了寄存器，
        应用程序可以访问寄存器缓冲区得知，应用程序的数据（如各种测量结果）也可以写入寄存
        器缓冲区，从而可以被远方主机读取。本程序能够解析Modbus协议中3种命令：
            03号命令：连续读取多个寄存器
            06号命令：写单个寄存器
            16号命令：连续写多个寄存器

  准备工作：
   1.   本程序以《带缓冲区的串口模块化程序库》中的串口收发函数作为底层通信链路，因此要
        使用本函数库，需要将UART.c、UART.h 以及本文件(Modbus.c)和Modbus.h共4个文件添加进工程，
        并且在需要调用寄存器读写函数的文件开头包含Modbus.h。在使用本函数库之前，请先配置串口
        函数库UART.c，保证下面4个函数被开启：
            1.UART_GetChar
            2.UART_PutChar
            3.UART_GetCharsInRxBuf
            4.UART_IsTxEmpty  
   2.   设置串口收发缓冲区大小，原则是：
            1.TXBUF_SIZE的大小必须大于03指令最长可能发送的字节数(最多可能连续读取的寄存器数*2+5)
            2.RXBUF_SIZE的大小必须大于16指令最长可能接收的字节数(最多可能连续写入的寄存器数*2+9)  

   3.   在本文件开头处设置Modbus协议相关参数

   4.   主程序中，开启一个定时中断，在中断内调用Modbus_Tick()函数。定时周期1-10ms左右

   5.   Modbus_Process()函数也需要被周期性地调用，可以在定时中断内调用，但是该函数执行时间较长
        (要解析协议、发送数据)而且执行时间与命令要读写的字节数成正比，如果中断内调用，会使中断
        执行时间变长，甚至长于定时周期，导致其他中断响应延迟，并使得主循环执行变慢。该函数也可
        以在主循环内调用，这时回应数据帧将会被延迟最多1个主循环周期时间。
        

Modbus协议简介：Modbus协议中，数据存储、传输均以"寄存器"为单位，每个寄存器都是2字节数据，高字节
       在先。通讯帧中，除CRC校验低字节在先以外，其余双字节数据均高字节在先。
       Modbus协议的每次通讯请求必须由主机发起，主机按下述格式发送通讯请求帧，各命令的帧结构如下：

   03号命令(读寄存器):
       命令格式:设备地址[1字节]+功能码[03H]+起始寄存器[2字节]+读取寄存器个数N[2字节]+CRC[2字节]
       返回格式:设备地址[1字节]+功能码[03H]+字节数[1字节]+依次N个寄存器值[2N字节]+CRC[2字节]

   06号命令(写单个寄存器):
       命令格式: 设备地址[1字节]+功能码[06H]+写入寄存器地址[2字节]+写入数据[2字节]+CRC[2字节]
       返回格式：原样重复一遍上述命令

   16号(10H号)命令(连续写多个寄存器)
       命令格式: 设备地址[1字节]+功能码[10H]+写入起始寄存器地址[2字节]+写入寄存器个数N[2字节]
                 +写入数据字节总数(2N)[1字节]+依次写入的寄存器数据[2N字节]+CRC[2字节]
       返回格式：设备地址[1字节]+功能码[10H]+写入起始寄存器地址[2字节]+写入寄存器个数N[2字节]+CRC[2B]

通讯范例：
主机->从机
字节:   字节1	字节2	字节3	字节4   字节5	字节6	    字节7	字节8
数据:   0x01	0x03	0x00    0x00    0x00    0x04	     0x44	0x09
含义:   地址    读	  从寄存器0000开始  共读取0004个寄存器	   CRC校验

1号从机响应，返回数据：
字节:	字节1	字节2	字节3	   字节4	字节5	字节6	字节7	字节8	字节9   字节10	字节11	字节13	字节14
数据:	0x01	0x03	0x08	   0x13     0x88	0x00    0x04	0x00     0x14   0x6D    0x41	 XX      XX
含义	地址   读成功  返回8字节    寄存器0=0x1348  寄存器1=0x0400  寄存器2=0x0014   寄存器2=0x6D41    CRC校验
                        	

                             
     
  (C)西安电子科技大学 测控技术与仪器教研中心 编写：谢楷 2009/11/11 
  
*/                        
//            
//                                               软件结构
//|           |                                                  |                           |                            |
//|<--物理层->|<------------------链路层------------------------>|<----------协议层--------->|<----------应用层---------->|
//|           |                                                  |                           |                            |
//
//            +------------+   +-------------+   +-------------+   +---------+   +---------+   +--------------+
//            |  串    口  |   |  RX_BUFF[]  |   |UART_GetChar |   |         |   |         |   |Modbus_GetReg |
// RXD引脚 -->|  接收中断  |-->|  接收缓冲   |-->|串口读取函数 |-->|Modbus_  |-->|         |-->|寄存器读取函数|-->应用程序  
//            |            |   |  队列(FIFO) |   |             |   |Process()|   |         |   |              |
//            +------------+   +-------------+   +-------------+   |  函数   |   |REG_BUF[]|   +--------------+
//                                                                 |         |   | 寄存器  |                  
//            +------------+   +-------------+   +-------------+   |         |   | 缓存区  |   +--------------+
//            |  串    口  |   |  TX_BUFF[]  |   |UART_PutChar |   |(Modbus  |   |         |   |Modbus_SetReg |
// TXD引脚 <--|  发送中断  |<--|  发送缓冲   |<--|串口发送函数 |<--| 协议栈) |<--|         |<--|寄存器设置函数|<--应用程序  
//            |            |   |  队列(FIFO) |   |             |   |         |   |         |   |              |
//            +------------+   +-------------+   +-------------+   +---+-----+   +---------+   +--------------+
//                                                                     |    
//                                                              +------+-------+
//                                                              |定时中断内调用|
/// DIR引脚 <---------------------------------------------------|Modbus_Tick() |
//                                                              |提供计时节拍  |
//                                                              +--------------+
//

//本程序硬件连接：
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
#include "UART.h"         /* Modbus协议栈需要调用串口收发函数作为链路层*/
#include "Modbus.h"
//#include "bsp.h"
//#include "config.h"

/*本程序库需要调用以下4个串口函数，在UART.c中，它们的编译开关必须被打开。
UART_GetChar、UART_PutChar、UART_GetCharsInRxBuf、UART_IsTxEmpty      

TXBUF_SIZE的大小必须大于03指令最长可能发送的字节数
RXBUF_SIZE的大小必须大于16指令最长可能接收的字节数                     */




/*-------------------本行以下内容请勿修改---------------------*/

unsigned char SlaveAddr=1;        /*通讯地址，开机默认通讯地址=1 */
unsigned int  REG_BUF_LOCATE  REG_BUF[REG_NUM];    /*Modbus寄存器缓冲区*/
unsigned int  CRCREG=0xffff;     /*CRC迭代计算暂存变量*/
unsigned char RxFrameFlag=0;               /*收到数据帧的标志*/


/*****************************************************************
 
                   Layer1.  物理层：TTL/485芯片

*****************************************************************/

/*****************************************************************
 
                   Layer2.  链路层：UART.c内提供的串口收发函数

*****************************************************************/

/*****************************************************************
 
                   Layer3.  Modbus协议栈层：

*****************************************************************/
/****************************************************************************
* 名    称：Caculate_CRC16_Byte()
* 功    能：计算CRC校验，以迭代方式计算1字节的CRC校验值
* 入口参数：DAT:迭代计算CRC的1字节数据
            CRC:前一次的的CRC迭代结果
* 出口参数：当前的CRC值。
* 说    明: 该函数适合串口每发送一字节时迭代计算CRC校验，以节省时间
****************************************************************************/
unsigned int Caculate_CRC16_Byte(unsigned char DAT,unsigned int CRC)
{  unsigned char j;
         CRC=CRC^(unsigned int)DAT;
        for(j=0;j<8;j++)
          { 
             if(CRC&0x01)
             {
               CRC=CRC>>1;
               CRC=CRC^0xA001;       //一个经典的CRC迭代算法
             }
             else
             {
               CRC=CRC>>1;           
             } 
          } 
   return(CRC);               
}
/****************************************************************************
* 名    称：Caculate_CRC16_BUF()
* 功    能：计算一个数组的CRC校验
* 入口参数：*DAT:存放数据的数组名
            Lenth:数组中需要计算CRC的字节数
* 出口参数：CRC计算结果。
* 说    明: 该函数适合对接收缓冲区、发送缓冲等连续数据计算CRC校验值
****************************************************************************/
unsigned int Caculate_CRC16_BUF(unsigned char *DAT,unsigned char Lenth)
{
   unsigned int CRC=0xffff;                 //CRC初始值=全1
   unsigned char i;
   for(i=0;i<Lenth;i++)                     //对数组中Lenth个数据求CRC校验
      {
        CRC=Caculate_CRC16_Byte(DAT[i],CRC);//依次迭代计算
      }
  return(CRC);                              //返回结果
}    
/****************************************************************************
* 名    称：SendByte_CRC()
* 功    能：发送一个字节，同时迭代计算CRC校验值
* 入口参数：DAT:待发送的1字节数据
* 出口参数：无。
* 说    明: 该函数调用串口发送1字节函数
****************************************************************************/
void SendByte_CRC(unsigned char DAT)
 {
   CRCREG=Caculate_CRC16_Byte(DAT,CRCREG);//先迭代计算CRC
   UART_PutChar(DAT);                     //再从串口发出
 }                                        //无返回值，计算结果在CRCREG全局变量中

/****************************************************************************
* 名    称：SendWord_CRC()
* 功    能：发送一个16位数据，同时迭代计算CRC校验值
* 入口参数：WORD:待发送的2字节整形数据
* 出口参数：无。
****************************************************************************/
void SendWord_CRC(unsigned int WORD)
{
   SendByte_CRC(WORD/256);
   SendByte_CRC(WORD%256); 
}
/****************************************************************************
* 名    称：ModBus_Command03()
* 功    能：解析功能码[03]
* 入口参数：StartAddr:读取的起始寄存器
            RegNum   :读取的寄存器个数
* 出口参数：无。
* 说    明: Modbus协议中，功能码[03]:连续读取多个寄存器
            命令格式: 设备地址[1B]+功能码[1B]+起始寄存器[2B]+读取寄存器个数N[2B]+CRC[2B]
            返回格式：设备地址[1B]+功能码[1B]+字节数[1B]+DATA[2NB]+CRC[2B]
****************************************************************************/
void ModBus_Command03(StartAddr,RegNum)
{
  unsigned int EndAddr=StartAddr+RegNum;      //计算最后一个被读取的寄存器地址
  unsigned int Addr;

  SendByte_CRC(SlaveAddr);                    //返回地址
  SendByte_CRC(0x03);                         //返回功能码 03            
  SendByte_CRC(RegNum*2);                     //返回数据区字节数(2*寄存器数)
  for(Addr=StartAddr;Addr<EndAddr;Addr++)     //按照命令要求依次返回数据
    {
      SendWord_CRC(REG_BUF[Addr]);            //RegNum个寄存器，依次发送
    }
  UART_PutChar(CRCREG%256);                   //CRC,低位在先
  UART_PutChar(CRCREG/256); 
}
/****************************************************************************
* 名    称：ModBus_Command06()
* 功    能：解析功能码[06]
* 入口参数：WR_Addr:写入的寄存器地址
            WR_Data:写入的数据
            *Ptr:指向命令数据帧的指针
* 出口参数：无。
* 说    明: Modbus协议中，功能码[06]:写单个寄存器
            命令格式: 设备地址[1B]+功能码[1B]+寄存器地址[2B]+写入数据[2B]+CRC[2B]
            返回格式：原样重复一遍上述命令
****************************************************************************/
void ModBus_Command06(unsigned int WR_Addr,unsigned int WR_Data,unsigned char CMD_BUF_LOCATE *Ptr)
{           
    unsigned char i;
    REG_BUF[WR_Addr]=WR_Data;                //将数据存入相应的寄存器中
    for(i=0;i<8;i++)  UART_PutChar(Ptr[i]);  //原样重复一遍命令帧作为回复
}
/****************************************************************************
* 名    称：ModBus_Command16()
* 功    能：解析功能码[16]
* 入口参数：StartAddr:写入的起始寄存器
            RegNum   :写入的寄存器个数
            *Ptr:指向命令数据帧的指针
* 出口参数：无。
* 说    明: Modbus协议中，功能码[16]:连续写多个寄存器
            命令格式: 设备地址[1B]+功能码[1B]+写入起始寄存器地址[2B]++写入寄存器个数N[2B]+字节数(=2N)[1B]+写入数据[2NB]+CRC[2B]
            返回格式：设备地址[1B]+功能码[1B]+写入起始寄存器地址[2B]++写入寄存器个数N[2B]+CRC[2B]
****************************************************************************/
void ModBus_Command16(unsigned char StartAddr,unsigned char RegNum,unsigned char CMD_BUF_LOCATE *Ptr)
{
  unsigned int EndAddr=StartAddr+RegNum;   //计算最后一个被写入的寄存器地址
  unsigned int Addr,Data,i=0;
  for(Addr=StartAddr;Addr<EndAddr;Addr++)  //从起始地址开始，依次写入
    {
       Data=256*Ptr[2*i+7]+Ptr[2*i+8];     //从命令帧中依次获取被写入的数据
       REG_BUF[Addr]=Data;                 //写入相应的寄存器
       i++;                                //下一个
    } 
  SendByte_CRC(SlaveAddr);                 //返回地址
  SendByte_CRC(0x10);                      //返回功能码 16            
  SendWord_CRC(StartAddr);                 //返回写入的起始寄存器
  SendWord_CRC(RegNum);                    //返回写入的寄存器数
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
	
	SendByte_CRC(SlaveAddr);                 //返回地址
	SendByte_CRC(0x05);                      //返回功能码 16            
	SendWord_CRC(StartAddr);                 //返回写入的起始寄存器
	SendWord_CRC(WR_Data);                    //返回写入的数
	UART_PutChar(CRCREG%256);
	UART_PutChar(CRCREG/256);                //CRC
	*/
}
ModBus_Command01_ReadRelay(unsigned int StartAddr,unsigned int RegNum){
	/*
	unsigned char reg_group=0,status_byte_temp=0;//
	unsigned char temp=0;
	unsigned char status_group_temp[__RELAY_GROUP];
	unsigned char count=0;//记录已读取线圈状态的数量
	if (RegNum%8==0){
		reg_group = RegNum/8;
	}else{
		reg_group = RegNum/8+1;
	}
	
	SendByte_CRC(SlaveAddr);                 //返回地址
	SendByte_CRC(0x01);                      //返回功能码 16            
	SendByte_CRC(reg_group);                //返回继电器状态的字节数
	for (unsigned char i=0;i<reg_group;i++)
	{
		status_group_temp[i]=0;
		status_byte_temp=0;
		for (unsigned char j=0;j<8;j++){
			if (count>RegNum){//所有线圈的状态都读完了
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
* 名    称：ModBus_CommandErr()
* 功    能：返回各种错误代号
* 入口参数：Command:在何种命令时发生了错误
            ExceptionCode:错误代号
* 出口参数：无。
* 说    明: Modbus协议中，错误码格式是
            设备地址[1B]+(0x80+功能码)[1B]+错误代码[1B]+CRC[2B]
****************************************************************************/
void ModBus_CommandErr(unsigned char Command,unsigned char ExceptionCode)
{
  SendByte_CRC(SlaveAddr); 
  SendByte_CRC(0x80+Command);    //命令错误
  SendByte_CRC(ExceptionCode);   //返回错误代码
  UART_PutChar(CRCREG%256);
  UART_PutChar(CRCREG/256); 
}
void My_ModBus_Command01_ReadRelay(unsigned char offset,unsigned char reg_num){
	/*
	unsigned char relay_status = *(volatile unsigned char *)Reg_Data[offset].addr_l;
	SendByte_CRC(SlaveAddr);                 //返回地址
	SendByte_CRC(0x01);                      //返回功能码 01
	SendByte_CRC(1);                //返回继电器状态的字节数
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
	
	SendByte_CRC(SlaveAddr);                 //返回地址
	SendByte_CRC(0x05);                      //返回功能码 16
	SendWord_CRC(offset+__RELAY_REG_SATART_ADDR);                 //返回写入的起始寄存器
	SendWord_CRC(WR_Data);                    //返回写入的数
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
	for (i=0;i<reg_group;i++){//循环寄存器组数
		asm("nop");
		if((reg_num-i)<8){//循环到最后一个字节且剩余寄存器数量不足8个了
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
	SendByte_CRC(SlaveAddr);                 //返回地址
	SendByte_CRC(0x0f);                      //返回功能码 16
	SendWord_CRC(offset+__RELAY_REG_SATART_ADDR);                 //返回写入的起始寄存器
	SendWord_CRC(reg_num);                    //返回写入的数
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
		UART_PutChar(data);  //原样重复一遍命令帧作为回复
	}

	if (eeprom_addr==2){
		Modbus_SetAddr((unsigned char)(WR_Data&0x00ff));
	}

	eeprom_write_byte(eeprom_addr,(unsigned char)(WR_Data&0x00ff));
	eeprom_write_byte(eeprom_addr+1,(unsigned char)((WR_Data&0xff00)>>8));

	_delay_ms(200);
	//重新读取eeprom读取到内存
	*/
}
void My_ModBus_Command03(unsigned int StartAddr,unsigned char RegNum){
	/*
	unsigned int EndAddr=StartAddr+RegNum-__HOLDING_REG_START_ADDR;      //计算最后一个被读取的寄存器地址
	unsigned int start_addr=StartAddr-__HOLDING_REG_START_ADDR; 
	unsigned int Addr;
	unsigned int i=0,data=0;
	unsigned int data_temp[200];
	unsigned char reg_num=0;
	unsigned int data_len=0;
	
	unsigned int tt=0;
	reg_num=EndAddr-start_addr;
	for (i=0;i<reg_num;i++){//循环寄存器数量
		tt=__HOLDING_REG_OFFSET +start_addr+i;
		//去eeprom读取
		if ((Reg_Data[__HOLDING_REG_OFFSET +start_addr+i].addr_l==0)&&(Reg_Data[__HOLDING_REG_OFFSET +start_addr+i].addr_h==0)){

			break;
		}
		data = eeprom_read_byte(Reg_Data[__HOLDING_REG_OFFSET +start_addr+i].addr_l+1);
		data <<= 8;
		data |= eeprom_read_byte(Reg_Data[__HOLDING_REG_OFFSET +start_addr+i].addr_l);

		data_temp[i]=data;
	}
	if (i<reg_num){
		 ModBus_CommandErr(0x10,3);    //不可写，返回错误码03:寄存器地址超范围,非法数据值
		 return;
	}
	SendByte_CRC(SlaveAddr);                    //返回地址
	SendByte_CRC(0x03);                         //返回功能码 03
	SendByte_CRC(i*2);                     //返回数据区字节数(2*寄存器数)
	data_len=i;
	for (i=0;i<data_len;i++){
	 	SendWord_CRC(data_temp[i]);
		
	}

	UART_PutChar(CRCREG%256);                   //CRC,低位在先
	UART_PutChar(CRCREG/256);	
	*/
}
 void My_ModBus_Command16(unsigned int start_addr,unsigned int reg_num,unsigned char CMD_BUF_LOCATE *Ptr){
	 /*
	unsigned int start_addr_temp=start_addr-__HOLDING_REG_START_ADDR;
	unsigned int endaddr_temp=start_addr+reg_num-__HOLDING_REG_START_ADDR;      //计算最后一个被读取的寄存器地址
	unsigned int data=0;
	unsigned char i=0;
	unsigned int data_temp[200];
	unsigned int tt=0;
	//*Ptr[7]开始就是真正的待写入数据
	for (i=0;i<reg_num;i++){//循环寄存器数量,看范围超出没，
		tt = __HOLDING_REG_OFFSET +start_addr_temp+i;
		if ((Reg_Data[__HOLDING_REG_OFFSET +start_addr_temp+i].addr_l==0)&&(Reg_Data[__HOLDING_REG_OFFSET +start_addr_temp+i].addr_h==0)){			
			ModBus_CommandErr(0x10,3);    //不可写，返回错误码03:寄存器地址超范围
			return;
		}
	}
	//范围没超出
	for (i=0;i<reg_num;i++){
		tt=Ptr[7+i*2];
		tt=Ptr[7+i*2+1];
		eeprom_write_byte(Reg_Data[__HOLDING_REG_OFFSET +start_addr_temp+i].addr_l+1,Ptr[7+i*2]);
		eeprom_write_byte(Reg_Data[__HOLDING_REG_OFFSET +start_addr_temp+i].addr_l,Ptr[7+i*2+1]);
	}
	SendByte_CRC(SlaveAddr);                    //返回地址
	SendByte_CRC(0x10);                         //返回功能码 03
	SendWord_CRC(reg_num);
	SendByte_CRC(i*2);                     //返回数据区字节数(2*寄存器数)

	UART_PutChar(CRCREG%256);                   //CRC,低位在先
	UART_PutChar(CRCREG/256);

	if (Modbus_GetAddr()!=eeprom_read_byte(2)){
		Modbus_SetAddr(eeprom_read_byte(2));
	}
*/
 }
/****************************************************************************
* 名    称：Modbus_Process()
* 功    能：Modbus协议解析与处理
* 入口参数：无
* 出口参数：无。
* 说    明: 必须周期性地调用该函数。可以在定时中断内调用，但是该函数的执行
            时间较长，在中断内调用可能会造成其他中断实时性下降，也可以放在
            主循环内执行,这时数据返回最多会被延迟1个主循环周期时间。
****************************************************************************/
void Modbus_Process()
{ 
  unsigned char CMD_BUF_LOCATE  CMD_BUF[REG_NUM*2+10]; //Modbus命令帧缓冲
  unsigned int StartAddr,EndAddr,RegNum,WR_Addr,WR_Data;
  unsigned char Modbus_FrameLenth;          //命令帧长度变量
  unsigned char i,Chr,offset;
  if(RxFrameFlag!=1)  return;               //没收到数据帧，直接返回
  __DIS_UART_INT;//ES=0;                                     //串口数据判别期间关闭串口收发，避免期间收到新字符导致混乱
  Modbus_FrameLenth=UART_GetCharsInRxBuf(); //数据帧长度=当前接收缓冲区长度
  for(i=0;i<Modbus_FrameLenth;i++)          
      {
        UART_GetChar(&Chr);                 //依次取出串口缓冲区内所有的字符，
        CMD_BUF[i]=Chr;                     //存入命令求缓冲区内
      }
  asm("nop");
  if(CMD_BUF[0]!=SlaveAddr)  goto END;      //地址不符，非本机请求不处理，返回
  if(Caculate_CRC16_BUF(CMD_BUF,Modbus_FrameLenth)!=0) goto END;//校验错误,不处理，返回   
  CRCREG=0xffff;
  DIR_TX;                                   //485设为输出，下面开始返回数据
  switch (CMD_BUF[1])                       //判断收到的是什么命令
        {
		/*
          case 0x03:    //功能码03 [连续读多个寄存器]
             {
               StartAddr=CMD_BUF[2]*256+CMD_BUF[3];                     //读取寄存器起始地址
               RegNum=(CMD_BUF[4]*256+CMD_BUF[5]);                      //读取寄存器个数
               EndAddr  =StartAddr+RegNum;                              //计算最后一个寄存器的地址
               if     (Modbus_FrameLenth!=8) ModBus_CommandErr(0x03,1); //数据包残缺，返回错误码01：无效指令
               else if(EndAddr>REG_NUM) ModBus_CommandErr(0x03,2);      //超出寄存器地址范围，返回错误码02:无效地址
               else   ModBus_Command03(StartAddr,RegNum);               //正确的03指令，解析数据包。
               break;
             }
          case 0x06:   //功能码06 [写单个寄存器]
             {                                                          
               WR_Addr=CMD_BUF[2]*256+CMD_BUF[3];                        //写入寄存器的地址
               WR_Data=(CMD_BUF[4]*256+CMD_BUF[5]);                      //写入的数据
               if      (Modbus_FrameLenth!=8) ModBus_CommandErr(0x06,1); //数据包残缺，错误码01：无效指令
               else if((WR_Addr<WR_START)||(WR_Addr>WR_END)) ModBus_CommandErr(0x06,2);//地址溢出，错误码02:无效地址
               else ModBus_Command06(WR_Addr,WR_Data,CMD_BUF);                   //正确指令，解析数据包。
               break;                      
             }
          case 0x10:   //功能码16 (0x10) [连续写多个寄存器]
            {
               StartAddr=CMD_BUF[2]*256+CMD_BUF[3];                      //连续写入寄存器起始地址
               RegNum=(CMD_BUF[4]*256+CMD_BUF[5]);                       //连续写入寄存器个数
               EndAddr  =StartAddr+RegNum;                               //计算最后一个寄存器的地址
               if(StartAddr<WR_START)      ModBus_CommandErr(0x10,2);    //不可写，返回错误码02:无效地址
               else if(EndAddr>WR_END)     ModBus_CommandErr(0x10,2);    //不可写，返回错误码02:无效地址
               else if(Modbus_FrameLenth!=RegNum*2+9) ModBus_CommandErr( 0x10,1);//帧长不符，返回错误码01:无效命令
               else ModBus_Command16(StartAddr,RegNum,CMD_BUF);          //正确，解析之
			   break;
            }
		  case 0x01:
		   {
			   StartAddr=CMD_BUF[2]*256+CMD_BUF[3];                      //寄存器地址
			   RegNum=(CMD_BUF[4]*256+CMD_BUF[5]);                       //寄存器个数
			   if (RegNum>(__RELAY_NUM-StartAddr))	ModBus_CommandErr(0x01,1);//数量超范围
			   else if(StartAddr>(__RELAY_NUM-1))	ModBus_CommandErr(0x01,2);//地址超范围
			   else ModBus_Command01_ReadRelay(StartAddr,RegNum);
			   break;
		   }
		  case 0x05:   //功能码5 (0x05) [写单一线圈]
		   {
			   StartAddr=CMD_BUF[2]*256+CMD_BUF[3];                      //寄存器地址
			   WR_Data=(CMD_BUF[4]*256+CMD_BUF[5]);                      //写入的数据
			   ModBus_Command05_WriteRelay(StartAddr,WR_Data);
			   break;
		   }
	
		  case 0x01://WR
		  
			StartAddr=CMD_BUF[2]*256+CMD_BUF[3];                      //寄存器地址
			offset = StartAddr-__RELAY_REG_SATART_ADDR;
			RegNum=(CMD_BUF[4]*256+CMD_BUF[5]);                       //寄存器个数
			if ((StartAddr>9999)||(StartAddr<1)) ModBus_CommandErr(0x01,2);//指令地址超范围
			else if (RegNum>(__RELAY_NUM-offset))	ModBus_CommandErr(0x01,1);//数量超范围
			else if(offset>(__RELAY_NUM-1))	ModBus_CommandErr(0x01,3);//地址超范围
			else My_ModBus_Command01_ReadRelay(offset,RegNum);
			
			break;
		  case 0x05://WR
		  
			StartAddr=CMD_BUF[2]*256+CMD_BUF[3];                      //寄存器地址
			offset = StartAddr-__RELAY_REG_SATART_ADDR;
			WR_Data=(CMD_BUF[4]*256+CMD_BUF[5]);                      //写入的数据
			if ((StartAddr>9999)||(StartAddr<1)) ModBus_CommandErr(0x01,2);//指令地址超范围
			else if (offset >(__RELAY_NUM-1))ModBus_CommandErr(0x05,3);//地址超范围
			else My_ModBus_Command05_WriteRelay(offset,WR_Data);
			
			break;
		  case 0x0f://WR
		  
			StartAddr=CMD_BUF[2]*256+CMD_BUF[3];                      //寄存器地址
			offset = StartAddr-__RELAY_REG_SATART_ADDR;
			RegNum=(CMD_BUF[4]*256+CMD_BUF[5]);                      //写入的数据
			if(RegNum>(__RELAY_NUM-offset))ModBus_CommandErr(0x0f,1);//数量超范围
			else if (offset >(__RELAY_NUM-1))ModBus_CommandErr(0x05,2);//地址超范围
			else My_ModBus_Command15_WriteRelay(offset,RegNum,CMD_BUF);
			*/
			break;
		  case 0x02://R 离散寄存器
			ModBus_CommandErr(0x02,4);//未实现指令
			break;
		  case 0x04://R AD
			ModBus_CommandErr(0x04,4);//未实现指令
			break;
		  case 0x03://WR
		  /*
            StartAddr=CMD_BUF[2]*256+CMD_BUF[3];                     //读取寄存器起始地址
            RegNum=(CMD_BUF[4]*256+CMD_BUF[5]);                      //读取寄存器个数
            EndAddr  =StartAddr+RegNum;                              //计算最后一个寄存器的地址
            if     (Modbus_FrameLenth!=8) ModBus_CommandErr(0x03,1); //数据包残缺，返回错误码01：无效指令
            else if((StartAddr<__HOLDING_REG_START_ADDR)||(StartAddr>49999)) ModBus_CommandErr(0x03,2);      //超出寄存器地址范围，返回错误码02:无效地址
            else   My_ModBus_Command03(StartAddr,RegNum);               //正确的03指令，解析数据包。
			
            break;
		  case 0x06://WR
		  
            WR_Addr=CMD_BUF[2]*256+CMD_BUF[3];                        //写入寄存器的地址
            WR_Data=(CMD_BUF[4]*256+CMD_BUF[5]);                      //写入的数据
            if      (Modbus_FrameLenth!=8) ModBus_CommandErr(0x06,1); //数据包残缺，错误码01：无效指令
            else if((WR_Addr<__HOLDING_REG_START_ADDR)||(WR_Addr>49999)) ModBus_CommandErr(0x06,2);//地址溢出，错误码02:无效地址
            else My_ModBus_Command06(WR_Addr,WR_Data,CMD_BUF);                   //正确指令，解析数据包。
			
			break;
		  case 0x10://WR
		  
            StartAddr=CMD_BUF[2]*256+CMD_BUF[3];                        //写入寄存器的地址
            RegNum=(CMD_BUF[4]*256+CMD_BUF[5]);                      //写入的数据         
            if((StartAddr<__HOLDING_REG_START_ADDR)||(WR_Addr>49999)) ModBus_CommandErr(0x06,2);//地址溢出，错误码02:无效地址		
            else My_ModBus_Command16(StartAddr,RegNum,CMD_BUF);                   //正确指令，解析数据包。
			*/
			break;
          default:
             {
               ModBus_CommandErr(CMD_BUF[1],1);         //无法解析的命令，返回错误码01: 无效命令
               break;
             }               
        }
END:
  RxFrameFlag=0;        //Modbus请求帧处理完毕
  __EN_UART_INT;//ES=1;                 //开启中断，串口数据帧这时才开始发送
}


/****************************************************************************
* 名    称：Modbus_Tick()
* 功    能：Modbus协议中数据帧的捕获以及与时间相关的处理
* 入口参数：无
* 出口参数：无。
* 说    明: 必须周期性地调用该函数。推荐定时中断内调用。定时中断周期1-10ms
****************************************************************************/
void Modbus_Tick()
{
  unsigned char CharsCnt_N;    //存放接收缓冲区内当前字符数的变量  (_N=Now)
  static unsigned char CharsCnt_P;    //存放接收缓冲区内前一次字符数的变量(_P=Previous)
  static unsigned char IdleLineTimer; //串口空闲（无数据）计时变量，供判断帧结束用。
  if(UART_IsTxEmpty()) DIR_RX;       //如果串口已经发完了数据帧，485方向切换回接收。
  CharsCnt_N= UART_GetCharsInRxBuf();  		   //获取当前接收缓冲区内的字符数
  if(CharsCnt_P!=CharsCnt_N) IdleLineTimer=0;  //如果相邻两次缓冲区内的字符数不等，说明数据帧正在被接收
  if((CharsCnt_P==CharsCnt_N)&&(CharsCnt_N>0)) //如果相邻两次缓冲区内字符数相等,且不为0,说明线路空闲
     {                                         
          IdleLineTimer++;                     //累加计算线路空闲的时间
          if(IdleLineTimer>DIR_DELAY)           //当线路空闲累计时间到达设定时间，则判断为数据帧结束
          {
             IdleLineTimer=0;                                           
             RxFrameFlag=1;                    //置数据帧接收成功标志位
             CharsCnt_N=0;
             CharsCnt_P=0;                     //清空字符数记录
          }
     }
  CharsCnt_P= CharsCnt_N;                      //保存当前的字符数作为下一次比较的判据
}

/****************************************************************************
 
                       Layer4.  应用层接口:

****************************************************************************/

/****************************************************************************
* 名    称：Modbus_SetReg()
* 功    能：设置1个寄存器的值
* 入口参数：Addr:被设置的寄存器地址
            Data:设置的数值
* 出口参数：无。
* 说    明: 
****************************************************************************/
#if(MB_SET_REG)
void Modbus_SetReg(unsigned int Addr,unsigned int Data)
{
   if(Addr>REG_NUM)  return;   //超出了地址范围，直接返回
   REG_BUF[Addr]=Data;         //写入数据
}
#endif
/****************************************************************************
* 名    称：Modbus_GetReg()
* 功    能：获取1个寄存器的值
* 入口参数：Addr:被获取的寄存器地址
* 出口参数：寄存器的数值。
* 说    明: 
****************************************************************************/
#if(MB_GET_REG)
unsigned int Modbus_GetReg(unsigned int Addr)
{
   return(REG_BUF[Addr]); 
}
#endif
/****************************************************************************
* 名    称：Modbus_SetAddr()
* 功    能：设置本机的设备地址
* 入口参数：Addr:设备地址
* 出口参数：无
* 说    明: 本函数可以将设备地址设为0-255，但Modbus规定的有效设备地址范围
            是1-247。
****************************************************************************/
#if(MB_SET_ADDR)
void Modbus_SetAddr(unsigned char Addr)
{
   SlaveAddr=Addr;
}
#endif
/****************************************************************************
* 名    称：Modbus_GetAddr()
* 功    能：获取本机的设备地址
* 入口参数：无
* 出口参数：本机的设备地址
****************************************************************************/
#if(MB_GET_ADDR)
unsigned char Modbus_GetAddr()
{
   return(SlaveAddr);
}
void Send_Addr(void){

  SendByte_CRC(Modbus_GetAddr());   //返回错误代码
  UART_PutChar(CRCREG%256);
  UART_PutChar(CRCREG/256); 
}
#endif