#include "mbotLinuxUsart.h"
#include "usart.h"         //包含printf

/*--------------------------------发送协议-----------------------------------
//----------------55 aa size 00 00 00 00 00 crc8 0d 0a----------------------
//数据头55aa + 数据字节数size + 数据（利用共用体） + 校验crc8 + 数据尾0d0a
//注意：这里数据中预留了一个字节的控制位，其他的可以自行扩展，更改size和数据
--------------------------------------------------------------------------*/

/*--------------------------------接收协议-----------------------------------
//----------------55 aa size 00 00 00 00 00 crc8 0d 0a----------------------
//数据头55aa + 数据字节数size + 数据（利用共用体） + 校验crc8 + 数据尾0d0a
//注意：这里数据中预留了一个字节的控制位，其他的可以自行扩展，更改size和数据
--------------------------------------------------------------------------*/


/**************************************************************************
通信的发送函数和接收函数必须的一些常量、变量、共用体对象
**************************************************************************/

//数据接收暂存区
static unsigned char  receiveBuff[16] = {0};         
//通信协议常量
const unsigned char header[2]  = {0x55, 0xaa};
const unsigned char ender[2]   = {0x0d, 0x0a};


//发送数据（速度、加速度、横摆角）（-32767 - +32768）
union sendData
{
	float d;
	unsigned char data[4];
}SpeedNow, AccxNow, AccyNow, Yaw_rateNow, YawNow;

//油门、制动、前后轮转角
union receiveData
{
	short d;
	unsigned char data[2];
}throttleSet, BrakeSet, FrontAngSet, RearAngSet;

/**************************************************************************
函数功能：通过串口中断服务函数，获取上位机发送的控制信号，预留控制标志位，分别存入参数中
入口参数：油门信号、制动信号、前后轮转角
返回  值：无特殊意义
**************************************************************************/
int usartReceiveOneData(int *p_thrSet,int *p_brkSet, int *p_FrontAngSet, int *p_RearAngSet,unsigned char *p_crtlFlag)
{
	unsigned char USART_Receiver              = 0;          //接收数据
	static unsigned char checkSum             = 0;
	static unsigned char USARTBufferIndex     = 0;
	static short j=0,k=0;
	static unsigned char USARTReceiverFront   = 0;
	static unsigned char Start_Flag           = START;      //一帧数据传送开始标志位, 消息头接受完成则取反
	static short dataLength                   = 0;

	USART_Receiver = USART_ReceiveData(USART1);  
	//接收消息头
	if(Start_Flag == START)
	{
		if(USART_Receiver == 0xaa)                             //buf[1]
		{  
			if(USARTReceiverFront == 0x55)        //数据头两位 //buf[0]
			{
				Start_Flag = !START;              //收到数据头，开始接收数据
				//printf("header ok\n");
				receiveBuff[0]=header[0];         //buf[0]
				receiveBuff[1]=header[1];         //buf[1]
				USARTBufferIndex = 0;             //缓冲区初始化
				checkSum = 0x00;				  //校验和初始化
			}
		}
		else 
		{
			USARTReceiverFront = USART_Receiver;  
		}
	}
	else
    { 
		switch(USARTBufferIndex)
		{
			case 0://接收数据帧长度
				receiveBuff[2] = USART_Receiver;
				dataLength     = receiveBuff[2];            //buf[2]
				USARTBufferIndex++;
				break;
			case 1://接收所有数据，并赋值处理 
				receiveBuff[j + 3] = USART_Receiver;        //buf[3] - buf[10]					
				j++;
				if(j >= dataLength)                         
				{
					j = 0;
					USARTBufferIndex++;
				}
				break;
			case 2://接收校验值信息
				receiveBuff[3 + dataLength] = USART_Receiver;
				checkSum = getCrc8(receiveBuff, 3 + dataLength);
				  // 检查信息校验值
				if (checkSum != receiveBuff[3 + dataLength]) //buf[8]
				{
					printf("Received data check sum error!");
					return 0;
				}
				USARTBufferIndex++;
				break;
				
			case 3://接收信息尾
				if(k==0)
				{
					//数据0d     buf[9]  无需判断
					k++;
				}
				else if (k==1)
				{
					//数据0a     buf[10] 无需判断

					//进行控制量赋值操作				
					 for(k = 0; k < 2; k++)
					{
						throttleSet.data[k]  = receiveBuff[k + 3]; //buf[3]  buf[4]
						BrakeSet.data[k] = receiveBuff[k + 5]; //buf[5]  buf[6]
						FrontAngSet.data[k] = receiveBuff[k + 7]; //buf[7] buf[8]
						RearAngSet.data[k] = receiveBuff[k + 9]; //buf[9] buf[10]
					}				
					
					//控制信号赋值操作
					*p_thrSet  = (int)throttleSet.d;
					*p_brkSet = (int)BrakeSet.d;
					*p_FrontAngSet = (int)FrontAngSet.d;
					*p_RearAngSet = (int)RearAngSet.d;
					
					//ctrlFlag
					*p_crtlFlag = receiveBuff[11];                //buf[11]
					
					//-----------------------------------------------------------------
					//完成一个数据包的接收，相关变量清零，等待下一字节数据
					USARTBufferIndex   = 0;
					USARTReceiverFront = 0;
					Start_Flag         = START;
					checkSum           = 0;
					dataLength         = 0;
					j = 0;
					k = 0;
					//-----------------------------------------------------------------					
				}
				break;
			 default:break;
		}		
	}
	return 0;
}
/**************************************************************************
函数功能：将实时车速、横纵向加速度、角速度、偏航角数据进行打包，通过串口发送给Linux
入口参数：实时车速、横纵向加速度、角速度、偏航角
返回  值：无
**************************************************************************/
void usartSendData(float Speed_ac, float Accx_ac, float Accy_ac, float Yaw_rateac, float Yaw_ac, unsigned char ReceFlag)
{
	// 协议数据缓存数组
	unsigned char buf[27] = {0};
	int i, length = 0;

	// 测量值赋值
	SpeedNow.d  = Speed_ac;
	AccxNow.d = Accx_ac;
	AccyNow.d = Accy_ac;
	Yaw_rateNow.d = Yaw_rateac;
	YawNow.d = Yaw_ac;
	// 设置消息头
	for(i = 0; i < 2; i++)
		buf[i] = header[i];                      // buf[0] buf[1] 

	length = 21;
	buf[2] = length;                             // buf[2]
	for(i = 0; i < 4; i++)
	{
		buf[i + 3] = SpeedNow.data[i]; // buf[3] buf[4] buf[5] buf[6] 
		buf[i + 7] = AccxNow.data[i];  // buf[7] buf[8] buf[9] buf[10] 
		buf[i + 11] = AccyNow.data[i]; // buf[11] buf[12] buf[13] buf[14] 
		buf[i + 15] = Yaw_rateNow.data[i]; // buf[15] buf[16] buf[17] buf[18] 
	  buf[i + 19] = YawNow.data[i]; // buf[19] buf[20] buf[21] buf[22] 
	}
	// 预留控制指令
	buf[3 + length - 1] = ReceFlag;              // buf[23]
	
	// 设置校验值、消息尾
	buf[3 + length] = getCrc8(buf, 3 + length);  // buf[24]
	buf[3 + length + 1] = ender[0];              // buf[25]
	buf[3 + length + 2] = ender[1];              // buf[26]
	
	//发送字符串数据
	USART_Send_String(buf,sizeof(buf));
}
/**************************************************************************
函数功能：发送指定大小的字符数组，被usartSendData函数调用
入口参数：数组地址、数组大小
返回  值：无
**************************************************************************/
void USART_Send_String(u8 *p,u16 sendSize)
{ 
	static int length =0;
	while(length<sendSize)
	{   
		while( !(USART1->SR&(0x01<<7)) );//发送缓冲区为空
		USART1->DR=*p;                   
		p++;
		length++;
	}
	length =0;
}
/**************************************************************************
函数功能：计算八位循环冗余校验，被usartSendData和usartReceiveOneData函数调用
入口参数：数组地址、数组大小
返回  值：无
**************************************************************************/
unsigned char getCrc8(unsigned char *ptr, unsigned short len)
{
	unsigned char crc;
	unsigned char i;
	crc = 0;
	while(len--)
	{
		crc ^= *ptr++;
		for(i = 0; i < 8; i++)
		{
			if(crc&0x01)
                crc=(crc>>1)^0x8C;
			else 
                crc >>= 1;
		}
	}
	return crc;
}
/**********************************END***************************************/






