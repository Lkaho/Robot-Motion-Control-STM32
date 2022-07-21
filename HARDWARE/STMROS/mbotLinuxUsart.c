#include "mbotLinuxUsart.h"
#include "usart.h"         //����printf

/*--------------------------------����Э��-----------------------------------
//----------------55 aa size 00 00 00 00 00 crc8 0d 0a----------------------
//����ͷ55aa + �����ֽ���size + ���ݣ����ù����壩 + У��crc8 + ����β0d0a
//ע�⣺����������Ԥ����һ���ֽڵĿ���λ�������Ŀ���������չ������size������
--------------------------------------------------------------------------*/

/*--------------------------------����Э��-----------------------------------
//----------------55 aa size 00 00 00 00 00 crc8 0d 0a----------------------
//����ͷ55aa + �����ֽ���size + ���ݣ����ù����壩 + У��crc8 + ����β0d0a
//ע�⣺����������Ԥ����һ���ֽڵĿ���λ�������Ŀ���������չ������size������
--------------------------------------------------------------------------*/


/**************************************************************************
ͨ�ŵķ��ͺ����ͽ��պ��������һЩ���������������������
**************************************************************************/

//���ݽ����ݴ���
static unsigned char  receiveBuff[16] = {0};         
//ͨ��Э�鳣��
const unsigned char header[2]  = {0x55, 0xaa};
const unsigned char ender[2]   = {0x0d, 0x0a};


//�������ݣ��ٶȡ����ٶȡ���ڽǣ���-32767 - +32768��
union sendData
{
	float d;
	unsigned char data[4];
}SpeedNow, AccxNow, AccyNow, Yaw_rateNow, YawNow;

//���š��ƶ���ǰ����ת��
union receiveData
{
	short d;
	unsigned char data[2];
}throttleSet, BrakeSet, FrontAngSet, RearAngSet;

/**************************************************************************
�������ܣ�ͨ�������жϷ���������ȡ��λ�����͵Ŀ����źţ�Ԥ�����Ʊ�־λ���ֱ���������
��ڲ����������źš��ƶ��źš�ǰ����ת��
����  ֵ������������
**************************************************************************/
int usartReceiveOneData(int *p_thrSet,int *p_brkSet, int *p_FrontAngSet, int *p_RearAngSet,unsigned char *p_crtlFlag)
{
	unsigned char USART_Receiver              = 0;          //��������
	static unsigned char checkSum             = 0;
	static unsigned char USARTBufferIndex     = 0;
	static short j=0,k=0;
	static unsigned char USARTReceiverFront   = 0;
	static unsigned char Start_Flag           = START;      //һ֡���ݴ��Ϳ�ʼ��־λ, ��Ϣͷ���������ȡ��
	static short dataLength                   = 0;

	USART_Receiver = USART_ReceiveData(USART1);  
	//������Ϣͷ
	if(Start_Flag == START)
	{
		if(USART_Receiver == 0xaa)                             //buf[1]
		{  
			if(USARTReceiverFront == 0x55)        //����ͷ��λ //buf[0]
			{
				Start_Flag = !START;              //�յ�����ͷ����ʼ��������
				//printf("header ok\n");
				receiveBuff[0]=header[0];         //buf[0]
				receiveBuff[1]=header[1];         //buf[1]
				USARTBufferIndex = 0;             //��������ʼ��
				checkSum = 0x00;				  //У��ͳ�ʼ��
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
			case 0://��������֡����
				receiveBuff[2] = USART_Receiver;
				dataLength     = receiveBuff[2];            //buf[2]
				USARTBufferIndex++;
				break;
			case 1://�����������ݣ�����ֵ���� 
				receiveBuff[j + 3] = USART_Receiver;        //buf[3] - buf[10]					
				j++;
				if(j >= dataLength)                         
				{
					j = 0;
					USARTBufferIndex++;
				}
				break;
			case 2://����У��ֵ��Ϣ
				receiveBuff[3 + dataLength] = USART_Receiver;
				checkSum = getCrc8(receiveBuff, 3 + dataLength);
				  // �����ϢУ��ֵ
				if (checkSum != receiveBuff[3 + dataLength]) //buf[8]
				{
					printf("Received data check sum error!");
					return 0;
				}
				USARTBufferIndex++;
				break;
				
			case 3://������Ϣβ
				if(k==0)
				{
					//����0d     buf[9]  �����ж�
					k++;
				}
				else if (k==1)
				{
					//����0a     buf[10] �����ж�

					//���п�������ֵ����				
					 for(k = 0; k < 2; k++)
					{
						throttleSet.data[k]  = receiveBuff[k + 3]; //buf[3]  buf[4]
						BrakeSet.data[k] = receiveBuff[k + 5]; //buf[5]  buf[6]
						FrontAngSet.data[k] = receiveBuff[k + 7]; //buf[7] buf[8]
						RearAngSet.data[k] = receiveBuff[k + 9]; //buf[9] buf[10]
					}				
					
					//�����źŸ�ֵ����
					*p_thrSet  = (int)throttleSet.d;
					*p_brkSet = (int)BrakeSet.d;
					*p_FrontAngSet = (int)FrontAngSet.d;
					*p_RearAngSet = (int)RearAngSet.d;
					
					//ctrlFlag
					*p_crtlFlag = receiveBuff[11];                //buf[11]
					
					//-----------------------------------------------------------------
					//���һ�����ݰ��Ľ��գ���ر������㣬�ȴ���һ�ֽ�����
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
�������ܣ���ʵʱ���١���������ٶȡ����ٶȡ�ƫ�������ݽ��д����ͨ�����ڷ��͸�Linux
��ڲ�����ʵʱ���١���������ٶȡ����ٶȡ�ƫ����
����  ֵ����
**************************************************************************/
void usartSendData(float Speed_ac, float Accx_ac, float Accy_ac, float Yaw_rateac, float Yaw_ac, unsigned char ReceFlag)
{
	// Э�����ݻ�������
	unsigned char buf[27] = {0};
	int i, length = 0;

	// ����ֵ��ֵ
	SpeedNow.d  = Speed_ac;
	AccxNow.d = Accx_ac;
	AccyNow.d = Accy_ac;
	Yaw_rateNow.d = Yaw_rateac;
	YawNow.d = Yaw_ac;
	// ������Ϣͷ
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
	// Ԥ������ָ��
	buf[3 + length - 1] = ReceFlag;              // buf[23]
	
	// ����У��ֵ����Ϣβ
	buf[3 + length] = getCrc8(buf, 3 + length);  // buf[24]
	buf[3 + length + 1] = ender[0];              // buf[25]
	buf[3 + length + 2] = ender[1];              // buf[26]
	
	//�����ַ�������
	USART_Send_String(buf,sizeof(buf));
}
/**************************************************************************
�������ܣ�����ָ����С���ַ����飬��usartSendData��������
��ڲ����������ַ�������С
����  ֵ����
**************************************************************************/
void USART_Send_String(u8 *p,u16 sendSize)
{ 
	static int length =0;
	while(length<sendSize)
	{   
		while( !(USART1->SR&(0x01<<7)) );//���ͻ�����Ϊ��
		USART1->DR=*p;                   
		p++;
		length++;
	}
	length =0;
}
/**************************************************************************
�������ܣ������λѭ������У�飬��usartSendData��usartReceiveOneData��������
��ڲ����������ַ�������С
����  ֵ����
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






