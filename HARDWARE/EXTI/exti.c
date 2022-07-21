#include "exti.h"
#include "led.h"
#include "key.h"
#include "delay.h"
#include "usart.h"
#include "dir_gpio.h"
#include "stepdriver.h"
extern int brk_ang;
//Íâ²¿ÖĞ¶Ï0·şÎñ³ÌĞò
void EXTIX_Init(void)
{
 
 	EXTI_InitTypeDef EXTI_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;

    KEY_Init();	 //	°´¼ü¶Ë¿Ú³õÊ¼»¯

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	//Ê¹ÄÜ¸´ÓÃ¹¦ÄÜÊ±ÖÓ

    //GPIOE.2 ÖĞ¶ÏÏßÒÔ¼°ÖĞ¶Ï³õÊ¼»¯ÅäÖÃ   ÏÂ½µÑØ´¥·¢
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource2);

  	EXTI_InitStructure.EXTI_Line=EXTI_Line2;	//KEY2
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);	 	//¸ù¾İEXTI_InitStructÖĞÖ¸¶¨µÄ²ÎÊı³õÊ¼»¯ÍâÉèEXTI¼Ä´æÆ÷

   //GPIOE.3	  ÖĞ¶ÏÏßÒÔ¼°ÖĞ¶Ï³õÊ¼»¯ÅäÖÃ ÏÂ½µÑØ´¥·¢ //KEY1
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource3);
  	EXTI_InitStructure.EXTI_Line=EXTI_Line3;
  	EXTI_Init(&EXTI_InitStructure);	  	//¸ù¾İEXTI_InitStructÖĞÖ¸¶¨µÄ²ÎÊı³õÊ¼»¯ÍâÉèEXTI¼Ä´æÆ÷

   //GPIOE.4	  ÖĞ¶ÏÏßÒÔ¼°ÖĞ¶Ï³õÊ¼»¯ÅäÖÃ  ÏÂ½µÑØ´¥·¢	//KEY0
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource4);
  	EXTI_InitStructure.EXTI_Line=EXTI_Line4;
  	EXTI_Init(&EXTI_InitStructure);	  	//¸ù¾İEXTI_InitStructÖĞÖ¸¶¨µÄ²ÎÊı³õÊ¼»¯ÍâÉèEXTI¼Ä´æÆ÷


   //GPIOA.0	  ÖĞ¶ÏÏßÒÔ¼°ÖĞ¶Ï³õÊ¼»¯ÅäÖÃ ÉÏÉıÑØ´¥·¢ PA0  WK_UP
 	  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource0); 

  	EXTI_InitStructure.EXTI_Line=EXTI_Line0;
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  	EXTI_Init(&EXTI_InitStructure);		//¸ù¾İEXTI_InitStructÖĞÖ¸¶¨µÄ²ÎÊı³õÊ¼»¯ÍâÉèEXTI¼Ä´æÆ÷


  	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;			//Ê¹ÄÜ°´¼üWK_UPËùÔÚµÄÍâ²¿ÖĞ¶ÏÍ¨µÀ
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//ÇÀÕ¼ÓÅÏÈ¼¶2£¬ 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;					//×ÓÓÅÏÈ¼¶3
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//Ê¹ÄÜÍâ²¿ÖĞ¶ÏÍ¨µÀ
  	NVIC_Init(&NVIC_InitStructure); 

    NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;			//Ê¹ÄÜ°´¼üKEY2ËùÔÚµÄÍâ²¿ÖĞ¶ÏÍ¨µÀ
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//ÇÀÕ¼ÓÅÏÈ¼¶2£¬ 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;					//×ÓÓÅÏÈ¼¶2
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//Ê¹ÄÜÍâ²¿ÖĞ¶ÏÍ¨µÀ
  	NVIC_Init(&NVIC_InitStructure);


  	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;			//Ê¹ÄÜ°´¼üKEY1ËùÔÚµÄÍâ²¿ÖĞ¶ÏÍ¨µÀ
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//ÇÀÕ¼ÓÅÏÈ¼¶2 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;					//×ÓÓÅÏÈ¼¶1 
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//Ê¹ÄÜÍâ²¿ÖĞ¶ÏÍ¨µÀ
  	NVIC_Init(&NVIC_InitStructure);  	  //¸ù¾İNVIC_InitStructÖĞÖ¸¶¨µÄ²ÎÊı³õÊ¼»¯ÍâÉèNVIC¼Ä´æÆ÷

	  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;			//Ê¹ÄÜ°´¼üKEY0ËùÔÚµÄÍâ²¿ÖĞ¶ÏÍ¨µÀ
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//ÇÀÕ¼ÓÅÏÈ¼¶2 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;					//×ÓÓÅÏÈ¼¶0 
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//Ê¹ÄÜÍâ²¿ÖĞ¶ÏÍ¨µÀ
  	NVIC_Init(&NVIC_InitStructure);  	  //¸ù¾İNVIC_InitStructÖĞÖ¸¶¨µÄ²ÎÊı³õÊ¼»¯ÍâÉèNVIC¼Ä´æÆ÷
 
}

//====================================Íâ²¿ÖĞ¶Ï0·şÎñ³ÌĞò=================================================
//ÈË¹¤/×Ô¶¯¼İÊ»Ä£Ê½ÇĞ»»£ºLED0ÁÁÎª×Ô¶¯¼İÊ»£¬LED0ÃğÎªÈË¹¤¼İÊ
void EXTI0_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
	  delay_ms(10);//Ïû¶¶
		 
		if(WK_UP)
		{		 
      LED0 = !LED0;
			brk_ang = 0;
		}
		EXTI_ClearITPendingBit(EXTI_Line0); //Çå³ıLINE0ÉÏµÄÖĞ¶Ï±êÖ¾Î»  
	}
}
//====================================Íâ²¿ÖĞ¶Ï3·şÎñ³ÌĞò=================================================
//Çı¶¯µç»úÍ£Ö¹×ª¶¯
void EXTI3_IRQHandler(void)
{
	delay_ms(10);//Ïû¶¶
	if(KEY1==0)	 //°´¼üKEY1
	{				 
		DIR_BackWard = 1;
		DIR_ForWard  = 1;
	}		 
	EXTI_ClearITPendingBit(EXTI_Line3);  //Çå³ıLINE3ÉÏµÄÖĞ¶Ï±êÖ¾Î»  
}
//====================================Íâ²¿ÖĞ¶Ï4·şÎñ³ÌĞò=================================================
//¿ØÖÆµç»úĞı×ª·½Ïò
void EXTI4_IRQHandler(void)
{
	delay_ms(10);//Ïû¶¶
	if(KEY0==0)	 //°´¼üKEY0
	{
		DIR_BackWard = !DIR_BackWard;
		DIR_ForWard  = !DIR_ForWard;
		LED1 = !LED1;//Õı×ª·´×ªÇĞ»»Ö¸Ê¾µÆ
	}		 
	EXTI_ClearITPendingBit(EXTI_Line4);  //Çå³ıLINE4ÉÏµÄÖĞ¶Ï±êÖ¾Î»  
}
 
