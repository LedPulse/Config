/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
//#include "sdcard.h"
#include "stdio.h"
//#include "stm32f10x_usart.h"
#include "stm32f10x_spi.h"

#include <stdlib.h>
#include <string.h>

#include "integer.h"
//#include "ff.h"
//#include "diskio.h"
//#include "F1.h"
//# define l 
# define l1 96 //dots
# define del_0 30 //53 //36
# define del_1 30 //43 //36
/** @addtogroup StdPeriph_Examples
  * @{
  */

/** @addtogroup SDIO_Example
  * @{
  */
//#define SD_DETECT_PIN                    GPIO_Pin_14                 /* PB.14 */
//#define SD_DETECT_GPIO_PORT              GPIOB                       /* GPIOB */
//#define maxl3 180
/* Private typedef -----------------------------------------------------------*/
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

/* Private define ------------------------------------------------------------*/
//#define TxBufferSize1   (countof(TxBuffer1) - 1)
//#define RxBufferSize1   (countof(TxBuffer1) - 1)

/* Private macro -------------------------------------------------------------*/
#define countof(a)   (sizeof(a) / sizeof(*(a)))

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void NVIC_Configuration(void);
void Delay(__IO uint32_t nCount);
void Serial_Init(void);	
/* Private functions ---------------------------------------------------------*/
	uint16_t count,count1,tmpb,tmpa,tmpc,k;
	uint8_t buff[13000];
			uint8_t a0,a1,a2,b,dmaval;
	uint16_t pa[12]={0,1,2,4,5,6,7,8,11,12,14,15};
	uint16_t pb[8]={0,1,3,4,5,6,7,10};
	uint16_t pc[12]={0,1,2,3,4,5,6,7,8,10,11,12};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval : None
  */
/*
  void SPI2_IRQHandler(void)
  {
//    uint8_t ReceiveByte;//, SendByte;
 
if(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == SET) 
{
 buff[count] = SPI2->DR;
 if (count<2808) 
{
count++;
}
if (count >= 2808) {	  GPIO_SetBits(GPIOB, GPIO_Pin_12);//Delay(60);
}
 }
 }

*/
   void DMA1_Channel4_IRQHandler(void) {
    if (DMA_GetFlagStatus(DMA1_IT_TC4) == SET) {
 
      // ??????????, ??? ?????? ???????
   dmaval = 1; 
 
      DMA_Cmd(DMA1_Channel4, DISABLE);
			GPIO_SetBits(GPIOB, GPIO_Pin_12);
      DMA_ClearITPendingBit(DMA1_IT_TC4);
    }
  }

   void dma_init() {
    DMA_InitTypeDef dma;
     RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    dma.DMA_PeripheralBaseAddr = (uint32_t)(&(SPI2->DR));
    dma.DMA_MemoryBaseAddr = (uint32_t)buff;
    dma.DMA_DIR = DMA_DIR_PeripheralSRC;
    dma.DMA_BufferSize = l1*32*4+5;
    dma.DMA_M2M = DMA_M2M_Disable;
     dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
     dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
     dma.DMA_Mode = DMA_Mode_Normal;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_Priority = DMA_Priority_Medium;
    DMA_Init(DMA1_Channel4, &dma);
    SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, ENABLE);
    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
    NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  }
	
void Delay(__IO uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}

void RCC_Configuration(void)
{
  /* Setup the microcontroller system. Initialize the Embedded Flash Interface,
     initialize the PLL and update the SystemFrequency variable. */
  SystemInit();
}
void calc_rgb(uint8_t b,uint8_t dot,uint8_t col)
{
	uint16_t a[12];

	uint16_t i,tmp;

	tmp=(1<<b);

for (i=0;i<12;i++)
{
a[i]=(buff[4+dot*4+col+i*l1*4]&tmp)>>b;
}
tmpa=0;
for (i=0;i<12;i++)
{
	tmpa=tmpa|(a[i]<<pa[i]);
}
tmpa=~tmpa;

for (i=12;i<20;i++)
{
a[i-12]=(buff[4+dot*4+col+i*l1*4]&tmp)>>b;
}
tmpb=0;
for (i=0;i<8;i++)
{
	tmpb=tmpb|(a[i]<<pb[i]);
}
tmpb=~tmpb;


for (i=20;i<32;i++)
{
a[i-20]=(buff[4+dot*4+col+i*l1*4]&tmp)>>b;
}
tmpc=0;
for (i=0;i<12;i++)
{
	tmpc=tmpc|(a[i]<<pc[i]);
}
tmpc=~tmpc;


}


	void rgb(uint8_t n,uint16_t ss)
{
uint8_t j,i,tmp,m;
		uint8_t b5;
m=0xff;
tmp=0x1;	
	b5=(buff[5]&31);
for (j=0;j<2;j++)
	{
GPIO_Write( GPIOA, 0x0);
GPIO_Write( GPIOC, 0x0);
GPIO_Write( GPIOB, 0x0000|ss);
Delay(del_0);	
	if ((m&tmp)==0) 	
	{	
GPIO_Write( GPIOA, 0xFFFF);
GPIO_Write( GPIOC, 0xFFFF);
GPIO_Write( GPIOB, 0xFFFF|ss);
	}
	else 
	{
GPIO_Write( GPIOA, 0x0);
GPIO_Write( GPIOC, 0x0);
GPIO_Write( GPIOB, 0x0000|ss);
Delay(del_1);	
	}
	tmp=tmp<<1;
GPIO_Write( GPIOA, 0xFFFF);
GPIO_Write( GPIOC, 0xFFFF);
GPIO_Write( GPIOB, 0xFFFF|ss);
Delay(18);
	}
	


tmp=0x1;	
for (j=0;j<8;j++)
	{
GPIO_Write( GPIOA, 0x0);
GPIO_Write( GPIOC, 0x0);
GPIO_Write( GPIOB, 0x0000|ss);
Delay(del_0);	
	if ((n&tmp)==0) 	
	{	
GPIO_Write( GPIOA, 0xFFFF);
GPIO_Write( GPIOC, 0xFFFF);
GPIO_Write( GPIOB, 0xFFFF|ss);
	}
	else 
	{
GPIO_Write( GPIOA, 0x0);
GPIO_Write( GPIOC, 0x0);
GPIO_Write( GPIOB, 0x0000|ss);
Delay(del_1);	
	}
	tmp=tmp<<1;
GPIO_Write( GPIOA, 0xFFFF);
GPIO_Write( GPIOC, 0xFFFF);
GPIO_Write( GPIOB, 0xFFFF|ss);
Delay(18);
	}

	m=0x00;
tmp=0x1;	
for (j=0;j<1;j++)
	{
GPIO_Write( GPIOA, 0x0);
GPIO_Write( GPIOC, 0x0);
GPIO_Write( GPIOB, 0x0000|ss);
Delay(del_0);	
	if ((m&tmp)==0) 	
	{	
GPIO_Write( GPIOA, 0xFFFF);
GPIO_Write( GPIOC, 0xFFFF);
GPIO_Write( GPIOB, 0xFFFF|ss);
	}
	else 
	{
GPIO_Write( GPIOA, 0x0);
GPIO_Write( GPIOC, 0x0);
GPIO_Write( GPIOB, 0x0000|ss);
Delay(del_1);	
	}
	tmp=tmp<<1;
GPIO_Write( GPIOA, 0xFFFF);
GPIO_Write( GPIOC, 0xFFFF);
GPIO_Write( GPIOB, 0xFFFF|ss);
Delay(18);
	}



for (i=2;i<5;i++)
	{
for (j=0;j<8;j++)
	{
calc_rgb(j+b5,n-1,i);//1
GPIO_Write( GPIOA, 0x0);
GPIO_Write( GPIOC, 0x0);
GPIO_Write( GPIOB, 0x00|ss);
Delay(del_0);	
GPIO_Write( GPIOA, tmpa);
GPIO_Write( GPIOC, tmpc);
GPIO_Write( GPIOB, tmpb|ss);
Delay(del_1);	
GPIO_Write( GPIOA, 0xFFFF);
GPIO_Write( GPIOC, 0xFFFF);
GPIO_Write( GPIOB, 0xFFFF|ss);
Delay(18);
	}
}
Delay(del_0);	
GPIO_Write( GPIOA, 0x0);
GPIO_Write( GPIOC, 0x0);
GPIO_Write( GPIOB, 0x00|ss);
Delay(80);		//210
GPIO_Write( GPIOA, 0xFFFF);
GPIO_Write( GPIOC, 0xFFFF);
GPIO_Write( GPIOB, 0xFFFF|ss);
Delay(80);	//170
	///////////////////////////g
	///////////////////////////

}

/*
	void rgb(uint8_t n)
	{
uint8_t j,t;
uint8_t  tmp;	
GPIO_SetBits(GPIOB, GPIO_Pin_0);
Delay(260);			
GPIO_ResetBits(GPIOB, GPIO_Pin_0);
Delay(260);		
GPIO_SetBits(GPIOB, GPIO_Pin_0);
Delay(260);	
	
	tmp=add0[n];
for (j=0;j<4;j++)
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_0);
//GPIO_Write( GPIOB, 0x1000);
Delay(del_0);
		if ((tmp&0x1)==1)
		{	
Delay(del_0);
		}
//GPIO_Write( GPIOB, 0x10FF);
				GPIO_SetBits(GPIOB, GPIO_Pin_0);

Delay(30);
	tmp=tmp>>1;	
	}
	///////////////////////////
	tmp=add1[n];
for (j=0;j<8;j++)
	{
//GPIO_Write( GPIOB, 0x1000);
				GPIO_ResetBits(GPIOB, GPIO_Pin_0);

Delay(del_0);
		if ((tmp&0x1)==1)
		{	
Delay(del_0);
		}
				GPIO_SetBits(GPIOB, GPIO_Pin_0);

//GPIO_Write( GPIOB, 0x10FF);
Delay(30);
	tmp=tmp>>1;	
	}
	///////////////////////////
		tmp=add2[n];
for (j=0;j<8;j++)
	{
				GPIO_ResetBits(GPIOB, GPIO_Pin_0);

//GPIO_Write( GPIOB, 0x1000);
Delay(del_0);
		if ((tmp&0x1)==1)
		{	
Delay(del_0);
		}
				GPIO_SetBits(GPIOB, GPIO_Pin_0);

//GPIO_Write( GPIOB, 0x10FF);
Delay(30);
	tmp=tmp>>1;	
	}
	///////////////////////////
	tmp=buff[n*4+4+1];
for (j=0;j<8;j++)
	{
//GPIO_Write( GPIOB, 0x1000);
				GPIO_ResetBits(GPIOB, GPIO_Pin_0);

Delay(del_0);
		if ((tmp&0x1)==1)
		{	
Delay(del_0);
		}
				GPIO_SetBits(GPIOB, GPIO_Pin_0);

//GPIO_Write( GPIOB, 0x10FF);
Delay(30);
	tmp=tmp>>1;	
	}
	///////////////////////////
	///////////////////////////
	tmp=buff[n*4+4+2];
for (j=0;j<8;j++)
	{
//GPIO_Write( GPIOB, 0x1000);
				GPIO_ResetBits(GPIOB, GPIO_Pin_0);

Delay(del_0);
		if ((tmp&0x1)==1)
		{	
Delay(del_0);
		}
//GPIO_Write( GPIOB, 0x10FF);
				GPIO_SetBits(GPIOB, GPIO_Pin_0);

Delay(30);
	tmp=tmp>>1;	
	}
	///////////////////////////
	///////////////////////////
	tmp=buff[n*4+4+3];
for (j=0;j<8;j++)
	{
//GPIO_Write( GPIOB, 0x1000);
				GPIO_ResetBits(GPIOB, GPIO_Pin_0);

Delay(del_0);
		if ((tmp&0x1)==1)
		{	
Delay(del_0);
		}
//GPIO_Write( GPIOB, 0x10FF);
				GPIO_SetBits(GPIOB, GPIO_Pin_0);

Delay(30);
	tmp=tmp>>1;	
	}
	///////////////////////////

//	GPIO_Write( GPIOA, 0x0);
//GPIO_Write( GPIOC, 0x0);
//GPIO_Write( GPIOB, 0x1000);
			GPIO_ResetBits(GPIOB, GPIO_Pin_0);

Delay(260);		
//GPIO_Write( GPIOA, 0x1FF);
//GPIO_Write( GPIOC, 0xFF);
//GPIO_Write( GPIOB, 0x10FF);
			GPIO_SetBits(GPIOB, GPIO_Pin_0);

Delay(260);	

	
	}
*/

void stop40()
{
GPIO_Write( GPIOA, 0x0);
GPIO_Write( GPIOC, 0x0);
GPIO_Write( GPIOB, 0x0000);
	//		GPIO_ResetBits(GPIOB, GPIO_Pin_0);

Delay(180);		
GPIO_Write( GPIOA, 0xFFFF);
GPIO_Write( GPIOC, 0xFFFF);
GPIO_Write( GPIOB, 0xFFF);
		//	GPIO_SetBits(GPIOB, GPIO_Pin_0);

Delay(280);	

}
////////////////////


////////////////////


////////////////////



/*******************************************************************************
* Function Name  : main
* Description    : Main program.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int main(void)
{

 // uint32_t t,temp;
	uint16_t j;//,k,r,g,b,tmp;
	//char s[10];

  RCC_Configuration();
  Serial_Init();	
	GPIO_Write( GPIOA, 0x1FF);
GPIO_Write( GPIOC, 0xFF);
GPIO_Write( GPIOB, 0x00FF);

	Delay(30);
		  for (j=0;j<255;j++) 
  {
		rgb(j+1,0x0);
  }

  dmaval=0;
	 dma_init();
  DMA_Cmd(DMA1_Channel4, ENABLE);
	GPIO_ResetBits(GPIOB, GPIO_Pin_12); 

//GPIO_Write( GPIOB, 0xFF);
count=0;
dmaval=0;
	/*
	for(k=0;k<7000;k++)
	{buff[k]=0;}
		  for (j=1;j<=60;j++) 
  {
		rgb(j,0x1000);
  }
*/
	/////////////////////////////////////////////
	while (1)
      {
	/*		
	stop40();	
  if (count==2808 ) //4004  
  {   
	  for (j=0;j<l1;j++) 
  {
		rgb(j,0x1000);
  }
 	SPI_I2S_ClearFlag(SPI2, SPI_I2S_FLAG_RXNE);
	GPIO_ResetBits(GPIOB, GPIO_Pin_12); count=0;
	}
  //}
		*/	
//////////////////////////////////////////////////////////////////////////////////////////////////

	////////////
	
		stop40();	
  if (dmaval==1 ) //4004  
  {   
	  for (j=1;j<=l1;j++) 
  {
		rgb(j,0x1000);
  }
	/*
  for (j=0;j<l1;j++) 
  {
		rgb(j,0x1000);
  }
	*/
   dmaval=0;
	 dma_init();
  DMA_Cmd(DMA1_Channel4, ENABLE);
	GPIO_ResetBits(GPIOB, GPIO_Pin_12); 
	 dmaval=0;
	}
	

	
	
}
}

/*************************************************************************
 * Function Name: Serial_Init
 * Description: Init USARTs
 *************************************************************************/
void Serial_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
		 SPI_InitTypeDef SPI_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOB , ENABLE);

RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
GPIO_PinRemapConfig(GPIO_Remap_SWJ_NoJTRST, ENABLE);
GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
	
	

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_7|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_12|GPIO_Pin_11|GPIO_Pin_14|GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);


  ///////////////////////////////////////////////////////spi2 
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
 
 /*
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
 */
 /*
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
 */
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_RxOnly;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
  SPI_Init(SPI2, &SPI_InitStructure);
 
 
 SPI_I2S_ITConfig(SPI2,SPI_I2S_IT_RXNE, ENABLE);
 // NVIC_EnableIRQ(SPI2_IRQn);

 
  SPI_Cmd(SPI2, ENABLE);

	
  
}


/*******************************************************************************
* Function Name  : NVIC_Config
* Description    : Configures SDIO IRQ channel.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{
	/*
  NVIC_InitTypeDef NVIC_InitStructure;

   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

  NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	*/
}

/* Implementation of putchar (also used by printf function to output data)    */

//USART1_IRQHandler

/*******************************************************************************
* Function Name  : fputc
* Description    : Retargets the C library printf function to the USART.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

/*******************************************************************************
* Function Name  : USART_Scanf
* Description    : Gets numeric values from the hyperterminal.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

#ifdef  DEBUG
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(u8* file, u32 line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
 // while (1)
  {}
}
#endif



#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
 // while (1)
  {}
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
