/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stdio.h"
#include "stm32f10x_spi.h"
#include <stdlib.h>
#include <string.h>
#include "integer.h"

/* Defines -------------------------------------------------------------------*/
#define LED_DOTS            120     // Number of dots per LED string
#define DELAY_0             36      // Short delay timing
#define DELAY_1             36      // Long delay timing
#define LED_STRINGS         25      // Number of LED strings
#define BUFFER_SIZE         12020   // Data buffer size

/* Type definitions ----------------------------------------------------------*/
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

/* Macros --------------------------------------------------------------------*/
#define countof(a)   (sizeof(a) / sizeof(*(a)))

/* Global variables ----------------------------------------------------------*/
uint16_t count, count1, tmpb, tmpa, k;
uint8_t buffer[BUFFER_SIZE];
uint8_t a0, a1, a2, b, tmpc, dma_complete;

/* Function prototypes -------------------------------------------------------*/
void RCC_Configuration(void);
void NVIC_Configuration(void);
void Delay(__IO uint32_t nCount);
void Serial_Init(void);
void dma_init(void);
void calc_rgb(uint8_t bit, uint8_t dot, uint8_t col);
void send_rgb_data(uint8_t led_index, uint16_t control_bits);
void send_stop_signal(void);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief DMA transfer complete interrupt handler
 */
void DMA1_Channel4_IRQHandler(void) 
{
    if (DMA_GetFlagStatus(DMA1_IT_TC4) == SET) {
        // Signal DMA transfer complete
        dma_complete = 1;
        
        DMA_Cmd(DMA1_Channel4, DISABLE);
        GPIO_SetBits(GPIOB, GPIO_Pin_12);
        DMA_ClearITPendingBit(DMA1_IT_TC4);
    }
}

/**
 * @brief Initialize DMA for SPI2 reception
 */
void dma_init(void) 
{
    DMA_InitTypeDef dma_config;
    
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    
    dma_config.DMA_PeripheralBaseAddr = (uint32_t)(&(SPI2->DR));
    dma_config.DMA_MemoryBaseAddr = (uint32_t)buffer;
    dma_config.DMA_DIR = DMA_DIR_PeripheralSRC;
    dma_config.DMA_BufferSize = LED_DOTS * LED_STRINGS * 4 + 5;
    dma_config.DMA_M2M = DMA_M2M_Disable;
    dma_config.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma_config.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma_config.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma_config.DMA_Mode = DMA_Mode_Normal;
    dma_config.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma_config.DMA_Priority = DMA_Priority_Medium;
    DMA_Init(DMA1_Channel4, &dma_config);
    
    SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, ENABLE);
    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
    NVIC_EnableIRQ(DMA1_Channel4_IRQn);
}

/**
 * @brief Simple delay function
 * @param nCount: delay count
 */
void Delay(__IO uint32_t nCount)
{
    for(; nCount != 0; nCount--);
}

/**
 * @brief Configure system clocks
 */
void RCC_Configuration(void)
{
    SystemInit();
}

/**
 * @brief Calculate RGB values from received buffer
 * @param bit: bit position to extract
 * @param dot: dot index
 * @param col: column index
 */
void calc_rgb(uint8_t bit, uint8_t dot, uint8_t col)
{
    uint16_t rgb_values[9];
    uint16_t i, bit_mask;

    bit_mask = (1 << bit);

    // Extract first 9 bits (R + G MSB)
    for (i = 0; i < 9; i++) {
        rgb_values[i] = (buffer[4 + dot*4 + col + i*LED_DOTS*4] & bit_mask) >> bit;
    }
    tmpa = (rgb_values[0] | (rgb_values[1]<<1) | (rgb_values[2]<<2) | (rgb_values[3]<<3) | 
            (rgb_values[4]<<4) | (rgb_values[5]<<5) | (rgb_values[6]<<6) | (rgb_values[7]<<7) | 
            (rgb_values[8]<<8));
    tmpa = ~tmpa;

    // Extract next 8 bits (G LSBs)
    for (i = 9; i < 17; i++) {
        rgb_values[i-9] = (buffer[4 + dot*4 + col + i*LED_DOTS*4] & bit_mask) >> bit;
    }
    tmpc = (rgb_values[0] | (rgb_values[1]<<1) | (rgb_values[2]<<2) | (rgb_values[3]<<3) | 
            (rgb_values[4]<<4) | (rgb_values[5]<<5) | (rgb_values[6]<<6) | (rgb_values[7]<<7));
    tmpc = ~tmpc;

    // Extract last 8 bits (B)
    for (i = 17; i < 25; i++) {
        rgb_values[i-17] = (buffer[4 + dot*4 + col + i*LED_DOTS*4] & bit_mask) >> bit;
    }
    tmpb = (~((rgb_values[0] | (rgb_values[1]<<1) | (rgb_values[2]<<2) | (rgb_values[3]<<3) | 
               (rgb_values[4]<<4) | (rgb_values[5]<<5) | (rgb_values[6]<<6) | (rgb_values[7]<<7))) | 0x000);
}

/**
 * @brief Send RGB data to LED string
 * @param led_index: LED index in string
 * @param control_bits: control bits to OR with output
 */
void send_rgb_data(uint8_t led_index, uint16_t control_bits)
{
    uint8_t j, i, temp_val, brightness_bits;
    uint8_t bit_mask = 0xFF;
    uint8_t shift_val = 0x01;

    brightness_bits = (buffer[5] & 31);

    // Send 2 header bits
    for (j = 0; j < 2; j++) {
        GPIO_Write(GPIOA, 0x0);
        GPIO_Write(GPIOC, 0x0);
        GPIO_Write(GPIOB, 0x0000 | control_bits);
        Delay(DELAY_0);

        if ((bit_mask & shift_val) == 0) {
            GPIO_Write(GPIOA, 0x1FF);
            GPIO_Write(GPIOC, 0xFF);
            GPIO_Write(GPIOB, 0xFF | control_bits);
        } else {
            GPIO_Write(GPIOA, 0x0);
            GPIO_Write(GPIOC, 0x0);
            GPIO_Write(GPIOB, 0x0000 | control_bits);
            Delay(DELAY_1);
        }
        shift_val = shift_val << 1;
        GPIO_Write(GPIOA, 0x1FF);
        GPIO_Write(GPIOC, 0xFF);
        GPIO_Write(GPIOB, 0xFF | control_bits);
        Delay(22);
    }

    // Send LED index (8 bits)
    shift_val = 0x01;
    for (j = 0; j < 8; j++) {
        GPIO_Write(GPIOA, 0x0);
        GPIO_Write(GPIOC, 0x0);
        GPIO_Write(GPIOB, 0x0000 | control_bits);
        Delay(DELAY_0);

        if ((led_index & shift_val) == 0) {
            GPIO_Write(GPIOA, 0x1FF);
            GPIO_Write(GPIOC, 0xFF);
            GPIO_Write(GPIOB, 0xFF | control_bits);
        } else {
            GPIO_Write(GPIOA, 0x0);
            GPIO_Write(GPIOC, 0x0);
            GPIO_Write(GPIOB, 0x0000 | control_bits);
            Delay(DELAY_1);
        }
        shift_val = shift_val << 1;
        GPIO_Write(GPIOA, 0x1FF);
        GPIO_Write(GPIOC, 0xFF);
        GPIO_Write(GPIOB, 0xFF | control_bits);
        Delay(22);
    }

    // Send 1 separator bit
    temp_val = 0x00;
    shift_val = 0x01;
    for (j = 0; j < 1; j++) {
        GPIO_Write(GPIOA, 0x0);
        GPIO_Write(GPIOC, 0x0);
        GPIO_Write(GPIOB, 0x0000 | control_bits);
        Delay(DELAY_0);

        if ((temp_val & shift_val) == 0) {
            GPIO_Write(GPIOA, 0x1FF);
            GPIO_Write(GPIOC, 0xFF);
            GPIO_Write(GPIOB, 0xFF | control_bits);
        } else {
            GPIO_Write(GPIOA, 0x0);
            GPIO_Write(GPIOC, 0x0);
            GPIO_Write(GPIOB, 0x0000 | control_bits);
            Delay(DELAY_1);
        }
        shift_val = shift_val << 1;
        GPIO_Write(GPIOA, 0x1FF);
        GPIO_Write(GPIOC, 0xFF);
        GPIO_Write(GPIOB, 0xFF | control_bits);
        Delay(22);
    }

    // Send RGB data (3 bytes * 8 bits each)
    for (i = 2; i < 5; i++) {
        for (j = 0; j < 8; j++) {
            calc_rgb(j + brightness_bits, led_index - 1, i);
            
            GPIO_Write(GPIOA, 0x0);
            GPIO_Write(GPIOC, 0x0);
            GPIO_Write(GPIOB, 0x00 | control_bits);
            Delay(DELAY_0);
            
            GPIO_Write(GPIOA, tmpa);
            GPIO_Write(GPIOC, tmpc);
            GPIO_Write(GPIOB, tmpb | control_bits);
            Delay(DELAY_1);
            
            GPIO_Write(GPIOA, 0x1FF);
            GPIO_Write(GPIOC, 0xFF);
            GPIO_Write(GPIOB, 0xFF | control_bits);
            Delay(22);
        }
    }

    // Send end-of-data signal
    Delay(DELAY_0);
    GPIO_Write(GPIOA, 0x0);
    GPIO_Write(GPIOC, 0x0);
    GPIO_Write(GPIOB, 0x00 | control_bits);
    Delay(180);
    GPIO_Write(GPIOA, 0x1FF);
    GPIO_Write(GPIOC, 0xFF);
    GPIO_Write(GPIOB, 0xFF | control_bits);
    Delay(170);
}

/**
 * @brief Send stop signal to LED strings
 */
void send_stop_signal(void)
{
    GPIO_Write(GPIOA, 0x0);
    GPIO_Write(GPIOC, 0x0);
    GPIO_Write(GPIOB, 0x0000);
    Delay(180);
    
    GPIO_Write(GPIOA, 0x1FF);
    GPIO_Write(GPIOC, 0xFF);
    GPIO_Write(GPIOB, 0x00FF);
    Delay(280);
}

/**
 * @brief Main program
 */
int main(void)
{
    uint16_t led_index;

    RCC_Configuration();
    Serial_Init();
    
    // Initialize GPIO outputs high
    GPIO_Write(GPIOA, 0x1FF);
    GPIO_Write(GPIOC, 0xFF);
    GPIO_Write(GPIOB, 0x00FF);
    Delay(30);

    // Send initial test pattern
    for (led_index = 0; led_index < 255; led_index++) {
        send_rgb_data(led_index + 1, 0x0);
    }

    // Initialize DMA
    dma_complete = 0;
    dma_init();
    DMA_Cmd(DMA1_Channel4, ENABLE);
    GPIO_ResetBits(GPIOB, GPIO_Pin_12);
    count = 0;
    dma_complete = 0;

    // Main loop
    while (1) {
        send_stop_signal();
        
        if (dma_complete == 1) {
            // Send data to all LEDs in string
            for (led_index = 1; led_index <= LED_DOTS; led_index++) {
                send_rgb_data(led_index, 0x1000);
            }
            
            // Restart DMA for next frame
            dma_complete = 0;
            dma_init();
            DMA_Cmd(DMA1_Channel4, ENABLE);
            GPIO_ResetBits(GPIOB, GPIO_Pin_12);
            dma_complete = 0;
        }
    }
}

/**
 * @brief Initialize GPIO and SPI peripherals
 */
void Serial_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef SPI_InitStructure;

    // Enable peripheral clocks
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    
    // Disable JTAG to free up pins
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_NoJTRST, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);

    // Configure GPIOB pins 0-7 as outputs
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | 
                                  GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // Configure GPIOB pin 12 as output (control pin)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // Configure GPIOC pins 0-7 as outputs
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | 
                                  GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // Configure GPIOA pins 0-8 as outputs
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | 
                                  GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Configure SPI2
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
    
    // Configure SPI2 pins (PB13=SCK, PB15=MOSI as inputs)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // Configure SPI2 peripheral
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_RxOnly;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
    SPI_Init(SPI2, &SPI_InitStructure);

    SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);
    SPI_Cmd(SPI2, ENABLE);
}

/**
 * @brief Configure NVIC (currently empty)
 */
void NVIC_Configuration(void)
{
    // NVIC configuration if needed
}

#ifdef DEBUG
/**
 * @brief Assert failed handler for debugging
 */
void assert_failed(uint8_t* file, uint32_t line)
{
    // Add debug handling here if needed
}
#endif

#ifdef USE_FULL_ASSERT
/**
 * @brief Assert failed handler
 */
void assert_failed(uint8_t* file, uint32_t line)
{
    // Add assert handling here if needed
}
#endif