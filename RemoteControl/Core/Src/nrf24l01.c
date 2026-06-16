#include "nrf24l01.h"

#define NRF24L01_W_CE_L HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_RESET)    //NRF24L01发送与接收使能
#define NRF24L01_W_CE_H HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_SET)

#define NRF24L01_W_CS_L HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET)   //片选
#define NRF24L01_W_CS_H HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET)

#define NRF24L01_W_SCK_L HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_RESET)
#define NRF24L01_W_SCK_H HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_SET)

#define NRF24L01_W_MOSI_L HAL_GPIO_WritePin(MOSI_GPIO_Port, MOSI_Pin, GPIO_PIN_RESET)
#define NRF24L01_W_MOSI_H HAL_GPIO_WritePin(MOSI_GPIO_Port, MOSI_Pin, GPIO_PIN_SET)

uint8_t NRF24L01_R_MISO(void)
{
    return HAL_GPIO_ReadPin(MISO_GPIO_Port, MISO_Pin);
}

uint8_t NRF24L01_SWAP_Byte(uint8_t byte)
{
    for (uint8_t i = 0; i < 8; i++)
    {
       if(byte & 0x80 == 0x80 ) NRF24L01_W_MOSI_H;
       else NRF24L01_W_MOSI_L;
       NRF24L01_W_SCK_H;
       byte <<= 1;

       if(NRF24L01_R_MISO()) byte |= 0x01;
       NRF24L01_W_SCK_L;
    }
    return byte;
}
