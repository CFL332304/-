#include "MyUsart.h"

/* USART helper functions are used for debug output and ANO host plotting. */
/**
* @brief 重定向printf (重定向fputc)，
					使用时候记得勾选上魔法棒->Target->UseMicro LIB 
					可能需要在C文件加typedef struct __FILE FILE;
					包含这个文件#include "stdio.h"
* @param 
* @return 
*/
int fputc(int ch,FILE *stream)
{
	HAL_UART_Transmit(&huart1, ( uint8_t *)&ch, 1,0xFFFF);
	return ch;
}



/* Frame buffer for ANO protocol output. */
uint8_t data_to_send[100];
void ANODT_SEND_F2(int16_t _a, int16_t _b, int16_t _c, int16_t _d)
{
    uint8_t _cnt = 0;
    /* ANO protocol uses two checksums: sumcheck and accumulated addcheck. */
    uint8_t sumcheck = 0; //和校验
    uint8_t addcheck = 0; //附加和校验
    uint8_t i=0;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xFF;
    data_to_send[_cnt++] = 0xF2;
    /* Payload is four int16_t values, so the data length is 8 bytes. */
    data_to_send[_cnt++] = 8; //数据长度
	//单片机为小端模式-低地址存放低位数据，匿名上位机要求先发低位数据，所以先发低地址
    data_to_send[_cnt++] = BYTE0(_a);
    data_to_send[_cnt++] = BYTE1(_a);
	
    data_to_send[_cnt++] = BYTE0(_b);
    data_to_send[_cnt++] = BYTE1(_b);
	
    data_to_send[_cnt++] = BYTE0(_c);
    data_to_send[_cnt++] = BYTE1(_c);
	
    data_to_send[_cnt++] = BYTE0(_d);
    data_to_send[_cnt++] = BYTE1(_d);
	
	  for ( i = 0; i < data_to_send[3]+4; i++)
    {
        sumcheck += data_to_send[i];
        addcheck += sumcheck;
    }

    data_to_send[_cnt++] = sumcheck;
    data_to_send[_cnt++] = addcheck;
	/* Send the assembled ANO frame through USART3. */
	
	HAL_UART_Transmit(&huart3,data_to_send,_cnt,0xFFFF);//这里是串口发送函数
}
