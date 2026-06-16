#ifndef __MYUSART_H__
#define __MYUSART_H__


#include "bspsystem.h"


//需要发送16位,32位数据，对数据拆分，之后每次发送单个字节
//拆分过程：对变量dwTemp 去地址然后将其转化成char类型指针，最后再取出指针所指向的内容
#define BYTE0(dwTemp)  (*(char *)(&dwTemp))
#define BYTE1(dwTemp)  (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)  (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)  (*((char *)(&dwTemp) + 3))

void ANODT_SEND_F2(int16_t _a, int16_t _b, int16_t _c, int16_t _d);


// #define BUFFER_SIZE 1024           /* 环形缓冲区大小 根据实际的数据大小进行调整 */ 
// /* 定义环形缓冲区结构体 */ 
// typedef struct {
//     uint8_t  buffer[BUFFER_SIZE];  /* 缓冲区   */ 
//     volatile uint32_t head;    /* 写入指针 */ 
//     volatile uint32_t tail;    /* 读取指针 */ 
//     SemaphoreHandle_t mutex;   /* 互斥锁，保证线程安全 */ 
// } ringbuffer_t;

// void RingBuffer_Init(ringbuffer_t *rb);
// int32_t RingBuffer_IsEmpty(ringbuffer_t *rb);
// int32_t RingBuffer_IsFull(ringbuffer_t *rb);
// int32_t RingBuffer_Write(ringbuffer_t *rb, char data);
// int32_t RingBuffer_Read(ringbuffer_t *rb, char *data);
// void RingBufferWriteFormatted(ringbuffer_t* rb, const char* format, ...);
// void myprintf(const char *fmt, ...);
// static void debugPrintUART(const char *pData);

#endif
