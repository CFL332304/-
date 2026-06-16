#ifndef __BSPSYSTEM_H__
#define __BSPSYSTEM_H__

#include "main.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "event_groups.h"
#include "cmsis_os.h"
#include "tim.h"
#include "usart.h"
#include "math.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "stdarg.h"

#include "oled.h"
#include "user.h"
#include "MyI2C.h"
#include "MyFunc.h"
#include "MyUsart.h"
#include "MPU6050_DMP.h"
#include "nrf24l01.h"

#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1A
#define MPU6050_GYRO_CONFIG  0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40
#define MPU6050_TEMP_OUT_H   0x41
#define MPU6050_TEMP_OUT_L   0x42
#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_GYRO_XOUT_L  0x44
#define MPU6050_GYRO_YOUT_H  0x45
#define MPU6050_GYRO_YOUT_L  0x46
#define MPU6050_GYRO_ZOUT_H  0x47
#define MPU6050_GYRO_ZOUT_L  0x48
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_PWR_MGMT_2   0x6C
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_ADDRESS      0xD0

extern uint8_t RDataBuff[100];
extern char g_SerialRdataPacket[100];
extern QueueHandle_t SerialRDataQueue;
extern QueueHandle_t MPU6050_Queue;
extern QueueHandle_t SerialPrintDataQueue;
extern QueueHandle_t ControlAngleQueue;
extern QueueHandle_t OledVelocityQueue;
extern QueueHandle_t ControlVelocityQueue;
extern QueueHandle_t RemoteControlQueue;
extern QueueHandle_t RemoteDisplayQueue;
extern SemaphoreHandle_t xPrintDataMutex;

typedef struct{
    uint8_t Duty;
    uint8_t polarity;
}Velocity_PWM;

struct VelocityStructure {
    float A;
    float B;
    uint32_t timestamp;
};

typedef struct{
    int16_t Ax;
    int16_t Ay;
    int16_t Az;
    int16_t Gx;
    int16_t Gy;
    int16_t Gz;
}MPU6050_Data;

typedef struct __FILE FILE;

#endif
