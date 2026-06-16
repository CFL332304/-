#ifndef __MYI2C_H__
#define __MYI2C_H__

#include "bspsystem.h"

/* Low-level software I2C primitives. */
void MyI2C_Start(void);
void MyI2C_Stop(void);
void MyI2C_SendByte(uint8_t Byte);
uint8_t MyI2C_ReceiveByte(void);
void MyI2C_SendAck(uint8_t AckBit);
uint8_t MyI2C_ReceiveAck(void);

/* MPU6050 register access helpers used by the attitude task and DMP library. */
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t MPU6050_ReadReg(uint8_t RegAddress);
void MPU6050_ReadRegs(uint8_t RegAddress, uint8_t *DataArray, uint8_t Count);
void MPU6050_Init(void);
int MPU6050_Read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);
int MPU6050_Write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
void MPU6050_GetData(int16_t *Accel_X, int16_t *Accel_Y, int16_t *Accel_Z,
                     int16_t *Gyro_X, int16_t *Gyro_Y, int16_t *Gyro_Z);

#endif
