#include "MyI2C.h"

/* Software I2C bus for MPU6050. PB8 is SCL and PB9 is SDA. */
#define MYI2C_GPIO_PORT     GPIOB
#define MYI2C_SCL_PIN       GPIO_PIN_8
#define MYI2C_SDA_PIN       GPIO_PIN_9
#define MYI2C_DELAY_LOOPS   10U

static void MyI2C_Delay(void)
{
    uint8_t i;

    for (i = 0U; i < MYI2C_DELAY_LOOPS; i++)
    {
        __NOP();
    }
}

static void MyI2C_W_SCL(uint8_t bit)
{
    if (bit != 0U)
    {
        MYI2C_GPIO_PORT->BSRR = MYI2C_SCL_PIN;
    }
    else
    {
        MYI2C_GPIO_PORT->BRR = MYI2C_SCL_PIN;
    }
    MyI2C_Delay();
}

static void MyI2C_W_SDA(uint8_t bit)
{
    if (bit != 0U)
    {
        MYI2C_GPIO_PORT->BSRR = MYI2C_SDA_PIN;
    }
    else
    {
        MYI2C_GPIO_PORT->BRR = MYI2C_SDA_PIN;
    }
    MyI2C_Delay();
}

static uint8_t MyI2C_R_SDA(void)
{
    return ((MYI2C_GPIO_PORT->IDR & MYI2C_SDA_PIN) != 0U) ? 1U : 0U;
}

void MyI2C_Start(void)
{
    /* SDA falling while SCL is high generates an I2C start condition. */
    MyI2C_W_SDA(1U);
    MyI2C_W_SCL(1U);
    MyI2C_W_SDA(0U);
    MyI2C_W_SCL(0U);
}

void MyI2C_Stop(void)
{
    /* SDA rising while SCL is high generates an I2C stop condition. */
    MyI2C_W_SDA(0U);
    MyI2C_W_SCL(1U);
    MyI2C_W_SDA(1U);
}

void MyI2C_SendByte(uint8_t Byte)
{
    uint8_t i;

    /* Send MSB first, sampling one bit on every SCL high level. */
    for (i = 0U; i < 8U; i++)
    {
        MyI2C_W_SDA((Byte & 0x80U) != 0U);
        Byte <<= 1U;
        MyI2C_W_SCL(1U);
        MyI2C_W_SCL(0U);
    }
}

uint8_t MyI2C_ReceiveByte(void)
{
    uint8_t i;
    uint8_t Byte = 0U;

    /* Release SDA before reading so the slave can drive the line. */
    MyI2C_W_SDA(1U);
    for (i = 0U; i < 8U; i++)
    {
        Byte <<= 1U;
        MyI2C_W_SCL(1U);
        if (MyI2C_R_SDA() != 0U)
        {
            Byte |= 0x01U;
        }
        MyI2C_W_SCL(0U);
    }

    return Byte;
}

void MyI2C_SendAck(uint8_t AckBit)
{
    MyI2C_W_SDA(AckBit);
    MyI2C_W_SCL(1U);
    MyI2C_W_SCL(0U);
    MyI2C_W_SDA(1U);
}

uint8_t MyI2C_ReceiveAck(void)
{
    uint8_t AckBit;

    MyI2C_W_SDA(1U);
    MyI2C_W_SCL(1U);
    AckBit = MyI2C_R_SDA();
    MyI2C_W_SCL(0U);

    return AckBit;
}

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS);
    (void)MyI2C_ReceiveAck();
    MyI2C_SendByte(RegAddress);
    (void)MyI2C_ReceiveAck();
    MyI2C_SendByte(Data);
    (void)MyI2C_ReceiveAck();
    MyI2C_Stop();
}

uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
    uint8_t Data;

    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS);
    (void)MyI2C_ReceiveAck();
    MyI2C_SendByte(RegAddress);
    (void)MyI2C_ReceiveAck();

    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS | 0x01U);
    (void)MyI2C_ReceiveAck();
    Data = MyI2C_ReceiveByte();
    MyI2C_SendAck(1U);
    MyI2C_Stop();

    return Data;
}

void MPU6050_ReadRegs(uint8_t RegAddress, uint8_t *DataArray, uint8_t Count)
{
    uint8_t i;

    if ((DataArray == NULL) || (Count == 0U))
    {
        return;
    }

    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS);
    (void)MyI2C_ReceiveAck();
    MyI2C_SendByte(RegAddress);
    (void)MyI2C_ReceiveAck();

    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS | 0x01U);
    (void)MyI2C_ReceiveAck();
    for (i = 0U; i < Count; i++)
    {
        DataArray[i] = MyI2C_ReceiveByte();
        MyI2C_SendAck((i < (Count - 1U)) ? 0U : 1U);
    }
    MyI2C_Stop();
}

void MPU6050_Init(void)
{
    /* Wake MPU6050 and set sample rate, low-pass filter, gyro range and accel range. */
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01U);
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00U);
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x04U);
    MPU6050_WriteReg(MPU6050_CONFIG, 0x03U);
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18U);
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18U);
}

int MPU6050_Read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    if ((buf == NULL) || (len == 0U))
    {
        return -1;
    }

    /* addr is the 7-bit device address used by the DMP library interface. */
    MyI2C_Start();
    MyI2C_SendByte(addr << 1U);
    (void)MyI2C_ReceiveAck();
    MyI2C_SendByte(reg);
    (void)MyI2C_ReceiveAck();

    MyI2C_Start();
    MyI2C_SendByte((addr << 1U) | 0x01U);
    (void)MyI2C_ReceiveAck();
    while (len != 0U)
    {
        *buf = MyI2C_ReceiveByte();
        MyI2C_SendAck((len == 1U) ? 1U : 0U);
        buf++;
        len--;
    }
    MyI2C_Stop();

    return 0;
}

int MPU6050_Write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
    uint8_t i;

    if ((data == NULL) || (len == 0U))
    {
        return -1;
    }

    MyI2C_Start();
    MyI2C_SendByte(addr << 1U);
    (void)MyI2C_ReceiveAck();
    MyI2C_SendByte(reg);
    (void)MyI2C_ReceiveAck();
    for (i = 0U; i < len; i++)
    {
        MyI2C_SendByte(data[i]);
        (void)MyI2C_ReceiveAck();
    }
    MyI2C_Stop();

    return 0;
}

void MPU6050_GetData(int16_t *Accel_X, int16_t *Accel_Y, int16_t *Accel_Z,
                     int16_t *Gyro_X, int16_t *Gyro_Y, int16_t *Gyro_Z)
{
    uint8_t Data[14];

    if ((Accel_X == NULL) || (Accel_Y == NULL) || (Accel_Z == NULL) ||
        (Gyro_X == NULL) || (Gyro_Y == NULL) || (Gyro_Z == NULL))
    {
        return;
    }

    /* MPU6050 stores accel, temperature and gyro data in one continuous block. */
    MPU6050_ReadRegs(MPU6050_ACCEL_XOUT_H, Data, 14U);

    *Accel_X = (int16_t)((Data[0] << 8) | Data[1]);
    *Accel_Y = (int16_t)((Data[2] << 8) | Data[3]);
    *Accel_Z = (int16_t)((Data[4] << 8) | Data[5]);
    *Gyro_X = (int16_t)((Data[8] << 8) | Data[9]);
    *Gyro_Y = (int16_t)((Data[10] << 8) | Data[11]);
    *Gyro_Z = (int16_t)((Data[12] << 8) | Data[13]);
}
