#ifndef __USER_H__
#define __USER_H__

#include "bspsystem.h"

#define VelocityAcessEventBits 0x01U

/* Remote packet format: 12 bytes, byte0 sync, byte11 XOR checksum. */
#define REMOTE_PACKET_SYNC     0xA5U
#define REMOTE_ADC_FULL_SCALE  4096.0f
#define REMOTE_OUTPUT_SCALE    3.0f
#define REMOTE_ADC_CENTER      2048.0f
#define REMOTE_ADC_DEAD_ZONE   50.0f
#define REMOTE_LV_ADC_ZERO_MIN 1900.0f
#define REMOTE_LV_ADC_ZERO_MAX 2300.0f
#define REMOTE_RH_ADC_ZERO_MIN 2300.0f
#define REMOTE_RH_ADC_ZERO_MAX 2600.0f
#define REMOTE_LV_CONTROL_LIMIT 2.5f
#define REMOTE_RH_CONTROL_LIMIT 3.0f
#define REMOTE_BALANCE_DEFAULT_RUN 1U
#define REMOTE_KEY_PID_START   0x04U
#define REMOTE_KEY_PID_STOP    0x08U
#define REMOTE_PACKET_TIMEOUT_MS 500U
#define OLED_ANGLE_REFRESH_MS  50U
#define OLED_SLOW_REFRESH_MS   150U

/* Tuned balance parameters. Keep these values unless retuning on the car. */
#define MPU6050_GYRO_Y_OFFSET  2.44f
#define BALANCE_ANGLE_OFFSET   +3.5f
#define BALANCE_SPEED_KP       7.2f   //13.0 
#define BALANCE_SPEED_KI       0.04f  //0.074
#define BALANCE_SPEED_KD       18.0f   //16.0
#define BALANCE_SPEED_ERRSUM_LIMIT 800.0f
#define BALANCE_SPEED_INTEGRAL_SEPA 1.5f
#define BALANCE_SPEED_OUTPUT_LIMIT 20.0f
#define BALANCE_SPEED_LOOP_DIV 6U
#define BALANCE_SPEED_FEEDBACK_DIR 1.0f
#define BALANCE_TURN_FEEDBACK_DIR 1.0f
#define BALANCE_STOP_TARGET_DEAD_ZONE 0.08f
#define BALANCE_STOP_SPEED_DEAD_ZONE 0.15f
#define BALANCE_STOP_ERRSUM_DECAY 0.90f
#define BALANCE_STOP_ERRSUM_ZERO 0.05f
#define BALANCE_TURN_TARGET_DEAD_ZONE 0.08f
#define BALANCE_TURN_SPEED_DEAD_ZONE 0.12f
#define BALANCE_TURN_ERRSUM_DECAY 0.85f
#define BALANCE_TURN_ERRSUM_ZERO 0.05f

typedef struct RemoteControlData
{
    /* Raw ADC/button values decoded directly from the wireless packet. */
    uint16_t lv;
    uint16_t lh;
    uint16_t rv;
    uint16_t rh;
    uint8_t sw_status;
    uint8_t sequence;
}RemoteControlData_t;

typedef struct{
    /* Normalized remote values used by control and display tasks. */
    float lv;
    float lh;
    float rv;
    float rh;
    uint8_t run_flag;
    uint8_t rx_ok;
    uint8_t sw_status;
    uint8_t sequence;
    uint32_t timestamp;
}RemoteControlStatus_t;

typedef struct{
    /* Attitude data sent from MPU6050_Task to display and control tasks. */
    float Acc_YAngle;
    float Acc_ZAngle;
    float MixAngle;
    float Gyro_Y;
    float OneRakeFilter;
}AngleDataStructure;

typedef struct{
    /* Final PWM composition: average balance PWM plus differential turn PWM. */
    int AvePWM;
    int PWMA;
    int PWMB;
    int difPWM;
}VectPwmStructure;

typedef struct{
    /* One queued UART print message. Keep it small to save FreeRTOS heap. */
    char data[96];
}PrintDataStructure;

void OLED_Task(void *para);
void MPU6050_Task(void *para);
void VelocityCulculate_Task(void *para);
void HostComp_Task(void *para);
void SerialDataProcess_Task(void *para);
void UartPrint_Task(void *para);
void PDVectAndPISoeed_Task(void *para);
void NRF24L01_Receive_Task(void *para);
void UartPrint_Put(const char *fmt, ...);

extern EventGroupHandle_t xVelocityEventGroup;
extern SemaphoreHandle_t xVelocityDataMutex;
extern struct VelocityStructure xSharedVelocityData;
extern QueueHandle_t ControlAngleQueue;
extern QueueHandle_t OledVelocityQueue;
extern QueueHandle_t ControlVelocityQueue;
extern QueueHandle_t RemoteControlQueue;
extern QueueHandle_t RemoteDisplayQueue;

#endif
