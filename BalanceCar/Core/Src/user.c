#include "user.h"

/* MPU6050 raw data cache, updated only by MPU6050_Task. */
MPU6050_Data g_MPU6050_GETDATA;
uint8_t g_flag;

/* Queues are used as task-to-task mailboxes. Most of them keep only one item,
 * so producers overwrite old data and consumers always get the newest state. */
QueueHandle_t MPU6050_Queue;
QueueHandle_t MeasureVelocity_Queue;
QueueHandle_t SerialPrintDataQueue;
QueueHandle_t ControlAngleQueue;
QueueHandle_t OledVelocityQueue;
QueueHandle_t ControlVelocityQueue;
QueueHandle_t RemoteControlQueue;
QueueHandle_t RemoteDisplayQueue;
QueueHandle_t HostCompShowQueue;
QueueHandle_t SerialRDataQueue;

EventGroupHandle_t xVelocityEventGroup;
SemaphoreHandle_t xVelocityDataMutex;
SemaphoreHandle_t xPrintDataMutex;

struct VelocityStructure xSharedVelocityData;
AngleDataStructure AngleData;

/* UART DMA/IDLE interrupt copies one complete frame here before sending it to
 * SerialDataProcess_Task through SerialRDataQueue. */
char g_SerialRdataPacket[100];
char SerialRdataPacket[100];

#define OLED_SAFE_TEXT_COLUMNS       15U
#define OLED_TEXT_WIDTH              128U

static volatile uint8_t balanceRunFlag = REMOTE_BALANCE_DEFAULT_RUN;

/* Remote packets use XOR checksum over the first 11 bytes. */
static uint8_t RemoteApp_Checksum(const uint8_t *payload, uint8_t length)
{
    uint8_t checksum = 0U;
    uint8_t i;

    for (i = 0U; i < length; i++)
    {
        checksum ^= payload[i];
    }

    return checksum;
}

static uint16_t RemoteApp_GetU16(const uint8_t *payload, uint8_t index)
{
    return (uint16_t)payload[index] | ((uint16_t)payload[index + 1U] << 8U);
}

static float RemoteApp_AdcToControlValue(uint16_t adc, float center)
{
    float value;
    float delta;
    float absDelta;

    if (adc > 4096U)
    {
        adc = 4096U;
    }

    if ((center < 1.0f) || (center >= REMOTE_ADC_FULL_SCALE))
    {
        center = REMOTE_ADC_CENTER;
    }

    delta = (float)adc - center;
    absDelta = (delta >= 0.0f) ? delta : -delta;
    /* Dead zone filters small joystick jitter around the center point. */
    if (absDelta <= REMOTE_ADC_DEAD_ZONE)
    {
        return 0.0f;
    }

    if (delta >= 0.0f)
    {
        value = delta * REMOTE_OUTPUT_SCALE / (REMOTE_ADC_FULL_SCALE - center);
    }
    else
    {
        value = delta * REMOTE_OUTPUT_SCALE / center;
    }

    if (value > REMOTE_OUTPUT_SCALE)
    {
        value = REMOTE_OUTPUT_SCALE;
    }
    else if (value < -REMOTE_OUTPUT_SCALE)
    {
        value = -REMOTE_OUTPUT_SCALE;
    }

    return value;
}

static float RemoteApp_AdcToControlValueWithZone(uint16_t adc,
                                                 float zeroMin,
                                                 float zeroMax,
                                                 float outputLimit)
{
    float value;

    if (adc > 4096U)
    {
        adc = 4096U;
    }

    if (outputLimit <= 0.0f)
    {
        outputLimit = REMOTE_OUTPUT_SCALE;
    }

    if ((zeroMin < 1.0f) || (zeroMax <= zeroMin) ||
        (zeroMax >= REMOTE_ADC_FULL_SCALE))
    {
        return RemoteApp_AdcToControlValue(adc, REMOTE_ADC_CENTER);
    }

    /* Some channels are not centered exactly at 2048, so use a custom zero zone. */
    if (((float)adc >= zeroMin) && ((float)adc <= zeroMax))
    {
        return 0.0f;
    }

    if ((float)adc < zeroMin)
    {
        value = ((float)adc - zeroMin) * outputLimit / zeroMin;
    }
    else
    {
        value = ((float)adc - zeroMax) * outputLimit /
                (REMOTE_ADC_FULL_SCALE - zeroMax);
    }

    if (value > outputLimit)
    {
        value = outputLimit;
    }
    else if (value < -outputLimit)
    {
        value = -outputLimit;
    }

    return value;
}

static float Balance_LimitFloat(float value, float limit)
{
    if (value > limit)
    {
        value = limit;
    }
    else if (value < -limit)
    {
        value = -limit;
    }

    return value;
}

static uint8_t RemoteApp_DecodePacket(const uint8_t *payload,
                                      RemoteControlStatus_t *status)
{
    RemoteControlData_t data;

    if ((payload == NULL) || (status == NULL))
    {
        return 0U;
    }

    if (payload[0] != REMOTE_PACKET_SYNC)
    {
        return 0U;
    }

    if (RemoteApp_Checksum(payload, 11U) != payload[11])
    {
        return 0U;
    }

    data.sequence = payload[1];
    data.lv = RemoteApp_GetU16(payload, 2U);
    data.lh = RemoteApp_GetU16(payload, 4U);
    data.rv = RemoteApp_GetU16(payload, 6U);
    data.rh = RemoteApp_GetU16(payload, 8U);
    data.sw_status = payload[10];

    /* Buttons directly control the global balance enable flag. */
    if ((data.sw_status & REMOTE_KEY_PID_START) != 0U)
    {
        balanceRunFlag = 1U;
    }
    else if ((data.sw_status & REMOTE_KEY_PID_STOP) != 0U)
    {
        balanceRunFlag = 0U;
    }

    status->lv = RemoteApp_AdcToControlValueWithZone(data.lv,
                                                     REMOTE_LV_ADC_ZERO_MIN,
                                                     REMOTE_LV_ADC_ZERO_MAX,
                                                     REMOTE_LV_CONTROL_LIMIT);
    status->lh = RemoteApp_AdcToControlValue(data.lh, REMOTE_ADC_CENTER);
    status->rv = RemoteApp_AdcToControlValue(data.rv, REMOTE_ADC_CENTER);
    status->rh = RemoteApp_AdcToControlValueWithZone(data.rh,
                                                     REMOTE_RH_ADC_ZERO_MIN,
                                                     REMOTE_RH_ADC_ZERO_MAX,
                                                     REMOTE_RH_CONTROL_LIMIT);
    status->run_flag = balanceRunFlag;
    status->rx_ok = 1U;
    status->sw_status = data.sw_status;
    status->sequence = data.sequence;
    status->timestamp = xTaskGetTickCount();

    return 1U;
}

static void OLED_ShowLine(uint8_t page, const char *text)
{
    char line[OLED_SAFE_TEXT_COLUMNS + 1U];
    uint8_t i;
    uint8_t row;
    uint8_t col;

    /* OLED_ShowString does not clear old tail characters, so pad the line first. */
    for (i = 0U; i < OLED_SAFE_TEXT_COLUMNS; i++)
    {
        if ((text != NULL) && (text[i] != '\0'))
        {
            line[i] = text[i];
        }
        else
        {
            line[i] = ' ';
        }
    }
    line[OLED_SAFE_TEXT_COLUMNS] = '\0';
    OLED_ShowString(0U, page, (uint8_t *)line);

    for (row = 0U; row < 2U; row++)
    {
        if ((page + row) < 8U)
        {
            OLED_Set_Pos((uint8_t)(OLED_SAFE_TEXT_COLUMNS * 8U), (uint8_t)(page + row));
            for (col = (uint8_t)(OLED_SAFE_TEXT_COLUMNS * 8U); col < OLED_TEXT_WIDTH; col++)
            {
                OLED_WR_Byte(0x00U, OLED_DATA);
            }
        }
    }
}

void UartPrint_Put(const char *fmt, ...)
{
    PrintDataStructure msg;
    va_list args;

    if ((fmt == NULL) || (SerialPrintDataQueue == NULL))
    {
        return;
    }

    memset(&msg, 0, sizeof(msg));
    va_start(args, fmt);
    (void)vsnprintf(msg.data, sizeof(msg.data), fmt, args);
    va_end(args);

    /* If the print queue is full, drop the oldest message to keep recent debug data. */
    if (xQueueSend(SerialPrintDataQueue, &msg, 0U) != pdPASS)
    {
        PrintDataStructure drop;
        (void)xQueueReceive(SerialPrintDataQueue, &drop, 0U);
        (void)xQueueSend(SerialPrintDataQueue, &msg, 0U);
    }
}

void OLED_Task(void *para)
{
    AngleDataStructure displayAngle = {0};
    struct VelocityStructure displayVelocity = {0};
    RemoteControlStatus_t remote = {0};
    TickType_t lastWake;
    TickType_t lastSlowRefresh;
    TickType_t now;
    char line[20];

    (void)para;

    /* The display task is read-only: it receives snapshots and never changes control data. */
    OLED_Init();
    OLED_Clear();
    remote.run_flag = REMOTE_BALANCE_DEFAULT_RUN;
    lastWake = xTaskGetTickCount();
    lastSlowRefresh = lastWake - pdMS_TO_TICKS(OLED_SLOW_REFRESH_MS);

    while (1)
    {
        now = xTaskGetTickCount();
        /* Fast refresh: attitude is the most important real-time value on screen. */
        (void)xQueueReceive(MPU6050_Queue, &displayAngle, 0U);
        (void)snprintf(line, sizeof(line), "Ang:%+07.2f", displayAngle.MixAngle);
        OLED_ShowLine(0U, line);

        if ((now - lastSlowRefresh) >= pdMS_TO_TICKS(OLED_SLOW_REFRESH_MS))
        {
            lastSlowRefresh = now;
            /* Slow refresh: speed and remote status change less frequently than angle. */
            (void)xQueueReceive(OledVelocityQueue, &displayVelocity, 0U);
            (void)xQueueReceive(RemoteDisplayQueue, &remote, 0U);
            remote.run_flag = balanceRunFlag;

            (void)snprintf(line, sizeof(line), "A:%+4.1f B:%+4.1f", displayVelocity.A, displayVelocity.B);
            OLED_ShowLine(2U, line);

            (void)snprintf(line, sizeof(line), "LV:%+4.1f RH:%+4.1f",
                           remote.lv,
                           remote.rh);
            OLED_ShowLine(4U, line);

            (void)snprintf(line, sizeof(line), "Run:%u SW:%02X",
                           (unsigned int)remote.run_flag,
                           (unsigned int)remote.sw_status);
            OLED_ShowLine(6U, line);
        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(OLED_ANGLE_REFRESH_MS));
    }
}

void MPU6050_Task(void *para)
{
    float Alpha = 0.06f;
    TickType_t timer;
    TickType_t now;
    TickType_t printTimer;
    AngleDataStructure newAngle = {0};

    (void)para;

    MPU6050_Init();
    vTaskDelay(pdMS_TO_TICKS(20U));

    timer = xTaskGetTickCount();
    printTimer = timer;

    while (1)
    {
        /* Read raw acceleration and gyro data through software I2C. */
        MPU6050_GetData(&g_MPU6050_GETDATA.Ax, &g_MPU6050_GETDATA.Ay, &g_MPU6050_GETDATA.Az,
                        &g_MPU6050_GETDATA.Gx, &g_MPU6050_GETDATA.Gy, &g_MPU6050_GETDATA.Gz);

        /* Use real task interval as Kalman dt, but clamp abnormal scheduler gaps. */
        now = xTaskGetTickCount();
        Pitch_KalmanFilter.dt = (float)(now - timer) / 1000.0f;
        if (Pitch_KalmanFilter.dt <= 0.0f)
        {
            Pitch_KalmanFilter.dt = 0.005f;
        }
        else if (Pitch_KalmanFilter.dt > 0.05f)
        {
            Pitch_KalmanFilter.dt = 0.05f;
        }
        timer = now;

        /* Acc angle is stable at low frequency; gyro is responsive at high frequency. */
        newAngle.Acc_YAngle = (float)(atan2(g_MPU6050_GETDATA.Ax, g_MPU6050_GETDATA.Az) * 180.0 / 3.14159);
        newAngle.Acc_ZAngle = (float)(atan2(g_MPU6050_GETDATA.Ay, g_MPU6050_GETDATA.Az) * 180.0 / 3.14159);
        newAngle.OneRakeFilter = newAngle.OneRakeFilter +
                                 (-g_MPU6050_GETDATA.Gy - 40.0f) / 32768.0f * 2000.0f * Pitch_KalmanFilter.dt;
        newAngle.OneRakeFilter = Alpha * newAngle.Acc_YAngle + (1.0f - Alpha) * newAngle.OneRakeFilter;
        newAngle.Gyro_Y = g_MPU6050_GETDATA.Gy / 32768.0f * 2000.0f - MPU6050_GYRO_Y_OFFSET;
        newAngle.MixAngle = KalmanFilter(&Pitch_KalmanFilter, newAngle.Acc_YAngle, -newAngle.Gyro_Y) -
                            BALANCE_ANGLE_OFFSET;

        AngleData = newAngle;
        /* One angle sample feeds both OLED display and the balance control loop. */
        (void)xQueueOverwrite(MPU6050_Queue, &newAngle);
        (void)xQueueOverwrite(ControlAngleQueue, &newAngle);

        if ((now - printTimer) >= pdMS_TO_TICKS(50U))
        {
            printTimer = now;
            UartPrint_Put("[plot,%f,%f,%f]\r\n",
                          newAngle.Acc_YAngle, newAngle.OneRakeFilter, newAngle.MixAngle);
        }

        vTaskDelay(pdMS_TO_TICKS(5U));
    }
}

void PDVectAndPISoeed_Task(void *para)
{
    struct VelocityStructure RecVelocity = {0};
    RemoteControlStatus_t remote = {0};
    VectPwmStructure VectPwm = {0};
    AngleDataStructure controlAngle = {0};
    float AverageSpeed = 0.0f;
    float DifSpeed = 0.0f;
    float SpeedTargetAngle = 0.0f;
    TickType_t now;
    uint8_t Speedcount = 0U;
    uint8_t controlEnable = 0U;
    uint8_t controlRunFlag = 0U;
    uint8_t remoteFresh = 0U;

    (void)para;

    remote.run_flag = REMOTE_BALANCE_DEFAULT_RUN;

    while (1)
    {
        /* ControlAngleQueue is the control clock: one new angle triggers one control step. */
        if (xQueueReceive(ControlAngleQueue, &controlAngle, portMAX_DELAY) == pdPASS)
        {
            now = xTaskGetTickCount();
            (void)xQueueReceive(ControlVelocityQueue, &RecVelocity, 0U);
            (void)xQueueReceive(RemoteControlQueue, &remote, 0U);

            controlRunFlag = balanceRunFlag;
            /* Remote timeout is treated as zero command to avoid stale joystick motion. */
            remoteFresh = ((remote.rx_ok != 0U) &&
                           ((now - remote.timestamp) <= pdMS_TO_TICKS(REMOTE_PACKET_TIMEOUT_MS))) ? 1U : 0U;

            if (remoteFresh != 0U)
            {
                PIDspeedControl.TargetValue = remote.lv;
                PIDTurnControl.TargetValue = remote.rh;
            }
            else
            {
                remote.rx_ok = 0U;
                PIDspeedControl.TargetValue = 0.0f;
                PIDTurnControl.TargetValue = 0.0f;
            }

            /* Speed PI runs at a lower rate and outputs the target balance angle. */
            if (++Speedcount >= BALANCE_SPEED_LOOP_DIV)
            {
                Speedcount = 0U;
                AverageSpeed = (RecVelocity.A + RecVelocity.B) / 2.0f;
                AverageSpeed *= BALANCE_SPEED_FEEDBACK_DIR;

                /* Bleed speed integral only near stop to reduce small rollback. */
                if ((fabs(PIDspeedControl.TargetValue) <= BALANCE_STOP_TARGET_DEAD_ZONE) &&
                    (fabs(AverageSpeed) <= BALANCE_STOP_SPEED_DEAD_ZONE))
                {
                    PIDspeedControl.ErrSum *= BALANCE_STOP_ERRSUM_DECAY;
                    if (fabs(PIDspeedControl.ErrSum) <= BALANCE_STOP_ERRSUM_ZERO)
                    {
                        PIDspeedControl.ErrSum = 0.0f;
                    }
                }

                SpeedTargetAngle = PIAlgorithm(&PIDspeedControl, AverageSpeed);
                PIDVectControl.TargetValue = Balance_LimitFloat(SpeedTargetAngle, BALANCE_SPEED_OUTPUT_LIMIT);

                DifSpeed = (RecVelocity.A - RecVelocity.B) * BALANCE_TURN_FEEDBACK_DIR;
                VectPwm.difPWM = (int)TurnPIAlgorithm(&PIDTurnControl, DifSpeed);
            }

            controlEnable = (controlRunFlag != 0U) ? 1U : 0U;
            /* Stop motors and reset integrators when disabled or the car falls too far. */
            if ((controlEnable == 0U) ||
                (controlAngle.MixAngle > 40.0f) || (controlAngle.MixAngle < -40.0f))
            {
                Velocity_Ouput(0, 0);
                PID_ResetSpeedState(&PIDspeedControl);
                PID_ResetSpeedState(&PIDTurnControl);
                PIDVectControl.TargetValue = 0.0f;
                PIDVectControl.LastErr = 0.0f;
                PIDVectControl.Output = 0;
                VectPwm.AvePWM = 0;
                VectPwm.difPWM = 0;
                AverageSpeed = 0.0f;
                SpeedTargetAngle = 0.0f;
                Speedcount = 0U;
            }
            else
            {
                /* Upright PD uses angle error and gyro feedback, then adds turn difference. */
                VectPwm.AvePWM = -PDAlgorithm_withgyro(&PIDVectControl,
                                                        controlAngle.MixAngle,
                                                        controlAngle.Gyro_Y);
                VectPwm.PWMA = VectPwm.AvePWM + VectPwm.difPWM / 2;
                VectPwm.PWMB = VectPwm.AvePWM - VectPwm.difPWM / 2;
                Velocity_Ouput(VectPwm.PWMA, VectPwm.PWMB);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5U));
    }
}

void VelocityCulculate_Task(void *para)
{
    float AWeel_Veloc = 0.0f;
    float BWeel_Veloc = 0.0f;
    struct VelocityStructure newData;
    short AWeel_EncoderCount;
    short BWeel_EncoderCount;

    (void)para;

    while (1)
    {
        /* Read incremental encoder counts every 10 ms, then clear counters for next window. */
        AWeel_EncoderCount = (short)__HAL_TIM_GET_COUNTER(&htim2);
        BWeel_EncoderCount = (short)__HAL_TIM_GET_COUNTER(&htim4);
        __HAL_TIM_SET_COUNTER(&htim2, 0);
        __HAL_TIM_SET_COUNTER(&htim4, 0);

        AWeel_Veloc = (float)AWeel_EncoderCount * 100.0f / 20.0f / 13.0f / 4.0f;
        BWeel_Veloc = (float)BWeel_EncoderCount * 100.0f / 20.0f / 13.0f / 4.0f;

        newData.A = -AWeel_Veloc;
        newData.B = BWeel_Veloc;
        newData.timestamp = xTaskGetTickCount();

        /* Shared copy is used by HostComp_Task; queues are used by OLED and control tasks. */
        if (xVelocityDataMutex != NULL)
        {
            (void)xSemaphoreTake(xVelocityDataMutex, portMAX_DELAY);
            xSharedVelocityData = newData;
            (void)xSemaphoreGive(xVelocityDataMutex);
        }
        if (xVelocityEventGroup != NULL)
        {
            (void)xEventGroupSetBits(xVelocityEventGroup, VelocityAcessEventBits);
        }

        (void)xQueueOverwrite(OledVelocityQueue, &newData);
        (void)xQueueOverwrite(ControlVelocityQueue, &newData);

        vTaskDelay(pdMS_TO_TICKS(10U));
    }
}

void HostComp_Task(void *para)
{
    struct VelocityStructure input;
    EventBits_t EventBits;
    uint32_t lastTimestamp = 0U;

    (void)para;

    while (1)
    {
        EventBits = xEventGroupWaitBits(xVelocityEventGroup,
                                        VelocityAcessEventBits,
                                        pdTRUE,
                                        pdFALSE,
                                        pdMS_TO_TICKS(100U));
        if ((EventBits & VelocityAcessEventBits) != 0U)
        {
            (void)xSemaphoreTake(xVelocityDataMutex, portMAX_DELAY);
            input = xSharedVelocityData;
            (void)xSemaphoreGive(xVelocityDataMutex);

            /* Avoid sending the same velocity frame repeatedly to the host computer. */
            if (input.timestamp != lastTimestamp)
            {
                lastTimestamp = input.timestamp;
                ANODT_SEND_F2((int16_t)(input.A * 100.0f), 300,
                              (int16_t)(input.B * 100.0f), 300);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50U));
    }
}

void SerialDataProcess_Task(void *para)
{
    (void)para;

    while (1)
    {
        /* Commands are complete frames stripped of '[' and ']' by the UART IDLE handler. */
        if (xQueueReceive(SerialRDataQueue, &SerialRdataPacket, portMAX_DELAY) == pdPASS)
        {
            char *Tag = strtok(SerialRdataPacket, ",");

            if (Tag == NULL)
            {
                continue;
            }

            if (strcmp(Tag, "key") == 0)
            {
                (void)strtok(NULL, ",");
                (void)strtok(NULL, ",");
            }
            else if (strcmp(Tag, "zhongzhi") == 0)
            {
                char *Name = strtok(NULL, ",");
                char *Value = strtok(NULL, ",");
                (void)Name;
                if (Value != NULL)
                {
                    PIDVectControl.TargetValue = atof(Value);
                }
            }
            else if (strcmp(Tag, "slider") == 0)
            {
                char *Name = strtok(NULL, ",");
                char *Value = strtok(NULL, ",");

                if ((Name == NULL) || (Value == NULL))
                {
                    continue;
                }

                /* Upper-computer sliders tune PID parameters online during testing. */
                if (strcmp(Name, "Vectkp") == 0)
                {
                    PIDVectControl.kp = atof(Value);
                    UartPrint_Put("slider,Vectkp,%f\r\n", PIDVectControl.kp);
                }
                else if (strcmp(Name, "Vectkd") == 0)
                {
                    PIDVectControl.kd = atof(Value);
                    UartPrint_Put("slider,Vectkd,%f\r\n", PIDVectControl.kd);
                }
                else if (strcmp(Name, "Skp") == 0)
                {
                    PIDspeedControl.kp = atof(Value);
                    UartPrint_Put("slider,Skp,%f\r\n", PIDspeedControl.kp);
                }
                else if (strcmp(Name, "Ski") == 0)
                {
                    PIDspeedControl.ki = atof(Value);
                    UartPrint_Put("slider,Ski,%f\r\n", PIDspeedControl.ki);
                }
                else if (strcmp(Name, "Tkp") == 0)
                {
                    PIDTurnControl.kp = atof(Value);
                    UartPrint_Put("slider,Tkp,%f\r\n", PIDTurnControl.kp);
                }
                else if (strcmp(Name, "Tki") == 0)
                {
                    PIDTurnControl.ki = atof(Value);
                    UartPrint_Put("slider,Tki,%f\r\n", PIDTurnControl.ki);
                }
            }
            else if (strcmp(Tag, "joystick") == 0)
            {
                char *LHStr = strtok(NULL, ",");
                char *LVStr = strtok(NULL, ",");
                char *RHStr = strtok(NULL, ",");
                char *RVStr = strtok(NULL, ",");

                if ((LHStr != NULL) && (LVStr != NULL) && (RHStr != NULL) && (RVStr != NULL))
                {
                    int LH = atoi(LHStr);
                    int LV = atoi(LVStr);
                    int RH = atoi(RHStr);
                    int RV = atoi(RVStr);

                    PIDspeedControl.TargetValue = (float)LV / 25.0f;
                    PIDTurnControl.TargetValue = (float)RH / 25.0f;
                    UartPrint_Put("joystick,%d,%d,%d,%d\r\n", LH, LV, RH, RV);
                }
            }
        }
    }
}

void UartPrint_Task(void *para)
{
    PrintDataStructure msg;
    uint16_t length;

    (void)para;

    while (1)
    {
        /* All printf-like output is serialized here to avoid UART contention. */
        if (xQueueReceive(SerialPrintDataQueue, &msg, portMAX_DELAY) == pdPASS)
        {
            length = (uint16_t)strlen(msg.data);
            if (length > 0U)
            {
                (void)HAL_UART_Transmit(&huart1, (uint8_t *)msg.data, length, 0xFFFFU);
            }
        }
    }
}

void NRF24L01_Receive_Task(void *para)
{
    uint8_t ready = 0U;
    uint8_t payload[NRF24L01_REMOTE_PAYLOAD_WIDTH];
    RemoteControlStatus_t remote = {0};
    TickType_t lastPacketTick = 0U;
    TickType_t now;

    (void)para;

    remote.run_flag = REMOTE_BALANCE_DEFAULT_RUN;

    while (1)
    {
        now = xTaskGetTickCount();
        if (ready == 0U)
        {
            /* Keep retrying wireless initialization; the rest of the system can still run. */
            ready = NRF24L01_Init();
            if (ready == 0U)
            {
                remote.rx_ok = 0U;
                remote.run_flag = balanceRunFlag;
                remote.timestamp = now;
                (void)xQueueOverwrite(RemoteControlQueue, &remote);
                (void)xQueueOverwrite(RemoteDisplayQueue, &remote);
                vTaskDelay(pdMS_TO_TICKS(1000U));
                continue;
            }

            NRF24L01_RxMode();
            lastPacketTick = now;
            (void)xQueueOverwrite(RemoteDisplayQueue, &remote);
            UartPrint_Put("nrf24l01,rx_ready\r\n");
        }

        if (NRF24L01_ReceivePacket(payload, NRF24L01_REMOTE_PAYLOAD_WIDTH) != 0U)
        {
            /* Valid remote packets update both control and OLED display queues. */
            if (RemoteApp_DecodePacket(payload, &remote) != 0U)
            {
                lastPacketTick = remote.timestamp;
                (void)xQueueOverwrite(RemoteControlQueue, &remote);
                (void)xQueueOverwrite(RemoteDisplayQueue, &remote);
            }
            else
            {
                NRF24L01_FlushRx();
            }
        }

        now = xTaskGetTickCount();
        /* When packets stop arriving, clear joystick values but keep the run flag state. */
        if ((now - lastPacketTick) > pdMS_TO_TICKS(REMOTE_PACKET_TIMEOUT_MS))
        {
            remote.rx_ok = 0U;
            remote.run_flag = balanceRunFlag;
            remote.lv = 0.0f;
            remote.lh = 0.0f;
            remote.rv = 0.0f;
            remote.rh = 0.0f;
            (void)xQueueOverwrite(RemoteControlQueue, &remote);
            (void)xQueueOverwrite(RemoteDisplayQueue, &remote);
        }

        vTaskDelay(pdMS_TO_TICKS(5U));
    }
}
