#include "MyFunc.h"

static int LimitInt(int value, int limit)
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

static float LimitFloat(float value, float limit)
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

void VelocityBroad_Init(void)
{
    /* Event group + mutex are used by the optional host-computer velocity task. */
    xVelocityEventGroup = xEventGroupCreate();
    xVelocityDataMutex = xSemaphoreCreateMutex();
    xPrintDataMutex = NULL;

    xSharedVelocityData.A = 0.0f;
    xSharedVelocityData.B = 0.0f;
    xSharedVelocityData.timestamp = 0U;
}

void Queue_Init(void)
{
    /* Length-1 queues carry the newest state only; old sensor data is not useful for control. */
    MPU6050_Queue = xQueueCreate(1U, sizeof(AngleDataStructure));
    ControlAngleQueue = xQueueCreate(1U, sizeof(AngleDataStructure));
    OledVelocityQueue = xQueueCreate(1U, sizeof(struct VelocityStructure));
    ControlVelocityQueue = xQueueCreate(1U, sizeof(struct VelocityStructure));
    RemoteControlQueue = xQueueCreate(1U, sizeof(RemoteControlStatus_t));
    RemoteDisplayQueue = xQueueCreate(1U, sizeof(RemoteControlStatus_t));
    SerialRDataQueue = xQueueCreate(4U, sizeof(g_SerialRdataPacket));
    SerialPrintDataQueue = xQueueCreate(4U, sizeof(PrintDataStructure));
}

void Velocity_Ouput(int Aweel, int Bweel)
{
    /* Limit command first so PWM compare values always stay inside timer period. */
    Aweel = LimitInt(Aweel, 1000);
    Bweel = LimitInt(Bweel, 1000);

    /* A wheel: direction GPIOs decide polarity, TIM1_CH4 decides magnitude. */
    if (Aweel >= 0)
    {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, Aweel);
    }
    else
    {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, -Aweel);
    }

    /* B wheel: direction GPIOs decide polarity, TIM1_CH1 decides magnitude. */
    if (Bweel >= 0)
    {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Bweel);
    }
    else
    {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -Bweel);
    }
}

int PWM_Map(int PWMValue)
{
    int Output;

    /* Optional motor dead-zone compensation. Current control path outputs raw PWM. */
    PWMValue = LimitInt(PWMValue, 1000);
    if (PWMValue > 0)
    {
        Output = (int)(PWMValue * 0.9f) + 100;
        if (Output > 1000)
        {
            Output = 1000;
        }
        if (Output <= 200)
        {
            Output = 200;
        }
    }
    else if (PWMValue < 0)
    {
        Output = (int)(PWMValue * 0.99f) - 10;
        if (Output < -1000)
        {
            Output = -1000;
        }
        if (Output >= -10)
        {
            Output = -10;
        }
    }
    else
    {
        Output = 0;
    }

    return Output;
}

PIDspeedControlStructure PIDspeedControl;
PIDspeedControlStructure PIDTurnControl;
PID_Para PIDVectControl;

void PID_Init(void)
{
    /* Upright loop: PD control from balance angle to average motor PWM. */
    PIDVectControl.TargetValue = +0.0f;
    PIDVectControl.ActualValue = 0.0f;
    PIDVectControl.NowErr = 0.0f;
    PIDVectControl.LastErr = 0.0f;
    PIDVectControl.ErrSum = 0.0f;
    PIDVectControl.ErrSum_Limit = 0.0f;
    PIDVectControl.IntelSepaThreshold = 20.0f;
    PIDVectControl.OutputOffset = 0.0f;
    PIDVectControl.kp = 37.0f;
    PIDVectControl.ki = 0.0f;
    PIDVectControl.kd = 2.8f;
    PIDVectControl.OutputLimit = 1000;
    PIDVectControl.Output = 0;

    /* Speed loop: PI/PD-like control from wheel speed to target balance angle. */
    PIDspeedControl.TargetValue = 0.0f;
    PIDspeedControl.ActualValue = 0.0f;
    PIDspeedControl.NowErr = 0.0f;
    PIDspeedControl.LastErr = 0.0f;
    PIDspeedControl.ErrSum = 0.0f;
    PIDspeedControl.ErrSum_Limit = BALANCE_SPEED_ERRSUM_LIMIT;
    PIDspeedControl.IntelSepaThreshold = BALANCE_SPEED_INTEGRAL_SEPA;
    PIDspeedControl.OutputOffset = 0.0f;
    PIDspeedControl.kp = BALANCE_SPEED_KP;
    PIDspeedControl.ki = BALANCE_SPEED_KI;
    PIDspeedControl.kd = BALANCE_SPEED_KD;
    PIDspeedControl.OutputLimit = BALANCE_SPEED_OUTPUT_LIMIT;
    PIDspeedControl.Output = 0.0f;

    /* Turn loop: speed difference control, added as left/right PWM difference. */
    PIDTurnControl.TargetValue = 0.0f;
    PIDTurnControl.ActualValue = 0.0f;
    PIDTurnControl.NowErr = 0.0f;
    PIDTurnControl.LastErr = 0.0f;
    PIDTurnControl.ErrSum = 0.0f;
    PIDTurnControl.ErrSum_Limit = 500.0f;
    PIDTurnControl.IntelSepaThreshold = 10.0f;
    PIDTurnControl.OutputOffset = 0.0f;
    PIDTurnControl.kp = 100.0f;
    PIDTurnControl.ki = 0.5f;
    PIDTurnControl.kd = 0.0f;
    PIDTurnControl.OutputLimit = 500.0f;
    PIDTurnControl.Output = 0.0f;
}

void PID_ResetSpeedState(PIDspeedControlStructure *PIDParameter)
{
    if (PIDParameter == NULL)
    {
        return;
    }

    /* Reset dynamic state but keep tuned kp/ki/kd and limits unchanged. */
    PIDParameter->ActualValue = 0.0f;
    PIDParameter->NowErr = 0.0f;
    PIDParameter->LastErr = 0.0f;
    PIDParameter->ErrSum = 0.0f;
    PIDParameter->Output = 0.0f;
}

int PIDAlgorithm(PID_Para *PIDParameter, float RealityValue)
{
    static float a = 0.3, k = 0.1;
    static uint8_t C;
    /* Generic positional PID with first-order input filtering. */
    PIDParameter->ActualValue = a*RealityValue + (1-a)*PIDParameter->ActualValue;
    PIDParameter->NowErr = PIDParameter->TargetValue - PIDParameter->ActualValue;
    C = 1/(k*PIDParameter->NowErr + 1);
    PIDParameter->ErrSum += PIDParameter->NowErr;
    if (PIDParameter->ErrSum > PIDParameter->ErrSum_Limit)
        PIDParameter->ErrSum = PIDParameter->ErrSum_Limit;
    else if (PIDParameter->ErrSum < -PIDParameter->ErrSum_Limit)
        PIDParameter->ErrSum = -PIDParameter->ErrSum_Limit;
    PIDParameter->Output = PIDParameter->kp*PIDParameter->NowErr
                                + PIDParameter->ki*PIDParameter->ErrSum*C
                                + PIDParameter->kd*(PIDParameter->NowErr - PIDParameter->LastErr);
    if(PIDParameter->Output > 5) PIDParameter->Output += PIDParameter->OutputOffset;
    else if(PIDParameter->Output < -5) PIDParameter->Output -= PIDParameter->OutputOffset;
    else PIDParameter->Output = 0;
    PIDParameter->LastErr = PIDParameter->NowErr;
    return PIDParameter->Output;
}

int PDAlgorithm(PID_Para *PIDParameter, float RealityValue)
{
    static float a = 0.3f;

    /* PD variant using error difference as derivative term. */
    PIDParameter->ActualValue = a * RealityValue + (1.0f - a) * PIDParameter->ActualValue;
    PIDParameter->NowErr = PIDParameter->TargetValue - PIDParameter->ActualValue;
    PIDParameter->Output = PIDParameter->kp * PIDParameter->NowErr +
                           PIDParameter->kd * (PIDParameter->NowErr - PIDParameter->LastErr);
    PIDParameter->Output = LimitInt(PIDParameter->Output, PIDParameter->OutputLimit);
    PIDParameter->LastErr = PIDParameter->NowErr;

    return PIDParameter->Output;
}

int PDAlgorithm_withgyro(PID_Para *PIDParameter, float RealityValue, float gyro)
{
    float a = 0.3;
    /* For balancing, gyro is a cleaner derivative signal than angle difference. */
    PIDParameter->ActualValue = a*RealityValue + (1-a)*PIDParameter->ActualValue;
    PIDParameter->ActualValue = RealityValue;
    PIDParameter->NowErr = PIDParameter->TargetValue - PIDParameter->ActualValue;
    PIDParameter->Output = PIDParameter->kp*PIDParameter->NowErr
                                + PIDParameter->kd*gyro;
    if (PIDParameter->Output > PIDParameter->OutputLimit) {
        PIDParameter->Output = PIDParameter->OutputLimit;
    }
    else if (PIDParameter->Output < -PIDParameter->OutputLimit) {
        PIDParameter->Output = -PIDParameter->OutputLimit;
    }
    PIDParameter->LastErr = PIDParameter->NowErr;
    return PIDParameter->Output;
}

float PIAlgorithm(PIDspeedControlStructure *PIDParameter, float RealityValue)
{
    float a = 0.3f;
    float absErr;
    float lastActual;
    float actualDelta;

    lastActual = PIDParameter->ActualValue;
    PIDParameter->ActualValue =
        a * RealityValue + (1.0f - a) * PIDParameter->ActualValue;
    PIDParameter->NowErr = PIDParameter->TargetValue - PIDParameter->ActualValue;
    absErr = fabs(PIDParameter->NowErr);
    actualDelta = PIDParameter->ActualValue - lastActual;

    /* Integral separation prevents large errors from winding up the speed loop. */
    if ((PIDParameter->IntelSepaThreshold <= 0.0f) ||
        (absErr <= PIDParameter->IntelSepaThreshold))
    {
        PIDParameter->ErrSum += PIDParameter->NowErr;
        PIDParameter->ErrSum = LimitFloat(PIDParameter->ErrSum, PIDParameter->ErrSum_Limit);
    }

    /* Derivative on measurement keeps damping but avoids target-step kickback. */
    PIDParameter->Output = PIDParameter->kp * PIDParameter->NowErr +
                           PIDParameter->ki * PIDParameter->ErrSum -
                           PIDParameter->kd * actualDelta;
    PIDParameter->Output = LimitFloat(PIDParameter->Output, PIDParameter->OutputLimit);
    PIDParameter->LastErr = PIDParameter->NowErr;

    return PIDParameter->Output;
}

float TurnPIAlgorithm(PIDspeedControlStructure *PIDParameter, float RealityValue)
{
    float a = 0.3f;
    float absErr;

    if ((fabs(PIDParameter->TargetValue) <= BALANCE_TURN_TARGET_DEAD_ZONE) &&
        (fabs(RealityValue) <= BALANCE_TURN_SPEED_DEAD_ZONE))
    {
        RealityValue = 0.0f;
        PIDParameter->ErrSum *= BALANCE_TURN_ERRSUM_DECAY;
        if (fabs(PIDParameter->ErrSum) <= BALANCE_TURN_ERRSUM_ZERO)
        {
            PIDParameter->ErrSum = 0.0f;
        }
    }

    PIDParameter->ActualValue =
        a * RealityValue + (1.0f - a) * PIDParameter->ActualValue;
    PIDParameter->NowErr = PIDParameter->TargetValue - PIDParameter->ActualValue;
    absErr = fabs(PIDParameter->NowErr);

    if ((PIDParameter->IntelSepaThreshold <= 0.0f) ||
        (absErr <= PIDParameter->IntelSepaThreshold))
    {
        PIDParameter->ErrSum += PIDParameter->NowErr;
        PIDParameter->ErrSum = LimitFloat(PIDParameter->ErrSum, PIDParameter->ErrSum_Limit);
    }

    PIDParameter->Output = PIDParameter->kp * PIDParameter->NowErr +
                           PIDParameter->ki * PIDParameter->ErrSum +
                           PIDParameter->kd * (PIDParameter->NowErr - PIDParameter->LastErr);
    PIDParameter->Output = LimitFloat(PIDParameter->Output, PIDParameter->OutputLimit);
    PIDParameter->LastErr = PIDParameter->NowErr;

    return PIDParameter->Output;
}

KalmanParameter Pitch_KalmanFilter;

void KalmanFilter_Init(void)
{
    /* Initial covariance/noise values are part of the attitude filter tuning. */
    KalmanParameter init = {
        .X_Matrix = {{0.0f}, {2.0f}},
        .K_Matrix = {{0.0f}, {0.0f}},
        .P_Matrix = {{0.15f, 0.0f}, {0.0f, 0.15f}},
        .Q_Matrix = {{0.04f, 0.0f}, {0.0f, 0.005f}},
        .R = 0.28f,
        .dt = 0.03f
    };

    memcpy(&Pitch_KalmanFilter, &init, sizeof(KalmanParameter));
}

float KalmanFilter(KalmanParameter *KalmanAngle, float Acc_Angle, float Gyro_y)
{
    float temp00;
    float temp01;
    float measureErr;
    float invS;

    /* Predict angle with gyro integration; X[1] estimates gyro bias. */
    KalmanAngle->X_Matrix[0][0] = KalmanAngle->X_Matrix[0][0] +
                                  (Gyro_y - KalmanAngle->X_Matrix[1][0]) * KalmanAngle->dt;

    KalmanAngle->P_Matrix[0][0] = KalmanAngle->P_Matrix[0][0] -
                                  (KalmanAngle->P_Matrix[1][0] + KalmanAngle->P_Matrix[0][1]) * KalmanAngle->dt +
                                  KalmanAngle->P_Matrix[1][1] * (KalmanAngle->dt * KalmanAngle->dt) +
                                  KalmanAngle->dt * KalmanAngle->Q_Matrix[0][0];
    KalmanAngle->P_Matrix[0][1] = KalmanAngle->P_Matrix[0][1] - KalmanAngle->P_Matrix[1][1] * KalmanAngle->dt;
    KalmanAngle->P_Matrix[1][0] = KalmanAngle->P_Matrix[1][0] - KalmanAngle->P_Matrix[1][1] * KalmanAngle->dt;
    KalmanAngle->P_Matrix[1][1] = KalmanAngle->P_Matrix[1][1] + KalmanAngle->dt * KalmanAngle->Q_Matrix[1][1];

    /* Correct the prediction with accelerometer angle measurement. */
    invS = 1.0f / (KalmanAngle->P_Matrix[0][0] + KalmanAngle->R);
    KalmanAngle->K_Matrix[0][0] = KalmanAngle->P_Matrix[0][0] * invS;
    KalmanAngle->K_Matrix[1][0] = KalmanAngle->P_Matrix[1][0] * invS;

    measureErr = Acc_Angle - KalmanAngle->X_Matrix[0][0];
    KalmanAngle->X_Matrix[0][0] = KalmanAngle->X_Matrix[0][0] + KalmanAngle->K_Matrix[0][0] * measureErr;
    KalmanAngle->X_Matrix[1][0] = KalmanAngle->X_Matrix[1][0] + KalmanAngle->K_Matrix[1][0] * measureErr;

    temp00 = KalmanAngle->P_Matrix[0][0];
    temp01 = KalmanAngle->P_Matrix[0][1];

    KalmanAngle->P_Matrix[0][0] = KalmanAngle->P_Matrix[0][0] - KalmanAngle->K_Matrix[0][0] * temp00;
    KalmanAngle->P_Matrix[0][1] = KalmanAngle->P_Matrix[0][1] - KalmanAngle->K_Matrix[0][0] * temp01;
    KalmanAngle->P_Matrix[1][0] = KalmanAngle->P_Matrix[1][0] - KalmanAngle->K_Matrix[1][0] * temp00;
    KalmanAngle->P_Matrix[1][1] = KalmanAngle->P_Matrix[1][1] - KalmanAngle->K_Matrix[1][0] * temp01;

    return KalmanAngle->X_Matrix[0][0];
}

void delay_us(uint32_t nus)
{
    uint32_t ticks;
    uint32_t told;
    uint32_t tnow;
    uint32_t tcnt = 0U;
    uint32_t reload = SysTick->LOAD;

    /* Busy-wait microsecond delay for software buses; suspend scheduling for accuracy. */
    ticks = nus * (SystemCoreClock / 1000000U);
    told = SysTick->VAL;
    vTaskSuspendAll();
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks)
            {
                break;
            }
        }
    }
    (void)xTaskResumeAll();
}
