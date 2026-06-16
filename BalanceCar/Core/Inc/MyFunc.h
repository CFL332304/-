#ifndef __MYFUNC_H__
#define __MYFUNC_H__

#include "bspsystem.h"

typedef struct{
    /* Generic PID parameters for the upright angle loop. */
    float kp;
    float ki;
    float kd;
    volatile float TargetValue;
    float ActualValue;
    float NowErr;
    float LastErr;
    float ErrSum;
    float ErrSum_Limit;
    float IntelSepaThreshold;
    float OutputOffset;
    int OutputLimit;
    int Output;
}PID_Para;

typedef struct{
    /* Speed/turn loop PID parameters use float output and integral limits. */
    float TargetValue;
    float ActualValue;
    float kp;
    float ki;
    float kd;
    float NowErr;
    float LastErr;
    float ErrSum;
    float ErrSum_Limit;
    float IntelSepaThreshold;
    float OutputOffset;
    float OutputLimit;
    float Output;
}PIDspeedControlStructure;

typedef struct {
    /* Two-state Kalman filter: angle and gyro bias. */
    float X_Matrix[2][1];
    float P_Matrix[2][2];
    float K_Matrix[2][1];
    float Q_Matrix[2][2];
    float R;
    float dt;
} KalmanParameter;

extern PID_Para PIDVectControl;
extern PIDspeedControlStructure PIDTurnControl;
extern PIDspeedControlStructure PIDspeedControl;
extern KalmanParameter Pitch_KalmanFilter;

void Velocity_Ouput(int Aweel, int Bweel);
int PWM_Map(int PWMValue);
void delay_us(uint32_t nus);
void VelocityBroad_Init(void);
void Queue_Init(void);

void PID_Init(void);
void PID_ResetSpeedState(PIDspeedControlStructure *PIDParameter);
int PIDAlgorithm(PID_Para *PIDParameter, float RealityValue);
int PDAlgorithm(PID_Para *PIDParameter, float RealityValue);
float PIAlgorithm(PIDspeedControlStructure *PIDParameter, float RealityValue);
float TurnPIAlgorithm(PIDspeedControlStructure *PIDParameter, float RealityValue);
int PDAlgorithm_withgyro(PID_Para *PIDParameter, float RealityValue, float gyro);

void KalmanFilter_Init(void);
float KalmanFilter(KalmanParameter *KalmanAngle, float Acc_Angle, float Gyro_y);

#endif
