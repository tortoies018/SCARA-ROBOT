#ifndef __SCARA_H
#define __SCARA_H

#include "main.h"
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define STEPS_PER_REV          200
#define MICROSTEPS             8
#define STEPS_PER_MOTOR_REV    (STEPS_PER_REV * MICROSTEPS)
#define PULLEY_RATIO           5
#define STEPS_PER_JOINT_REV    (STEPS_PER_MOTOR_REV * PULLEY_RATIO)

#define ARM1_LENGTH_MM         150.0f
#define ARM2_LENGTH_MM         100.0f

#define SERVO_UP_CCR           52
#define SERVO_DOWN_CCR         100

#define HOME_SPEED             800
#define HOME_BACKOFF_STEPS     200
#define HOME_APPROACH_SPEED    200
#define HOME_TIMEOUT_MS        10000
#define SERIAL_BUF_SIZE        64
#define TIMER_CLOCK            72000000UL

#define MIN(a,b)    (((a)<(b))?(a):(b))
#define MAX(a,b)    (((a)>(b))?(a):(b))
#define ABS(x)      (((x)>=0)?(x):-(x))
#define SIGN(x)     (((x)>=0)?1:-1)

typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    volatile int32_t remaining_steps;
    volatile int32_t current_position;
    volatile uint8_t busy;
    volatile uint8_t moving_forward;
} MotorAxis;

typedef enum
{
    ROBOT_IDLE,
    ROBOT_MOVING,
    ROBOT_HOMING,
    ROBOT_STOPPED,
    ROBOT_ERROR
} RobotState;

typedef enum
{
    PEN_UP,
    PEN_DOWN
} PenState;

typedef struct
{
    MotorAxis motor1;
    MotorAxis motor2;
    RobotState state;
    PenState pen;
    uint32_t home_start_ms;
    uint8_t home_m1_done;
    uint8_t home_m2_done;
    uint8_t home_approach_phase;
} SCARA_Context;

extern SCARA_Context scara;

void SCARA_Init(void);
void SCARA_SetSpeed(uint32_t spd);
uint8_t SCARA_ProcessSerial(void);
void SCARA_Home(void);
void SCARA_PenUp(void);
void SCARA_PenDown(void);
void SCARA_MoveRelative(int32_t d1, int32_t d2, uint32_t speed);
void SCARA_MoveAbsolute(int32_t s1, int32_t s2, uint32_t speed);
void SCARA_Stop(void);
uint8_t SCARA_IsBusy(void);
void SCARA_GetPosition(int32_t *s1, int32_t *s2);
void SCARA_SetPosition(int32_t s1, int32_t s2);
void SCARA_SendResponse(const char *fmt, ...);

void SCARA_UART_RxCallback(uint8_t byte);
void SCARA_OnTimerPeriodElapsed(TIM_HandleTypeDef *htim);

void motor_start(MotorAxis *axis, int32_t steps, uint32_t speed_hz);
void motor_stop(MotorAxis *axis);

#endif
