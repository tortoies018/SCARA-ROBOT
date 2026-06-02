#ifndef __SCARA_H
#define __SCARA_H

#include "main.h"
#include <string.h>
#include <stdio.h>

/* ======================== 机械参数 ======================== */
#define STEPS_PER_REV          200     /* 步进电机每圈步数 */
#define MICROSTEPS             8       /* DM542 细分 */
#define STEPS_PER_MOTOR_REV    (STEPS_PER_REV * MICROSTEPS)  /* 电机轴每圈脉冲 */
#define PULLEY_RATIO           1       /* 直连 (无同步带) */
#define STEPS_PER_JOINT_REV    (STEPS_PER_MOTOR_REV * PULLEY_RATIO)  /* 关节每圈脉冲 */

#define ARM1_LENGTH_MM         110.0f  /* 主动臂 mm */
#define ARM2_LENGTH_MM         220.0f  /* 从动臂 mm */

/* ======================== 舵机 ======================== */
#define SERVO_UP_CCR           52  /* 抬笔 PWM 占空比 (1.04ms @ 50Hz) */
#define SERVO_DOWN_CCR         100 /* 下笔 PWM 占空比 (2.00ms @ 50Hz) */

/* ======================== 回零 ======================== */
#define HOME_SPEED             800     /* 回零搜索速度 (steps/s) */
#define HOME_BACKOFF_STEPS     200     /* 回零后退步数 */
#define HOME_APPROACH_SPEED    200     /* 回零慢速逼近速度 (steps/s) */
#define HOME_TIMEOUT_MS        10000   /* 回零超时 (ms) */

/* ======================== 串口 ======================== */
#define SERIAL_BUF_SIZE        64      /* 命令行缓冲长度 */
#define RX_FIFO_SIZE           128     /* UART 接收 FIFO 大小 */

/* ======================== 电机 ======================== */
#define TIMER_CLOCK            72000000UL  /* 定时器时钟 72MHz */
#define DEFAULT_SPEED_DPS      90          /* 默认速度 (°/s) */
#define MIN_SPEED_DPS          5           /* 最低速度 (°/s) */
#define MAX_SPEED_DPS          720         /* 最高速度 (°/s) */
#define MIN_SPEED_HZ           50          /* 最低步进频率 (Hz) */
#define MAX_SPEED_HZ           20000       /* 最高步进频率 (Hz) */
#define MAX_SPEED              20000       /* 最高步进频率 */

/* ======================== 角度/步进换算 ======================== */
#define DEG_TO_STEPS(d)  ((int32_t)((int64_t)(d) * STEPS_PER_JOINT_REV / 360))
#define DEGS_TO_HZ(d)    ((uint32_t)((uint64_t)(d) * STEPS_PER_JOINT_REV / 360))

/* ======================== 工具宏 ======================== */
#define MIN(a,b)    (((a)<(b))?(a):(b))
#define MAX(a,b)    (((a)>(b))?(a):(b))
#define ABS(x)      (((x)>=0)?(x):-(x))

/* ======================== 电机轴状态 ======================== */
typedef struct
{
    TIM_HandleTypeDef *htim;            /* 定时器句柄 */
    uint32_t channel;                   /* PWM 通道 */
    volatile int32_t remaining_steps;   /* 剩余步数 */
    volatile int32_t current_position;  /* 当前位置 (步数) */
    volatile uint8_t busy;              /* 运动中标志 */
    volatile uint8_t moving_forward;    /* 正转标志 */
} MotorAxis;

/* ======================== 机器人状态 ======================== */
typedef enum
{
    ROBOT_IDLE,     /* 空闲 */
    ROBOT_MOVING,   /* 运动中 */
    ROBOT_HOMING,   /* 回零中 */
    ROBOT_STOPPED,  /* 已停止 */
    ROBOT_ERROR     /* 错误 */
} RobotState;

/* ======================== 笔状态 ======================== */
typedef enum
{
    PEN_UP,     /* 抬笔 */
    PEN_DOWN    /* 下笔 */
} PenState;

/* ======================== SCARA 上下文 ======================== */
typedef struct
{
    MotorAxis motor1;           /* 电机1 (大臂) */
    MotorAxis motor2;           /* 电机2 (小臂) */
    RobotState state;           /* 机器人状态 */
    PenState pen;               /* 笔状态 */
    uint32_t home_start_ms;     /* 回零开始时间戳 */
    uint8_t home_m1_done;       /* 电机1回零完成标志 */
    uint8_t home_m2_done;       /* 电机2回零完成标志 */
    uint8_t home_approach_phase;/* 回零阶段 */
    uint32_t speed_dps;         /* 默认速度 (°/s) */
    volatile uint8_t rdy_pending; /* RDY 待发送标志 */
} SCARA_Context;

extern SCARA_Context scara;     /* 全局 SCARA 上下文 */

/* ======================== 初始化 ======================== */
void SCARA_Init(void);

/* ======================== 电机控制 ======================== */
void motor_start(MotorAxis *axis, int32_t steps, uint32_t speed_hz);
void motor_stop(MotorAxis *axis);

/* ======================== 运动控制 ======================== */
void SCARA_MoveRelative(int32_t d1, int32_t d2, uint32_t speed);
void SCARA_MoveAbsolute(int32_t s1, int32_t s2, uint32_t speed);
void SCARA_Stop(void);
void SCARA_StartContinuous(int32_t steps1, int32_t steps2, uint32_t speed_hz);
void SCARA_StopContinuous(void);
uint8_t SCARA_IsBusy(void);

/* ======================== 位置查询 ======================== */
void SCARA_GetPosition(int32_t *s1, int32_t *s2);
void SCARA_SetPosition(int32_t s1, int32_t s2);
void SCARA_SetSpeed(uint32_t spd);
uint32_t SCARA_GetSpeed(void);

/* ======================== 回零 ======================== */
void SCARA_Home(void);

/* ======================== 舵机 ======================== */
void SCARA_PenUp(void);
void SCARA_PenDown(void);
void SCARA_EnableMotors(void);
void SCARA_DisableMotors(void);

/* ======================== 串口 ======================== */
uint8_t SCARA_ProcessSerial(void);
void SCARA_SendResponse(const char *str);
void SCARA_UART_RxCallback(uint8_t byte);
void SCARA_UART_InitRx(void);

/* ======================== HAL 回调转发 ======================== */
void SCARA_OnTimerPeriodElapsed(TIM_HandleTypeDef *htim);

#endif
