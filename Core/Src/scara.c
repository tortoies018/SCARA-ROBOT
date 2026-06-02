#include "scara.h"
#include "tim.h"
#include "usart.h"

SCARA_Context scara;  /* 全局 SCARA 上下文 */

/* ==================== 定时器频率计算 ==================== */
/* 计算 prescaler: 使 ARR 落在 16 位范围内 */
static uint32_t calc_timer_psc(uint32_t speed_hz)
{
    if (speed_hz == 0) return 0;
    uint32_t raw_div = TIMER_CLOCK / speed_hz;
    if (raw_div <= 65536UL) return 0;
    uint32_t psc = raw_div / 65536UL;
    if (psc > 65535) psc = 65535;
    return psc > 0 ? psc - 1 : 0;
}

/* 计算 auto-reload 值 */
static uint32_t calc_timer_arr(uint32_t speed_hz, uint32_t psc)
{
    if (speed_hz == 0) return 0;
    uint32_t timer_clk = TIMER_CLOCK / (psc + 1);
    uint32_t arr = timer_clk / speed_hz;
    if (arr > 65535) arr = 65535;
    if (arr < 2) arr = 2;
    return arr - 1;
}

/* 配置 PWM 频率和占空比 */
static void configure_pwm(MotorAxis *axis, uint32_t speed_hz)
{
    uint32_t psc = calc_timer_psc(speed_hz);
    uint32_t arr = calc_timer_arr(speed_hz, psc);
    __HAL_TIM_SET_PRESCALER(axis->htim, psc);
    __HAL_TIM_SET_AUTORELOAD(axis->htim, arr);
    __HAL_TIM_SET_COMPARE(axis->htim, axis->channel, arr >> 1);  /* 50% 占空比 */
}

/* ==================== 电机启停 ==================== */
/* 启动电机: 设置方向/使能 → 配置 PWM → 生成 UG 加载影子寄存器 → 启动定时器 */
void motor_start(MotorAxis *axis, int32_t steps, uint32_t speed_hz)
{
    if (steps == 0) return;
    uint8_t forward = steps > 0;
    axis->moving_forward = forward;

    /* 设置方向引脚和使能引脚 */
    if (axis->htim->Instance == TIM1)
    {
        HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ENA1_GPIO_Port, ENA1_Pin, GPIO_PIN_RESET);  /* DM542: 低电平使能 */
    }
    else
    {
        //两个电机型号不同，方向定义不同
        HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
        HAL_GPIO_WritePin(ENA2_GPIO_Port, ENA2_Pin, GPIO_PIN_RESET);
    }

    configure_pwm(axis, speed_hz);
    __HAL_TIM_SET_COUNTER(axis->htim, 0);

    /* 生成 UG 事件加载影子寄存器，避免首脉冲频率错误 */
    axis->htim->Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(axis->htim, TIM_FLAG_UPDATE);

    axis->remaining_steps = ABS(steps);
    axis->busy = 1;

    __HAL_TIM_ENABLE_IT(axis->htim, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(axis->htim);

    /* 启用 PWM 通道 + 更新中断 + 计数器 */
    HAL_TIM_PWM_Start_IT(axis->htim, axis->channel);
}

/* 停止电机: 停止 PWM 和中断 (不禁用 ENA，保持锁定转矩) */
void motor_stop(MotorAxis *axis)
{
    HAL_TIM_PWM_Stop_IT(axis->htim, axis->channel);
    axis->remaining_steps = 0;
    axis->busy = 0;
}

/* 使能两个电机 (低电平使能) */
void SCARA_EnableMotors(void)
{
    HAL_GPIO_WritePin(ENA1_GPIO_Port, ENA1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ENA2_GPIO_Port, ENA2_Pin, GPIO_PIN_RESET);
}

/* 禁用两个电机 (高电平禁能) */
void SCARA_DisableMotors(void)
{
    HAL_GPIO_WritePin(ENA1_GPIO_Port, ENA1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ENA2_GPIO_Port, ENA2_Pin, GPIO_PIN_SET);
}

/* ==================== 初始化 ==================== */
void SCARA_Init(void)
{
    memset(&scara, 0, sizeof(scara));
    scara.motor1.htim = &htim1;
    scara.motor1.channel = TIM_CHANNEL_1;
    scara.motor2.htim = &htim3;
    scara.motor2.channel = TIM_CHANNEL_1;
    scara.state = ROBOT_IDLE;
    scara.pen = PEN_UP;
    scara.default_speed = DEFAULT_SPEED;

    /* 运行时配置 TIM4 → 50Hz 舵机 PWM */
    __HAL_TIM_SET_PRESCALER(&htim4, 1439);
    __HAL_TIM_SET_AUTORELOAD(&htim4, 999);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, SERVO_UP_CCR);
    htim4.Instance->EGR = TIM_EGR_UG;  /* 加载影子寄存器 */
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

    /* 电机默认使能 */
    SCARA_EnableMotors();

    /* 启动 UART 中断接收 */
    SCARA_UART_InitRx();

    SCARA_SendResponse("SCARA READY\r\n");
}

/* ==================== 回零 ==================== */
void SCARA_Home(void)
{
    if (scara.state == ROBOT_HOMING || scara.state == ROBOT_MOVING) return;
    scara.state = ROBOT_HOMING;
    scara.home_m1_done = 0;
    scara.home_m2_done = 0;
    scara.home_approach_phase = 0;
    scara.home_start_ms = HAL_GetTick();

    /* 清零当前位置，向负方向搜索极限位置 */
    scara.motor1.current_position = 0;
    scara.motor2.current_position = 0;

    motor_start(&scara.motor1, -2000, HOME_SPEED);
    motor_start(&scara.motor2, -2000, HOME_SPEED);

    SCARA_SendResponse("OK HOME\r\n");
}

/* ==================== 舵机控制 ==================== */
void SCARA_PenUp(void)
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, SERVO_UP_CCR);
    HAL_Delay(300);  /* 等待舵机到位 */
    scara.pen = PEN_UP;
}

void SCARA_PenDown(void)
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, SERVO_DOWN_CCR);
    HAL_Delay(300);
    scara.pen = PEN_DOWN;
}

/* ==================== 运动控制 ==================== */
/* DDA 比例速度协调: 两电机速度按步数比例分配，保证同时到达 */
void SCARA_MoveRelative(int32_t d1, int32_t d2, uint32_t speed)
{
    if (scara.state == ROBOT_HOMING || scara.state == ROBOT_MOVING)
    {
        SCARA_SendResponse("ER BUSY\r\n");
        return;
    }
    if (scara.state == ROBOT_STOPPED)
    {
        SCARA_SendResponse("ER STOPPED\r\n");
        return;
    }
    if (d1 == 0 && d2 == 0)
    {
        SCARA_SendResponse("OK\r\n");
        return;
    }

    if (speed == 0) speed = scara.default_speed;
    if (speed < MIN_SPEED) speed = MIN_SPEED;
    if (speed > MAX_SPEED) speed = MAX_SPEED;

    int32_t a1 = ABS(d1), a2 = ABS(d2);
    uint32_t max_steps = MAX(a1, a2);
    if (max_steps == 0)
    {
        SCARA_SendResponse("OK\r\n");
        return;
    }

    /* 速度与步数成正比，大行程电机跑的更快，两电机同时停止 */
    uint32_t s1 = (uint32_t)((uint64_t)a1 * speed / max_steps);
    uint32_t s2 = (uint32_t)((uint64_t)a2 * speed / max_steps);
    if (s1 < 50) s1 = 50;
    if (s2 < 50) s2 = 50;

    scara.state = ROBOT_MOVING;
    motor_start(&scara.motor1, d1, s1);
    motor_start(&scara.motor2, d2, s2);

    SCARA_SendResponse("OK MOVE\r\n");
}

void SCARA_MoveAbsolute(int32_t s1, int32_t s2, uint32_t speed)
{
    int32_t d1 = s1 - scara.motor1.current_position;
    int32_t d2 = s2 - scara.motor2.current_position;
    SCARA_MoveRelative(d1, d2, speed);
}

void SCARA_Stop(void)
{
    motor_stop(&scara.motor1);
    motor_stop(&scara.motor2);
    scara.state = ROBOT_STOPPED;
    SCARA_SendResponse("OK STOP\r\n");
}

uint8_t SCARA_IsBusy(void)
{
    return scara.motor1.busy || scara.motor2.busy || scara.state == ROBOT_HOMING;
}

/* ==================== 位置查询 ==================== */
void SCARA_GetPosition(int32_t *s1, int32_t *s2)
{
    *s1 = scara.motor1.current_position;
    *s2 = scara.motor2.current_position;
}

void SCARA_SetPosition(int32_t s1, int32_t s2)
{
    scara.motor1.current_position = s1;
    scara.motor2.current_position = s2;
}

void SCARA_SetSpeed(uint32_t spd)
{
    if (spd < MIN_SPEED) spd = MIN_SPEED;
    if (spd > MAX_SPEED) spd = MAX_SPEED;
    scara.default_speed = spd;
}

uint32_t SCARA_GetSpeed(void)
{
    return scara.default_speed;
}

/* ==================== 定时器中断回调 ==================== */
/* TIM1/TIM3 更新事件: 递减剩余步数，归零时停止电机 */
void SCARA_OnTimerPeriodElapsed(TIM_HandleTypeDef *htim)
{
    MotorAxis *axis = NULL;
    if (htim->Instance == TIM1) axis = &scara.motor1;
    else if (htim->Instance == TIM3) axis = &scara.motor2;
    else return;

    if (axis->remaining_steps > 0)
    {
        if (axis->moving_forward) axis->current_position++;
        else axis->current_position--;
        axis->remaining_steps--;
    }

    if (axis->remaining_steps == 0)
    {
        motor_stop(axis);
        /* 两电机均停止 → 状态切回 IDLE，设标志由主循环发 RDY */
        if (!scara.motor1.busy && !scara.motor2.busy && scara.state == ROBOT_MOVING)
        {
            scara.state = ROBOT_IDLE;
            scara.rdy_pending = 1;
        }
    }
}

/* ==================== HAL 回调 ==================== */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    SCARA_OnTimerPeriodElapsed(htim);
}
