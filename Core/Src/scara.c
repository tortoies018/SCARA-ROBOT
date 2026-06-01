#include "scara.h"
#include "tim.h"
#include "usart.h"

SCARA_Context scara;

static uint8_t rx_buf[SERIAL_BUF_SIZE];
static volatile uint16_t rx_index = 0;
static volatile uint8_t cmd_ready = 0;
static uint8_t uart_rx_byte;

static uint32_t calc_timer_psc(uint32_t speed_hz);
static uint32_t calc_timer_arr(uint32_t speed_hz, uint32_t psc);
static void configure_pwm(MotorAxis *axis, uint32_t speed_hz);

static void uart_send(const char *data, uint16_t len)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)data, len, HAL_MAX_DELAY);
}

static void configure_pwm(MotorAxis *axis, uint32_t speed_hz)
{
    uint32_t psc = calc_timer_psc(speed_hz);
    uint32_t arr = calc_timer_arr(speed_hz, psc);
    __HAL_TIM_SET_PRESCALER(axis->htim, psc);
    __HAL_TIM_SET_AUTORELOAD(axis->htim, arr);
    __HAL_TIM_SET_COMPARE(axis->htim, axis->channel, arr >> 1);
}

static uint32_t calc_timer_psc(uint32_t speed_hz)
{
    if (speed_hz == 0) return 0;
    uint32_t raw_div = TIMER_CLOCK / speed_hz;
    if (raw_div <= 65536UL) return 0;
    uint32_t psc = raw_div / 65536UL;
    if (psc > 65535) psc = 65535;
    return psc > 0 ? psc - 1 : 0;
}

static uint32_t calc_timer_arr(uint32_t speed_hz, uint32_t psc)
{
    if (speed_hz == 0) return 0;
    uint32_t timer_clk = TIMER_CLOCK / (psc + 1);
    uint32_t arr = timer_clk / speed_hz;
    if (arr > 65535) arr = 65535;
    if (arr < 2) arr = 2;
    return arr - 1;
}

void motor_start(MotorAxis *axis, int32_t steps, uint32_t speed_hz)
{
    if (steps == 0) return;
    uint8_t forward = steps > 0;
    axis->moving_forward = forward;

    if (axis->htim->Instance == TIM1)
    {
        HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ENA1_GPIO_Port, ENA1_Pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ENA2_GPIO_Port, ENA2_Pin, GPIO_PIN_RESET);
    }

    configure_pwm(axis, speed_hz);
    __HAL_TIM_SET_COUNTER(axis->htim, 0);

    __HAL_TIM_DISABLE_IT(axis->htim, TIM_IT_UPDATE);
    axis->htim->Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(axis->htim, TIM_FLAG_UPDATE);

    axis->remaining_steps = ABS(steps);
    axis->busy = 1;

    HAL_TIM_PWM_Start_IT(axis->htim, axis->channel);
}

void motor_stop(MotorAxis *axis)
{
    HAL_TIM_PWM_Stop_IT(axis->htim, axis->channel);
    axis->remaining_steps = 0;
    axis->busy = 0;

    if (axis->htim->Instance == TIM1)
    {
        HAL_GPIO_WritePin(ENA1_GPIO_Port, ENA1_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(ENA2_GPIO_Port, ENA2_Pin, GPIO_PIN_SET);
    }
}

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

    SCARA_PenUp();

    HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);

    SCARA_SendResponse("SCARA READY\r\n");
}

void SCARA_Home(void)
{
    if (scara.state == ROBOT_HOMING || scara.state == ROBOT_MOVING) return;
    scara.state = ROBOT_HOMING;
    scara.home_m1_done = 0;
    scara.home_m2_done = 0;
    scara.home_approach_phase = 0;
    scara.home_start_ms = HAL_GetTick();

    scara.motor1.current_position = 0;
    scara.motor2.current_position = 0;

    motor_start(&scara.motor1, -50000, HOME_SPEED);
    motor_start(&scara.motor2, -50000, HOME_SPEED);

    SCARA_SendResponse("OK HOME\r\n");
}

void SCARA_PenUp(void)
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, SERVO_UP_CCR);
    HAL_Delay(300);
    scara.pen = PEN_UP;
}

void SCARA_PenDown(void)
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, SERVO_DOWN_CCR);
    HAL_Delay(300);
    scara.pen = PEN_DOWN;
}

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
        if (!scara.motor1.busy && !scara.motor2.busy && scara.state == ROBOT_MOVING)
        {
            scara.state = ROBOT_IDLE;
        }
    }
}

void SCARA_SendResponse(const char *fmt, ...)
{
    char buf[128];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (len > 0) uart_send(buf, (uint16_t)(len > 127 ? 127 : len));
}

void SCARA_UART_RxCallback(uint8_t byte)
{
    if (cmd_ready) return;

    if (byte == '\n' || byte == '\r')
    {
        if (rx_index > 0)
        {
            rx_buf[rx_index] = '\0';
            cmd_ready = 1;
            rx_index = 0;
        }
    }
    else if (rx_index < SERIAL_BUF_SIZE - 1)
    {
        rx_buf[rx_index++] = byte;
    }
}

static int parse_int(const char **p)
{
    while (**p == ' ' || **p == '\t') (*p)++;
    int neg = 0;
    if (**p == '-')
    {
        neg = 1;
        (*p)++;
    }
    else if (**p == '+')
    {
        (*p)++;
    }
    int val = 0;
    while (**p >= '0' && **p <= '9')
    {
        val = val * 10 + (**p - '0');
        (*p)++;
    }
    return neg ? -val : val;
}

uint8_t SCARA_ProcessSerial(void)
{
    if (!cmd_ready) return 0;

    const char *cmd = (const char*)rx_buf;
    cmd_ready = 0;

    if (strcmp(cmd, "H") == 0 || strcmp(cmd, "HOME") == 0)
    {
        SCARA_Home();
    }
    else if (strcmp(cmd, "P0") == 0 || strcmp(cmd, "PEN 0") == 0)
    {
        SCARA_PenUp();
        SCARA_SendResponse("OK\r\n");
    }
    else if (strcmp(cmd, "P1") == 0 || strcmp(cmd, "PEN 1") == 0)
    {
        SCARA_PenDown();
        SCARA_SendResponse("OK\r\n");
    }
    else if (cmd[0] == 'M')
    {
        const char *p = cmd + 1;
        int32_t d1 = parse_int(&p);
        int32_t d2 = parse_int(&p);
        uint32_t speed = (uint32_t)parse_int(&p);
        SCARA_MoveRelative(d1, d2, speed);
    }
    else if (cmd[0] == 'A')
    {
        const char *p = cmd + 1;
        int32_t s1 = parse_int(&p);
        int32_t s2 = parse_int(&p);
        uint32_t speed = (uint32_t)parse_int(&p);
        SCARA_MoveAbsolute(s1, s2, speed);
    }
    else if (cmd[0] == 'V')
    {
        const char *p = cmd + 1;
        uint32_t speed = (uint32_t)parse_int(&p);
        SCARA_SetSpeed(speed);
        SCARA_SendResponse("OK SPEED %lu\r\n", speed);
    }
    else if (strcmp(cmd, "Q") == 0 || strcmp(cmd, "STATUS") == 0)
    {
        char buf[64];
        int n = snprintf(buf, sizeof(buf), "POS %ld %ld %s\r\n",
            scara.motor1.current_position, scara.motor2.current_position,
            SCARA_IsBusy() ? "BSY" : "RDY");
        if (n > 0) uart_send(buf, (uint16_t)(n > 63 ? 63 : n));
    }
    else if (strcmp(cmd, "!") == 0 || strcmp(cmd, "STOP") == 0)
    {
        SCARA_Stop();
    }
    else if (cmd[0] == 'S' && cmd[1] == 'P')
    {
        const char *p = cmd + 2;
        int32_t s1 = parse_int(&p);
        int32_t s2 = parse_int(&p);
        SCARA_SetPosition(s1, s2);
        SCARA_SendResponse("OK\r\n");
    }
    else
    {
        SCARA_SendResponse("ER UNKNOWN\r\n");
    }

    return 1;
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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    SCARA_OnTimerPeriodElapsed(htim);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        SCARA_UART_RxCallback(uart_rx_byte);
        HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
    }
}
