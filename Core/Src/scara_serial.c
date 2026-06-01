#include "scara.h"
#include "usart.h"

/* ==================== 环形 FIFO 缓冲区 ==================== */
static uint8_t uart_rx_byte;                    /* UART 单字节接收缓冲 */
static volatile uint8_t rx_fifo[RX_FIFO_SIZE];  /* 接收 FIFO */
static volatile uint16_t fifo_head = 0;          /* FIFO 写指针 */
static volatile uint16_t fifo_tail = 0;          /* FIFO 读指针 */

static char line_buf[SERIAL_BUF_SIZE];          /* 命令行积累缓冲 */
static volatile uint16_t line_idx = 0;           /* 命令行指针 */

/* ==================== UART 发送 ==================== */
static void uart_send(const char *data, uint16_t len)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)data, len, HAL_MAX_DELAY);
}

/* ==================== 响应发送 ==================== */
void SCARA_SendResponse(const char *str)
{
    if (str) uart_send(str, (uint16_t)strlen(str));
}

/* ==================== UART 接收初始化 ==================== */
void SCARA_UART_InitRx(void)
{
    HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
}

/* ==================== UART 接收回调 (ISR 中调用) ==================== */
/* 将 UART 接收到的字节压入环形 FIFO，永不丢失 */
void SCARA_UART_RxCallback(uint8_t byte)
{
    uint16_t next = (fifo_head + 1) % RX_FIFO_SIZE;
    if (next != fifo_tail)
    {
        rx_fifo[fifo_head] = byte;
        fifo_head = next;
    }
}

/* ==================== 整数解析 ==================== */
static int parse_int(const char **p)
{
    while (**p == ' ' || **p == '\t') (*p)++;
    int neg = 0;
    if (**p == '-') { neg = 1; (*p)++; }
    else if (**p == '+') { (*p)++; }
    int val = 0;
    while (**p >= '0' && **p <= '9')
    {
        val = val * 10 + (**p - '0');
        (*p)++;
    }
    return neg ? -val : val;
}

/* ==================== 命令处理 ==================== */
static void process_command(const char *cmd)
{
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
    /* M 指令: 相对移动，参数为 角度1 角度2 速度(°/s) */
    else if (cmd[0] == 'M')
    {
        const char *p = cmd + 1;
        int32_t a1 = parse_int(&p);
        int32_t a2 = parse_int(&p);
        uint32_t spd = (uint32_t)parse_int(&p);
        int32_t d1 = DEG_TO_STEPS(a1);
        int32_t d2 = DEG_TO_STEPS(a2);
        uint32_t speed_hz = DEGS_TO_HZ(spd);
        SCARA_MoveRelative(d1, d2, speed_hz);
    }
    /* A 指令: 绝对移动，参数为 角度1 角度2 速度(°/s) */
    else if (cmd[0] == 'A')
    {
        const char *p = cmd + 1;
        int32_t a1 = parse_int(&p);
        int32_t a2 = parse_int(&p);
        uint32_t spd = (uint32_t)parse_int(&p);
        int32_t s1 = DEG_TO_STEPS(a1);
        int32_t s2 = DEG_TO_STEPS(a2);
        uint32_t speed_hz = DEGS_TO_HZ(spd);
        SCARA_MoveAbsolute(s1, s2, speed_hz);
    }
    /* Q 指令: 查询位置和状态 */
    else if (strcmp(cmd, "Q") == 0 || strcmp(cmd, "STATUS") == 0)
    {
        char buf[64];
        int n = snprintf(buf, sizeof(buf), "POS %ld %ld %s\r\n",
            scara.motor1.current_position, scara.motor2.current_position,
            SCARA_IsBusy() ? "BSY" : "RDY");
        if (n > 0) uart_send(buf, (uint16_t)(n > 63 ? 63 : n));
    }
    /* ! 指令: 紧急停止 */
    else if (strcmp(cmd, "!") == 0 || strcmp(cmd, "STOP") == 0)
    {
        SCARA_Stop();
    }
    /* SP 指令: 设置当前位置 (步数) */
    else if (cmd[0] == 'S' && cmd[1] == 'P')
    {
        const char *p = cmd + 2;
        int32_t s1 = parse_int(&p);
        int32_t s2 = parse_int(&p);
        SCARA_SetPosition(s1, s2);
        SCARA_SendResponse("OK\r\n");
    }
    /* V 指令: 设置默认速度 */
    else if (cmd[0] == 'V')
    {
        const char *p = cmd + 1;
        uint32_t speed = (uint32_t)parse_int(&p);
        SCARA_SetSpeed(speed);
        SCARA_SendResponse("OK SPEED\r\n");
    }
    else
    {
        SCARA_SendResponse("ER UNKNOWN\r\n");
    }
}

/* ==================== 串口处理 (主循环调用) ==================== */
/* 从 FIFO 取出字节，按 \r/\n 切分为命令行，依次处理 */
uint8_t SCARA_ProcessSerial(void)
{
    uint8_t processed = 0;
    while (fifo_tail != fifo_head)
    {
        uint8_t byte = rx_fifo[fifo_tail];
        fifo_tail = (fifo_tail + 1) % RX_FIFO_SIZE;

        if (byte == '\n' || byte == '\r')
        {
            if (line_idx > 0)
            {
                line_buf[line_idx] = '\0';
                process_command(line_buf);
                line_idx = 0;
                processed = 1;
            }
        }
        else if (line_idx < SERIAL_BUF_SIZE - 1)
        {
            line_buf[line_idx++] = (char)byte;
        }
    }
    return processed;
}

/* ==================== HAL UART 接收完成回调 ==================== */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        SCARA_UART_RxCallback(uart_rx_byte);
        HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
    }
}
