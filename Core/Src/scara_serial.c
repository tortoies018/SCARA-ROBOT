#include "scara.h"
#include "usart.h"
#include <string.h>

/* ==================== DMA 收发缓冲区 ==================== */
#define DMA_RX_BUF_SIZE 64
#define DMA_TX_BUF_SIZE 128

static uint8_t dma_rx_buf[DMA_RX_BUF_SIZE];          /* DMA 接收缓冲区 */
static uint8_t dma_tx_buf[DMA_TX_BUF_SIZE];           /* DMA 发送缓冲区 */
static volatile uint8_t tx_busy = 0;                   /* DMA 发送忙标志 */

/* ==================== 环形 FIFO 缓冲区 ==================== */
static volatile uint8_t rx_fifo[RX_FIFO_SIZE];
static volatile uint16_t fifo_head = 0;
static volatile uint16_t fifo_tail = 0;

static char line_buf[SERIAL_BUF_SIZE];
static volatile uint16_t line_idx = 0;

/* ==================== UART 发送 (DMA) ==================== */
static void uart_send(const char *data, uint16_t len)
{
    if (len == 0) return;
    if (len > DMA_TX_BUF_SIZE) len = DMA_TX_BUF_SIZE;

    while (tx_busy) { }                      /* 等待上次发送完成 */
    memcpy(dma_tx_buf, data, len);            /* 复制到静态缓冲区 */
    tx_busy = 1;
    HAL_UART_Transmit_DMA(&huart1, dma_tx_buf, len);
}

/* ==================== 响应发送 ==================== */
void SCARA_SendResponse(const char *str)
{
    if (str) uart_send(str, (uint16_t)strlen(str));
}

/* ==================== UART 接收初始化 (DMA + IDLE) ==================== */
void SCARA_UART_InitRx(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, dma_rx_buf, DMA_RX_BUF_SIZE);
}

/* ==================== UART 接收回调 (ISR) ==================== */
/* DMA + IDLE 检测: 串口空闲时触发, 将收到的字节压入 FIFO */
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
    else if (cmd[0] == 'V')
    {
        const char *p = cmd + 1;
        uint32_t speed = (uint32_t)parse_int(&p);
        SCARA_SetSpeed(speed);
        SCARA_SendResponse("OK SPEED\r\n");
    }
    else if (cmd[0] == 'E')
    {
        const char *p = cmd + 1;
        int32_t val = parse_int(&p);
        if (val != 0) { SCARA_EnableMotors(); SCARA_SendResponse("OK ENABLED\r\n"); }
        else          { SCARA_DisableMotors(); SCARA_SendResponse("OK DISABLED\r\n"); }
    }
    else
    {
        SCARA_SendResponse("ER UNKNOWN\r\n");
    }
}

/* ==================== 串口处理 (主循环调用) ==================== */
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

/* ==================== HAL DMA 回调 ==================== */

/* DMA + IDLE 接收完成: 收到一帧数据后自动调用 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        for (uint16_t i = 0; i < Size; i++)
            SCARA_UART_RxCallback(dma_rx_buf[i]);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, dma_rx_buf, DMA_RX_BUF_SIZE);
    }
}

/* DMA 发送完成回调: 释放发送忙标志 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
        tx_busy = 0;
}
