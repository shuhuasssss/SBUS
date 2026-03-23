/**
 * @file  sbus.c
 * @brief SBUS 协议接收模块 — DMA 循环接收 + IDLE 中断
 *
 * ========================================
 *  使用说明
 * ========================================
 *
 * 1. 初始化（main.c 中调用一次）
 *
 *    sbus_init(&huart1);
 *
 * 2. 读取遥控器数据（随时调用，返回值拷贝，中断安全）
 *
 *    RC_ctrl_t rc = sbus_get_rc();
 *    int16_t left_x  = rc.Ch1;   // 左摇杆 左右   -1024 ~ +1023
 *    int16_t left_y  = rc.Ch2;   // 左摇杆 前后   -1024 ~ +1023
 *    int16_t right_y = rc.Ch3;   // 右摇杆 前后   -1024 ~ +1023
 *    int16_t right_x = rc.Ch4;   // 右摇杆 左右   -1024 ~ +1023
 *    int16_t sw_a    = rc.SA;    // 三段开关 A
 *    int16_t sw_b    = rc.SB;    // 三段开关 B
 *    // SC SD SE SF SG SH — 其余开关
 *    // LD RD — 左右拨轮
 *    // LS RS — 左右滑块
 *
 * 3. 检测连接状态（推荐在读数据前判断）
 *
 *    if (sbus_is_connected()) {
 *        // 遥控器在线，正常使用 rc.Ch1 等
 *    } else {
 *        // 遥控器断开，建议归零或停车
 *    }
 *
 * 4. 完整示例
 *
 *    sbus_init(&huart1);
 *
 *    while (1) {
 *        RC_ctrl_t rc = sbus_get_rc();
 *
 *        if (sbus_is_connected()) {
 *            motor_speed = rc.Ch3 * 4;
 *        } else {
 *            motor_speed = 0;
 *        }
 *
 *        HAL_Delay(5);
 *    }
 *
 * 5. 注意事项
 *
 *    - sbus_get_rc() 返回值拷贝，不需要加 volatile 或关中断
 *    - 遥控器断开 60ms 后自动归零（可通过 SBUS_TIMEOUT_MS 宏修改）
 *    - 中断处理已在 stm32f4xx_it.c 中配置好，用户不需要改动
 *    - 默认校验帧尾 0x00；DJI 等变种协议可在 sbus.h 中
 *      定义 SBUS_STRICT_TAIL 为 0 来跳过帧尾校验
 *
 * ========================================
 *
 *  Robustness features:
 *  - Frame head + tail validation (tail check configurable)
 *  - Failsafe / frame-lost flag parsing
 *  - Failsafe timeout (auto-clear all channels on signal loss)
 *  - IRQ-safe data access (disable IRQ around reads)
 *  - State machine bounds check
 *  - DMA error recovery
 *  - Parser state reset on re-init
 */
#include "sbus.h"
#include <string.h>

/* ---- DMA handle ---- */
DMA_HandleTypeDef hdma_usart1_rx;

/* ---- Receive buffer ---- */
static uint8_t sbus_rx_buf[SBUS_RX_BUF_NUM];

/* ---- Frame reassembly state machine ---- */
enum { SBUS_WAIT_START, SBUS_IN_FRAME };
static uint8_t  frame[25];
static volatile uint8_t frame_idx;
static volatile uint8_t sbus_state;

/* ---- Parsed result (written by ISR, read by main loop) ---- */
static volatile RC_ctrl_t rc_ctrl;

/* ---- Connection tracking ---- */
static volatile uint32_t sbus_last_tick;

/* ---- DMA NDTR tracking ---- */
static uint16_t last_ndtr;

/* ============================================================ */
/*  Bit-unpack one 25-byte SBUS frame                           */
/* ============================================================ */
static void sbus_to_rc(const uint8_t *buf, volatile RC_ctrl_t *rc)
{
    /* Validate frame head; tail check optional for DJI/variant SBUS */
    if (buf[0] != SBUS_FRAME_HEAD)
        return;
#if SBUS_STRICT_TAIL
    if (buf[24] != SBUS_FRAME_TAIL)
        return;
#endif

    /* Ch1 ~ Ch4 (joystick axes) */
    rc->Ch1 = (((uint16_t)buf[1]) | ((uint16_t)buf[2] << 8)) & 0x07FF;
    rc->Ch2 = ((uint16_t)((buf[2] & 0xf8) >> 3)) | (((uint16_t)(buf[3] & 0x3f)) << 5);
    rc->Ch3 = ((uint16_t)((buf[3] & 0xc0) >> 6)) | (((uint16_t)buf[4] << 2)) | (((uint16_t)(buf[5] & 0x01)) << 10);
    rc->Ch4 = ((uint16_t)((buf[5] & 0xfe) >> 1)) | (((uint16_t)(buf[6] & 0x0f)) << 7);

    /* SA ~ SH (switches) */
    rc->SA = ((uint16_t)((buf[6] & 0xf0) >> 4)) | (((uint16_t)(buf[7] & 0x7f)) << 4);
    rc->SB = ((uint16_t)((buf[7] & 0x80) >> 7)) | (((uint16_t)buf[8] << 1)) | (((uint16_t)(buf[9] & 0x03)) << 9);
    rc->SC = ((uint16_t)((buf[9] & 0xfc) >> 2)) | (((uint16_t)(buf[10] & 0x1f)) << 6);
    rc->SD = ((uint16_t)((buf[10] & 0xe0) >> 5)) | (((uint16_t)buf[11] << 3));
    rc->SE = ((uint16_t)buf[12]) | (((uint16_t)(buf[13] & 0x07)) << 8);
    rc->SF = ((uint16_t)((buf[13] & 0xf8) >> 3)) | (((uint16_t)(buf[14] & 0x3f)) << 5);
    rc->SG = ((uint16_t)((buf[14] & 0xc0) >> 6)) | (((uint16_t)buf[15] << 2)) | (((uint16_t)(buf[16] & 0x01)) << 10);
    rc->SH = ((uint16_t)((buf[16] & 0xfe) >> 1)) | (((uint16_t)(buf[17] & 0x0f)) << 7);

    /* LD, RD, LS, RS (dials / sliders) */
    rc->LD = ((uint16_t)((buf[17] & 0xf0) >> 4)) | (((uint16_t)(buf[18] & 0x7f)) << 4);
    rc->RD = ((uint16_t)((buf[18] & 0x80) >> 7)) | (((uint16_t)buf[19] << 1)) | (((uint16_t)(buf[20] & 0x03)) << 9);
    rc->LS = ((uint16_t)((buf[20] & 0xfc) >> 2)) | (((uint16_t)(buf[21] & 0x1f)) << 6);
    rc->RS = ((uint16_t)((buf[21] & 0xe0) >> 5)) | (((uint16_t)buf[22] << 3));

    /* Offset channels to -1024 ~ +1023 */
    rc->Ch1 -= RC_CH_VALUE_OFFSET;
    rc->Ch2 -= RC_CH_VALUE_OFFSET;
    rc->Ch3 -= RC_CH_VALUE_OFFSET;
    rc->Ch4 -= RC_CH_VALUE_OFFSET;

    /* Parse flags (byte 23) */
    rc->flags = buf[23];

    /* Update connection timestamp (also volatile) */
    sbus_last_tick = HAL_GetTick();
}

/* ============================================================ */
/*  Stream parser: accumulate bytes into 25-byte frames         */
/* ============================================================ */
static void sbus_parse_stream(const uint8_t *src, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        uint8_t b = src[i];

        switch (sbus_state) {
            case SBUS_WAIT_START:
                if (b == SBUS_FRAME_HEAD) {
                    frame[0] = b;
                    frame_idx = 1;
                    sbus_state = SBUS_IN_FRAME;
                }
                break;

            case SBUS_IN_FRAME:
                if (frame_idx < sizeof(frame)) {
                    frame[frame_idx++] = b;
                } else {
                    /* Overrun — shouldn't happen, but reset to be safe */
                    frame_idx = 0;
                    sbus_state = SBUS_WAIT_START;
                    break;
                }

                if (frame_idx == 25) {
                    sbus_to_rc(frame, &rc_ctrl);
                    /* Always restart scanning — frame may be invalid */
                    frame_idx = 0;
                    sbus_state = SBUS_WAIT_START;
                }
                break;
        }
    }
}

/* ============================================================ */
/*  Public API                                                  */
/* ============================================================ */

void sbus_init(UART_HandleTypeDef *huart)
{
    /* Reset parser state (safe for re-init) */
    frame_idx = 0;
    sbus_state = SBUS_WAIT_START;
    last_ndtr = SBUS_RX_BUF_NUM;
    sbus_last_tick = HAL_GetTick();
    memset((void *)&rc_ctrl, 0, sizeof(rc_ctrl));

    /* DMA controller clock enable */
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* Configure DMA2 Stream2 Channel4 for USART1 RX (circular) */
    hdma_usart1_rx.Instance                 = DMA2_Stream2;
    hdma_usart1_rx.Init.Channel             = DMA_CHANNEL_4;
    hdma_usart1_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode                = DMA_CIRCULAR;
    hdma_usart1_rx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_usart1_rx.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
    hdma_usart1_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
        Error_Handler();
    }

    /* Link DMA to UART handle */
    __HAL_LINKDMA(huart, hdmarx, hdma_usart1_rx);

    /* Enable DMA2 Stream2 interrupt */
    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

    /* Start circular DMA reception */
    HAL_UART_Receive_DMA(huart, sbus_rx_buf, SBUS_RX_BUF_NUM);

    /* Enable USART1 IDLE line interrupt */
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
}

void sbus_idle_handler(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma)
{
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(huart);

        uint16_t curr_ndtr = hdma->Instance->NDTR;

        /* Guard: if NDTR is out of range, reset */
        if (curr_ndtr > SBUS_RX_BUF_NUM) {
            last_ndtr = SBUS_RX_BUF_NUM;
            return;
        }

        uint16_t rx_len = (last_ndtr >= curr_ndtr) ?
                          (last_ndtr - curr_ndtr) :
                          (SBUS_RX_BUF_NUM - curr_ndtr + last_ndtr);
        last_ndtr = curr_ndtr;

        /* Guard: skip if nothing received */
        if (rx_len == 0 || rx_len > SBUS_RX_BUF_NUM)
            return;

        uint16_t tail = SBUS_RX_BUF_NUM - curr_ndtr;

        if (tail + rx_len <= SBUS_RX_BUF_NUM) {
            sbus_parse_stream(&sbus_rx_buf[tail], rx_len);
        } else {
            uint16_t first = SBUS_RX_BUF_NUM - tail;
            sbus_parse_stream(&sbus_rx_buf[tail], first);
            sbus_parse_stream(&sbus_rx_buf[0], rx_len - first);
        }
    }
}

void sbus_dma_error_handler(DMA_HandleTypeDef *hdma)
{
    if (hdma == &hdma_usart1_rx)
    {
        /* Abort and restart DMA on error */
        HAL_DMA_Abort(hdma);

        /* Reset parser state */
        frame_idx = 0;
        sbus_state = SBUS_WAIT_START;
        last_ndtr = SBUS_RX_BUF_NUM;

        /* Restart reception */
        extern UART_HandleTypeDef huart1;
        HAL_UART_Receive_DMA(&huart1, sbus_rx_buf, SBUS_RX_BUF_NUM);
    }
}

RC_ctrl_t sbus_get_rc(void)
{
    RC_ctrl_t snapshot;

    __disable_irq();
    if ((HAL_GetTick() - sbus_last_tick) > SBUS_TIMEOUT_MS) {
        /* Failsafe: zero everything */
        memset((void *)&rc_ctrl, 0, sizeof(rc_ctrl));
    }
    snapshot = rc_ctrl;
    __enable_irq();

    return snapshot;
}

uint8_t sbus_is_connected(void)
{
    __disable_irq();
    uint32_t elapsed = HAL_GetTick() - sbus_last_tick;
    __enable_irq();

    return (elapsed <= SBUS_TIMEOUT_MS);
}
