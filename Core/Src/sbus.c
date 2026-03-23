/**
 * @file  sbus.c
 * @brief SBUS protocol receiver — DMA circular + IDLE interrupt
 *
 * Robustness features:
 *  - Frame head + tail validation
 *  - Failsafe / frame-lost flag parsing
 *  - Failsafe timeout (auto-clear channels on signal loss)
 *  - State machine bounds check + re-sync on bad frame
 */
#include "sbus.h"

/* ---- DMA handle ---- */
DMA_HandleTypeDef hdma_usart1_rx;

/* ---- Receive buffer ---- */
static uint8_t sbus_rx_buf[SBUS_RX_BUF_NUM];

/* ---- Frame reassembly state machine ---- */
enum { SBUS_WAIT_START, SBUS_IN_FRAME };
static uint8_t  frame[25];
static uint8_t  frame_idx;
static uint8_t  sbus_state;

/* ---- Parsed result ---- */
static RC_ctrl_t rc_ctrl;

/* ---- Failsafe timeout ---- */
static volatile uint32_t sbus_last_tick;   /* HAL_GetTick() of last valid frame */
#define SBUS_TIMEOUT_MS  100               /* no frame for 100ms = disconnect */

/* ============================================================ */
/*  Bit-unpack one 25-byte SBUS frame                           */
/* ============================================================ */
static void sbus_to_rc(const uint8_t *buf, RC_ctrl_t *rc)
{
    /* Validate frame head and tail */
    if (buf[0] != SBUS_FRAME_HEAD || buf[24] != SBUS_FRAME_TAIL)
        return;

    rc->Start = buf[0];

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

    /* Mark data as valid */
    rc->valid = 1;

    /* Update connection timestamp */
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
    hdma_usart1_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
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

    /* Initialize timestamp */
    sbus_last_tick = HAL_GetTick();
}

void sbus_idle_handler(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma)
{
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(huart);

        static uint16_t last_ndtr = SBUS_RX_BUF_NUM;
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

const RC_ctrl_t *sbus_get_rc(void)
{
    /* Failsafe: if no frame received recently, clear data */
    if ((HAL_GetTick() - sbus_last_tick) > SBUS_TIMEOUT_MS) {
        rc_ctrl.valid = 0;
        rc_ctrl.Ch1 = 0;
        rc_ctrl.Ch2 = 0;
        rc_ctrl.Ch3 = 0;
        rc_ctrl.Ch4 = 0;
    }
    return &rc_ctrl;
}

uint8_t sbus_is_connected(void)
{
    return rc_ctrl.valid && ((HAL_GetTick() - sbus_last_tick) <= SBUS_TIMEOUT_MS);
}
