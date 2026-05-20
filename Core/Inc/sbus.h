#ifndef SBUS_H
#define SBUS_H

#include "main.h"

/* ---- Configuration ---- */
#define SBUS_RX_BUF_NUM    50
#define RC_FRAME_LENGTH    25
#define RC_CH_VALUE_OFFSET 1024u
#define SBUS_FRAME_HEAD    0x0F
#define SBUS_FRAME_TAIL    0x00

#ifndef SBUS_STRICT_TAIL
#define SBUS_STRICT_TAIL   0
#endif

/* ---- Channel limits ---- */
#define SBUS_CH_MIN        (-1024)
#define SBUS_CH_MAX        (1023)
#define SBUS_CH_MID        (0)

/* ---- Flags (byte 23) ---- */
#define SBUS_FLAG_FRAME_LOST  (1 << 2)  /* bit 2: frame lost */
#define SBUS_FLAG_FAILSAFE    (1 << 3)  /* bit 3: failsafe active */

/* ---- Failsafe timeout (ms) ---- */
#ifndef SBUS_TIMEOUT_MS
#define SBUS_TIMEOUT_MS  60
#endif

/* ---- Data structure ---- */
typedef __packed struct {
    int16_t Ch1, Ch2, Ch3, Ch4;
    int16_t SA, SB, SC, SD, SE, SF, SG, SH;
    int16_t LD, RD, LS, RS;
    uint8_t flags;       /* frame lost / failsafe */
} RC_ctrl_t;

typedef struct {
    uint32_t idle_count;
    uint32_t dma_error_count;
    uint32_t parsed_bytes;
    uint32_t head_count;
    uint32_t frame_count;
    uint32_t valid_frame_count;
    uint32_t bad_head_count;
    uint32_t bad_tail_count;
    uint32_t timeout_count;
    uint32_t uart_error_count;
    uint16_t last_ndtr;
    uint16_t last_rx_len;
    uint16_t last_old_pos;
    uint8_t last_uart_sr;
    uint8_t last_frame[RC_FRAME_LENGTH];
} SBUS_Debug_t;

extern volatile SBUS_Debug_t g_sbus_debug;

/* ---- Public API ---- */

/* Call once after UART+DMA init */
void sbus_init(UART_HandleTypeDef *huart);

/* Called from USART1_IRQHandler — do not call directly */
void sbus_idle_handler(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma);

/* Called from DMA2_Stream2_IRQHandler */
void sbus_dma_error_handler(DMA_HandleTypeDef *hdma);

/* Called from USART1_IRQHandler on UART framing/noise/overrun errors */
void sbus_uart_error_handler(UART_HandleTypeDef *huart);

/* Get snapshot of latest RC data (IRQ-safe copy) */
RC_ctrl_t sbus_get_rc(void);

/* Check if RC link is alive */
uint8_t sbus_is_connected(void);

#endif /* SBUS_H */
