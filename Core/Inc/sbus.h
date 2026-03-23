#ifndef SBUS_H
#define SBUS_H

#include "main.h"

/* ---- Configuration ---- */
#define SBUS_RX_BUF_NUM    50
#define RC_FRAME_LENGTH    25
#define RC_CH_VALUE_OFFSET 1024u
#define SBUS_FRAME_HEAD    0x0F
#define SBUS_FRAME_TAIL    0x00

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

/* ---- Public API ---- */

/* Call once after UART+DMA init */
void sbus_init(UART_HandleTypeDef *huart);

/* Called from USART1_IRQHandler — do not call directly */
void sbus_idle_handler(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma);

/* Called from DMA2_Stream2_IRQHandler */
void sbus_dma_error_handler(DMA_HandleTypeDef *hdma);

/* Get snapshot of latest RC data (IRQ-safe copy) */
RC_ctrl_t sbus_get_rc(void);

/* Check if RC link is alive */
uint8_t sbus_is_connected(void);

#endif /* SBUS_H */
