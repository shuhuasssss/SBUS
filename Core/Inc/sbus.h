#ifndef SBUS_H
#define SBUS_H

#include "main.h"

#define SBUS_RX_BUF_NUM    50
#define RC_FRAME_LENGTH    25
#define RC_CH_VALUE_OFFSET 1024u

typedef __packed struct {
    int16_t Start;
    int16_t Ch1, Ch2, Ch3, Ch4;
    int16_t SA, SB, SC, SD, SE, SF, SG, SH;
    int16_t LD, RD, LS, RS;
} RC_ctrl_t;

/* Call once after UART+DMA init */
void sbus_init(UART_HandleTypeDef *huart);

/* Called from USART1_IRQHandler */
void sbus_idle_handler(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma);

/* Get pointer to latest parsed data */
const RC_ctrl_t *sbus_get_rc(void);

#endif /* SBUS_H */
