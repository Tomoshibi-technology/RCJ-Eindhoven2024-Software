#ifndef __STS_H
#define __STS_H

#include "main.h"
#include "stm32f4xx_hal_conf.h"
#include "stm32f4xx_it.h"
#include <stdint.h>

class STS
{
public:
    STS(UART_HandleTypeDef *uart, uint8_t ID);

    void move(int16_t speed, int16_t goal_position, int16_t now_position);
    void send();
    int16_t calculate_position(int16_t now_position);

private:
    UART_HandleTypeDef *UART;
    uint8_t ID;
    int8_t rotation = 0;
    int16_t position = 0;
    int16_t pre_position = 0;
};

#endif
