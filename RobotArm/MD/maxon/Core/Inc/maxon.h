#ifndef __MAXON_H
#define __MAXON_H

#include "main.h"
#include <stdint.h>

class MAXON
{
public:
    MAXON(TIM_HandleTypeDef *htim, uint32_t timChannel, GPIO_TypeDef *gpio, uint16_t shdnPin);

    void move(int16_t speed);

private:
    TIM_HandleTypeDef *HTIM;
    uint32_t channel;
    GPIO_TypeDef *GPIO;
    uint16_t shdn;
};

#endif
