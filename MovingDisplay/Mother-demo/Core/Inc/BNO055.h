/*
 * bno055.hpp
 *
 *  Created on: Jun 30, 2024
 *      Author: jumpei
 */

#ifndef DEVICES_BNO055_H_
#define DEVICES_BNO055_H_


#include "stm32f4xx_hal.h"

class BNO055 {
public:
    BNO055(I2C_HandleTypeDef* i2cHandle, uint8_t address = 0x28);

    bool begin();
    void setMode(uint8_t mode);
    void getEulerAngles(float& heading, float& roll, float& pitch);

private:
    I2C_HandleTypeDef* _i2cHandle;
    uint8_t _address;

    HAL_StatusTypeDef write(uint8_t reg, uint8_t* data, uint16_t size);
    HAL_StatusTypeDef read(uint8_t reg, uint8_t* data, uint16_t size);
};




#endif /* DEVICES_BNO055_H_ */
