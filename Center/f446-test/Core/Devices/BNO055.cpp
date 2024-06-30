/*
 * bno055.cpp
 *
 *  Created on: Jun 30, 2024
 *      Author: jumpei
 */


#include "BNO055.h"
#include "main.h"

BNO055::BNO055(I2C_HandleTypeDef* i2cHandle, uint8_t address)
    : _i2cHandle(i2cHandle), _address(address << 1) {}

bool BNO055::begin() {
    uint8_t configMode = 0x00; // CONFIGMODE
    if (write(0x3D, &configMode, 1) != HAL_OK) {
        return false;
    }
    HAL_Delay(30);

    uint8_t ndofMode = 0x0C; // NDOF mode
    if (write(0x3D, &ndofMode, 1) != HAL_OK) {
        return false;
    }
    HAL_Delay(30);

    return true;
}

void BNO055::setMode(uint8_t mode) {
    write(0x3D, &mode, 1);
    HAL_Delay(30);
}

void BNO055::getEulerAngles(float& heading, float& roll, float& pitch) {
    uint8_t eulerData[6];
    if (read(0x1A, eulerData, 6) == HAL_OK) {
        int16_t headingRaw = ((int16_t)eulerData[1] << 8) | eulerData[0];
        int16_t rollRaw = ((int16_t)eulerData[3] << 8) | eulerData[2];
        int16_t pitchRaw = ((int16_t)eulerData[5] << 8) | eulerData[4];

        heading = headingRaw / 16.0;
        roll = rollRaw / 16.0;
        pitch = pitchRaw / 16.0;
    }
}

HAL_StatusTypeDef BNO055::write(uint8_t reg, uint8_t* data, uint16_t size) {
    return HAL_I2C_Mem_Write(_i2cHandle, _address, reg, I2C_MEMADD_SIZE_8BIT, data, size, HAL_MAX_DELAY);
}

HAL_StatusTypeDef BNO055::read(uint8_t reg, uint8_t* data, uint16_t size) {
    return HAL_I2C_Mem_Read(_i2cHandle, _address, reg, I2C_MEMADD_SIZE_8BIT, data, size, HAL_MAX_DELAY);
}



