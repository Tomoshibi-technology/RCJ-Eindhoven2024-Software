#ifndef __CALC_H
#define __CALC_H

#include <stdint.h>
#include "main.h"

class CALC{
    private:

    public:
		int16_t similarityPeak(uint8_t ledNum, int16_t refference, uint16_t widthAngle, uint16_t center, uint16_t widthHue);
		int16_t similarityRise(uint8_t ledNum, int16_t refference, uint16_t widthAngle, uint16_t center, uint16_t widthHue);
		int16_t similarityFall(uint8_t ledNum, int16_t refference, uint16_t widthAngle, uint16_t center, uint16_t widthHue);
		int16_t similarityNormal(uint8_t ledNum, int16_t refference, uint16_t width);
		uint8_t range(int8_t ledNum, int16_t refference, uint16_t width);
		int16_t calcRotation(int16_t targetRotation, int16_t nowRotation);
};

#endif
