#include "calc.h"
#include "math.h"

int16_t CALC::similarityPeak(uint8_t ledNum, int16_t refference, uint16_t widthAngle, uint16_t widthHue, uint16_t center){
	uint16_t angle = ledNum * 360 / 16 + 360 / 32;
	uint16_t diff = abs(angle - refference);
	if (diff > 180) {
		diff = 360 - diff;
	}

	if (diff >= widthAngle) {
		return 0;
	}

	float angleRad = angle * M_PI / 180.0;
	float refferenceRad = refference * M_PI / 180.0;
	float widthAngleRad = widthAngle * M_PI / 180.0;

	float result = cos(angleRad - refferenceRad);
	float zeroPoint = cos(widthAngleRad / 2);
	float normalizedResult = (result - zeroPoint) / (1 - zeroPoint);

	int16_t output = center + (normalizedResult - 1) * widthHue;

	return output;
}

int16_t CALC::similarityRise(uint8_t ledNum, int16_t refference, uint16_t widthAngle, uint16_t widthHue, uint16_t center){
	uint16_t angle = ledNum * 360 / 16 + 360 / 32;
	int16_t diff = angle - refference;
	if (diff > 180) {
		diff = diff - 360;
	}
	if (diff < -180) {
		diff = diff + 360;
	}

	if (abs(diff) >= widthAngle) {
		return 0;
	}

	float angleRad = angle * M_PI / 180.0;
	float refferenceRad = refference * M_PI / 180.0;
	float widthAngleRad = widthAngle * M_PI / 180.0;

	float result = cos(angleRad - refferenceRad);
	float zeroPoint = cos(widthAngleRad / 2);
	float normalizedResult = (result - zeroPoint) / (1 - zeroPoint);

    int16_t output = 0;
    if(diff > 0){
        output = center - (normalizedResult - 1) * widthHue;
    } else {
        output = center + (normalizedResult - 1) * widthHue;
    }

	return output;
}

int16_t CALC::similarityFall(uint8_t ledNum, int16_t refference, uint16_t widthAngle, uint16_t widthHue, uint16_t center){
	uint16_t angle = ledNum * 360 / 16 + 360 / 32;
	int16_t diff = angle - refference;
	if (diff > 180) {
		diff = diff - 360;
	}
	if (diff < -180) {
		diff = diff + 360;
	}

	if (abs(diff) >= widthAngle) {
		return 0;
	}

	float angleRad = angle * M_PI / 180.0;
	float refferenceRad = refference * M_PI / 180.0;
	float widthAngleRad = widthAngle * M_PI / 180.0;

	float result = cos(angleRad - refferenceRad);
	float zeroPoint = cos(widthAngleRad / 2);
	float normalizedResult = (result - zeroPoint) / (1 - zeroPoint);

    int16_t output = 0;
    if(diff > 0){
        output = center + (normalizedResult - 1) * widthHue;
    } else {
        output = center - (normalizedResult - 1) * widthHue;
    }

	return output;
}

int16_t CALC::similarityNormal(uint8_t ledNum, int16_t refference, uint16_t width){
	uint16_t angle = ledNum * 360 / 16 + 360 / 32;
	uint16_t diff = abs(angle - refference);
    if (diff > 180) {
        diff = 360 - diff;
    }

    if (diff >= width) {
        return 0;
    }

    float angleRad = angle * M_PI / 180.0;
    float refferenceRad = refference * M_PI / 180.0;
    float widthRad = width * M_PI / 180.0;

    float result = cos(angleRad - refferenceRad);
    float zeroPoint = cos(widthRad / 2);
    float output = (result - zeroPoint) / (1 - zeroPoint);

    return output * 255;
}

uint8_t CALC::range(int8_t ledNum, int16_t refference, uint16_t width){
	uint16_t angle = ledNum * 360 / 16 + 360 / 32;
	uint16_t diff = abs(angle - refference);
    if (diff > 180) {
        diff = 360 - diff;
    }

    if (diff >= width / 2) {
        return 0;
    }else{
    	return 1;
    }
}
