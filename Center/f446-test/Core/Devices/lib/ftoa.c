/*
 * ftoa.c
 *
 *  Created on: Jun 30, 2024
 *      Author: jumpei
 */

#include "ftoa.h"

void reverse(char *str, int len) {
    int i = 0, j = len - 1, temp;
    while (i < j) {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++;
        j--;
    }
}

int intToStr(int x, char str[], int d) {
    int i = 0;
    while (x) {
        str[i++] = (x % 10) + '0';
        x = x / 10;
    }

    // If number of digits required is more, then add 0s at the beginning
    while (i < d)
        str[i++] = '0';

    reverse(str, i);
    str[i] = '\0';
    return i;
}

int myAbs(int num) {
    return (num < 0) ? -num : num;
}

double myPow(double base, int exponent) {
    double result = 1.0;
    for (int i = 0; i < myAbs(exponent); ++i) {
        result *= base;
    }
    if (exponent < 0) {
        result = 1.0 / result;
    }
    return result;
}


void ftoa(float n, char *res, int afterpoint) {
    // Extract integer part
    int ipart = (int)n;

    // Extract floating part
    float fpart = n - (float)ipart;

    // convert integer part to string
    int i = intToStr(ipart, res, 0);

    // check for display option after point
    if (afterpoint != 0) {
        res[i] = '.';  // add dot

        // Get the value of fraction part up to given no.
        // of points after dot. The third parameter is needed
        // to handle cases like 233.007
        fpart = fpart * myPow(10, afterpoint);

        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}
