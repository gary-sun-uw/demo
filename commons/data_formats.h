#ifndef DATA_FORMATS_H
#define DATA_FORMATS_H

#include <stdint.h>

typedef struct BMI2SensAxisData
{
    /*! Data in x-axis */
    int16_t x;

    /*! Data in y-axis */
    int16_t y;

    /*! Data in z-axis */
    int16_t z;
} BMI2SensAxisData;

typedef struct BMI2SensData
{
    /*! Accelerometer axes data */
    struct BMI2SensAxisData acc;

    /*! Gyroscope axes data */
    struct BMI2SensAxisData gyr;

    /*! Sensor time */
    uint32_t sens_time;
} BMI2SensData;

#endif