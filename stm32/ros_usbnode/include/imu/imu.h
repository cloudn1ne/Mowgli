
#ifndef __IMU_H
#define __IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

/*
 * IMU functions that a compatible IMU needs to be able to provide
 */

uint8_t IMU_TestDevice(void);           // test of existence of IMU and if possible functioning 
void IMU_Init(void);                    // initialize IMU (make settings, ready to fetch readings

void IMU_ReadAccelerometer(int16_t *x, int16_t *y, int16_t *z);
void IMU_ReadGyro(int16_t *x, int16_t *y, int16_t *z);
void IMU_ReadMagnetometer(int16_t *x, int16_t *y, int16_t *z);
float IMU_ReadBarometerTemperatureC(void);
float IMU_ReadBarometerAltitudeMeters(void);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_H */