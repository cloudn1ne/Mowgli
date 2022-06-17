#ifndef __I2C_H
#define __I2C_H

#include <stdint.h>

void I2C_Init(void);
uint8_t I2C_Acclerometer_TestDevice(void);
void I2C_Accelerometer_Setup(void);
void I2C_ReadAccelerometer(float *x, float *y, float *z);
float I2C_ReadAccelerometerTemp(void);
int32_t I2C_platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
int32_t I2C_platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
uint8_t I2C_TestZLowINT(void);

#endif  /* __I2C_H */