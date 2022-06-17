#ifndef __I2C_H
#define __I2C_H


void I2C_Init(void);
void I2C_Test(void);    // debug code
uint8_t I2C_Acclerometer_TestDevice(void);
void I2C_Accelerometer_Setup(void);


int32_t I2C_platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
int32_t I2C_platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);


#endif  /* __I2C_H */