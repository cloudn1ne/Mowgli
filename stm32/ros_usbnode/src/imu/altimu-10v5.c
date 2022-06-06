
/**
  ******************************************************************************
  * @file    altimu-10v5.c
  * @author  Georg Swoboda <cn@warp.at>
  * @brief   Mowgli driver for the Pololu AltIMU-10v5 IMU
  ******************************************************************************
  * @attention
  *
  * Some code taken from the various Arduino drivers referenced at 
  * https://www.pololu.com/product/2739
  * 
  * This IMU consists of:
  *     LSM6DS33 (gyro and accelerometer)
  *     LIS3MDL (magnetometer)
  *     LPS25H (barometer)
  ******************************************************************************
  */

#include "imu/imu.h"
#include "imu/altimu-10v5.h"
#include "soft_i2c.h"
#include "main.h"
#include <math.h>


/**
  * @brief  Test Device 
  * Perform any tests possible before actually enabling and using the device,
  * for example check the i2c address and whoami registers if existing  
  *
  * @param  ctx      read / write interface definitions
  * @param  val      get the values of hpcf in reg CTRL_REG2
  * @retval          0 -> test failed 1-> test ok, good to init and use
  *
  */
uint8_t IMU_TestDevice(void)
{
    uint8_t  val;

    /* test the LSM6DS33 (gyro and accelerometer) */
    val = SW_I2C_UTIL_Read(DS33_ADDRESS, DS33_WHO_AM_I);
    if (val == DS33_WHO_ID)
    {
        debug_printf("   >> [AltIMU-10v5] - LSM6DS33 (Gyro / Accelerometer) FOUND at I2C addr=0x%0x\r\n", DS33_ADDRESS);
    }
    else
    {
        debug_printf("   >> [AltIMU-10v5] - Error probing for LSM6DS33 (Gyro / Accelerometer) at I2C addr=0x%0x\r\n", DS33_ADDRESS);
        return(0);
    }
    /* test the LIS3MDL (magnetometer) */
    val = SW_I2C_UTIL_Read(LIS3MDL_ADDRESS, LIS3MDL_WHO_AM_I);
    if (val == LIS3MDL_WHO_ID)
    {
        debug_printf("   >> [AltIMU-10v5] - LIS3MDL (Magnetometer) FOUND at I2C addr=0x%0x\r\n", LIS3MDL_ADDRESS);
    }
    else
    {
        debug_printf("   >> [AltIMU-10v5] - Error probing for LIS3MDL (Magnetometer) at I2C addr=0x%0x\r\n", LIS3MDL_ADDRESS);
        return(0);
    }
    /* test the LPS25H (barometer) */
    val = SW_I2C_UTIL_Read(LPS25H_ADDRESS, LPS25H_WHO_AM_I);
    if (val == LPS25H_WHO_ID)
    {
        debug_printf("   >> [AltIMU-10v5] - LPS25H (Barometer) FOUND at I2C addr=0x%0x\r\n", LPS25H_ADDRESS);
    }
    else
    {
        debug_printf("   >> [AltIMU-10v5] - Error probing for LPS25H (Barometer) at I2C addr=0x%0x\r\n", LPS25H_ADDRESS);
        return(0);
    }
    // all tests passed
    return(1); 
}

/**
  * @brief  Initialize IMU
  * LSM6DS33 +/- 2g acceleration and 245 gps for gyro    
  */
void IMU_Init(void)
{
    /*******************************/
    /* LSM6DS33 Gyro/Accelerometer */
    /*******************************/

    // ACCLEROMETER
    // 0x80 = 0b10000000
    // ODR = 1000 (1.66 kHz (high performance)); FS_XL = 00 (+/-2 g full scale)
    SW_I2C_UTIL_WRITE(DS33_ADDRESS, DS33_CTRL1_XL, 0x80);
    // GYRO
    // 0x80 = 0b010000000
    // ODR = 1000 (1.66 kHz (high performance)); FS_XL = 00 (245 degree per s)
    SW_I2C_UTIL_WRITE(DS33_ADDRESS, DS33_CTRL2_G, 0x80);
    // ACCELEROMETER + GYRO
    // 0x04 = 0b00000100
    // IF_INC = 1 (automatically increment register address)
    SW_I2C_UTIL_WRITE(DS33_ADDRESS, DS33_CTRL3_C, 0x04);

    // MAGNETOMETER
    // 0x70 = 0b01110000
    // OM = 11 (ultra-high-performance mode for X and Y); DO = 100 (10 Hz ODR)
    SW_I2C_UTIL_WRITE(LIS3MDL_ADDRESS, LIS3MDL_CTRL_REG1, 0x70);
    // 0x00 = 0b00000000
    // FS = 00 (+/- 4 gauss full scale)
    SW_I2C_UTIL_WRITE(LIS3MDL_ADDRESS, LIS3MDL_CTRL_REG2, 0x00);
    // 0x00 = 0b00000000
    // MD = 00 (continuous-conversion mode)
    SW_I2C_UTIL_WRITE(LIS3MDL_ADDRESS, LIS3MDL_CTRL_REG3, 0x00);
    // 0x0C = 0b00001100
    // OMZ = 11 (ultra-high-performance mode for Z)
    SW_I2C_UTIL_WRITE(LIS3MDL_ADDRESS, LIS3MDL_CTRL_REG4, 0x0C);

    // BAROMETER
    // 0xB0 = 0b10110000
    // PD = 1 (active mode);  ODR = 011 (12.5 Hz pressure & temperature output data rate)
    SW_I2C_UTIL_WRITE(LPS25H_ADDRESS, LPS25H_CTRL_REG1, 0xB0);      
}

/**
  * @brief  Reads the 3 accelerometer channels and stores them in *x,*y,*z
  * units are m/s^2
  */
void IMU_ReadAccelerometer(float *x, float *y, float *z)
{
    uint8_t accel_xyz[6];   // 2 bytes each

    SW_I2C_UTIL_Read_Multi(DS33_ADDRESS, DS33_OUTX_L_XL, 6, (uint8_t*)&accel_xyz);

/*
    uint8_t i;
    debug_printf("IMU_ReadAccelerometer Raw Bytes: ");
    for (i=0;i<6;i++)
    {
      debug_printf("%02x ", accel_xyz[i]);
    }
    debug_printf("\r\n");
*/
    *x =  (int16_t)(accel_xyz[1] << 8 | accel_xyz[0]) * DS33_G_FACTOR * MS2_PER_G;
    *y =  (int16_t)(accel_xyz[3] << 8 | accel_xyz[2]) * DS33_G_FACTOR * MS2_PER_G;
    *z =  (int16_t)(accel_xyz[5] << 8 | accel_xyz[4]) * DS33_G_FACTOR * MS2_PER_G;    
}

/**
  * @brief  Reads the 3 gyro channels and stores them in *x,*y,*z
  * units are rad/sec
  */
void IMU_ReadGyro(float *x, float *y, float *z)
{
    uint8_t gyro_xyz[6];   // 2 bytes each

    SW_I2C_UTIL_Read_Multi(DS33_ADDRESS, DS33_OUTX_L_G, 6, (uint8_t*)&gyro_xyz);
    
    *x = (int16_t)(gyro_xyz[1] << 8 | gyro_xyz[0]) * DS33_DPS_FACTOR * RAD_PER_G;
    *y = (int16_t)(gyro_xyz[3] << 8 | gyro_xyz[2]) * DS33_DPS_FACTOR * RAD_PER_G;
    *z = (int16_t)(gyro_xyz[5] << 8 | gyro_xyz[4]) * DS33_DPS_FACTOR * RAD_PER_G;    
}

/**
  * @brief  Reads the 3 magnetometer channels and stores them in *x,*y,*z  
  * units are tesla uncalibrated
  */
void IMU_ReadMagnetometerRaw(float *x, float *y, float *z)
{
    uint8_t mag_xyz[6];   // 2 bytes each

    SW_I2C_UTIL_Read_Multi(LIS3MDL_ADDRESS, LIS3MDL_OUT_X_L, 6, (uint8_t*)&mag_xyz);

    *x = (int16_t)(mag_xyz[1] << 8 | mag_xyz[0]) * LIS3MDL_GAUSS_FACTOR * T_PER_GAUSS;
    *y = (int16_t)(mag_xyz[3] << 8 | mag_xyz[2]) * LIS3MDL_GAUSS_FACTOR * T_PER_GAUSS;
    *z = (int16_t)(mag_xyz[5] << 8 | mag_xyz[4]) * LIS3MDL_GAUSS_FACTOR * T_PER_GAUSS;    
}

/**
  * @brief  Reads the raw barometer temp value
  * (internal function only)
  * @retval 16 bit raw temp value
  */
int16_t IMU_BarometerTempRaw(void)
{
    uint8_t temp[2];   
    int16_t retval; // temp
    
    // assert MSB to enable register address auto-increment
    SW_I2C_UTIL_Read_Multi(LPS25H_ADDRESS, LPS25H_TEMP_OUT_L | (1 << 7), 2, (uint8_t*)&temp);

    retval = (int16_t)(temp[1] << 8 | temp[0]);
    return(retval);    
}

/**
  * @brief  Reads the raw barometer pressure value
  * (internal function only)
  * @retval 24 bit raw temp value
  */
int32_t IMU_BarometerPressureRaw(void)
{
    uint8_t pressure[4]={0,0,0,0};   
    int32_t retval; // pressure (24bit)

    // assert MSB to enable register address auto-increment
    SW_I2C_UTIL_Read_Multi(LPS25H_ADDRESS, LPS25H_PRESS_OUT_XL | (1 << 7), 3, (uint8_t*)&pressure);

    retval = (int32_t)(pressure[2] << 16 | pressure[1] << 8 | pressure[0]);
    return(retval);    
}

/**
  * @brief  Calculate millibar of pressure
  * (internal function only)
  * @retval pressure in millibar
  */
float IMU_ReadBarometerPressureMilliBars(void)
{
    return (float)IMU_BarometerPressureRaw() / 4096;
}

/**
  * @brief  Calculate temperature in C
  * @retval temp in celsius
  */
float IMU_ReadBarometerTemperatureC(void)
{
    return 42.5 + (float)IMU_BarometerTempRaw() / 480;
}


/**
  * @brief  Calculate altitude in meters
  * assumes sealevel pressure of 1013.25mbar
  * @retval altitute in meters
  */
float IMU_ReadBarometerAltitudeMeters(void)
{
    return (1 - pow((float)IMU_ReadBarometerPressureMilliBars() / 1013.25, 0.190263)) * 44330.8;
}