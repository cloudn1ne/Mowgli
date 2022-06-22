
/**
  ******************************************************************************
  * @file    imu.c
  * @author  Georg Swoboda <cn@warp.at>
  * @brief   Mowgli IMU calibration routines as long as they are IMU independent 
  ******************************************************************************
  * @attention
  *
  * details: https://learn.adafruit.com/adafruit-sensorlab-magnetometer-calibration?view=all
  *          https://makersportal.com/blog/calibration-of-a-magnetometer-with-raspberry-pi
  ******************************************************************************
  */

#include "imu/imu.h"
#include "i2c.h"

/* accelerometer calibration values */
float imu_cal_ax = 0.0;
float imu_cal_ay = 0.0;
float imu_cal_az = 0.0;
/* gyro calibration values */
float imu_cal_gx = 0.0;
float imu_cal_gy = 0.0;
float imu_cal_gz = 0.0;
/* onboard accelerometer calibration values */
float onboard_imu_cal_ax = 0.0;
float onboard_imu_cal_ay = 0.0;
float onboard_imu_cal_az = 0.0;


/**
  * @brief  Reads the 3 magnetometer channels and stores them in *x,*y,*z  
  * 
  * units are tesla uncalibrated
  */ 
void IMU_ReadMagnetometer(float *x, float *y, float *z)
{
    IMU_ReadMagnetometerRaw(x, y, z);    
}

/**
  * @brief  Reads the 3 accelerometer axis and stores them in *x,*y,*z  
  * 
  * units are ms^2 uncalibrated
  */ 
void IMU_ReadAccelerometer(float *x, float *y, float *z)
{
    float imu_x, imu_y, imu_z;
    IMU_ReadAccelerometerRaw(&imu_x, &imu_y, &imu_z);
    // apply calibration
    *x = imu_x - imu_cal_ax;
    *y = imu_y - imu_cal_ay;
    *z = imu_z - imu_cal_az;
}

/**
  * @brief  Reads the 3 accelerometer gyro and stores them in *x,*y,*z  
  * 
  * units are rad/sec uncalibrated
  */ 
void IMU_ReadGyro(float *x, float *y, float *z)
{
    float imu_x, imu_y, imu_z;
    IMU_ReadGyroRaw(&imu_x, &imu_y, &imu_z);
    // apply calibration
    *x = imu_x - imu_cal_gx;
    *y = imu_y - imu_cal_gy;
    *z = imu_z - imu_cal_gz;
}

/*
 * Read onboard IMU acceleration in ms^2
 */
void IMU_Onboard_ReadAccelerometer(float *x, float *y, float *z)
{
   I2C_ReadAccelerometer(x, y, z);
}

/*
 * Read onboard IMU temperature in °C
 */
float IMU_Onboard_ReadTemp(void)
{
  return(I2C_ReadAccelerometerTemp());
}


/**
  * @brief Calibrates IMU accelerometers and gyro by averaging and storing those values as calibration factors 
  * it expects that the bot is leveled and not moving
  * 
  * magnetometer is not calibrated here, but uses the madgwick filter with bias values
  * 
  */ 
void IMU_Calibrate()
{
    float imu_x, imu_y, imu_z;
    float sum_x, sum_y, sum_z;
    uint16_t i;
    uint16_t samples = 100;
    
    debug_printf(" * IMU Calibration started - make sure bot is level and standing still ...\r\n");    
    
    /* calibrate accelerometer */
    sum_x = 0;
    sum_y = 0;
    sum_z = 0;
    for (i=0; i<samples; i++)
    {
      IMU_ReadAccelerometerRaw(&imu_x, &imu_y, &imu_z);
      sum_x += imu_x;
      sum_y += imu_y;
      sum_z += imu_z;
      HAL_Delay(10);      
    }
    imu_cal_ax = sum_x / samples;
    imu_cal_ay = sum_y / samples;
    imu_cal_az = 0;    // we dont want to calibrate Z because our IMU Sensor fusion stack expects gravity
    debug_printf(" * IMU Calibration factors accelerometer [%f %f %f]\r\n", imu_cal_ax, imu_cal_ay, imu_cal_az);

    /* calibrate gyro */
    sum_x = 0;
    sum_y = 0;
    sum_z = 0;
    for (i=0; i<samples; i++)
    {
      IMU_ReadGyroRaw(&imu_x, &imu_y, &imu_z);
      sum_x += imu_x;
      sum_y += imu_y;
      sum_z += imu_z;
      HAL_Delay(10);      
    }
    imu_cal_gx = sum_x / samples;
    imu_cal_gy = sum_y / samples;
    imu_cal_gz = sum_z / samples;    
    debug_printf(" * IMU Calibration factors gyro [%f %f %f]\r\n", imu_cal_gx, imu_cal_gy, imu_cal_gz);

    /* calibrate onboard accelerometer */
    sum_x = 0;
    sum_y = 0;
    sum_z = 0;
    for (i=0; i<samples; i++)
    {
      IMU_ReadAccelerometerRaw(&imu_x, &imu_y, &imu_z);
      sum_x += imu_x;
      sum_y += imu_y;
      sum_z += imu_z;
      HAL_Delay(10);      
    }
    onboard_imu_cal_ax = sum_x / samples;
    onboard_imu_cal_ay = sum_y / samples;
    onboard_imu_cal_az = 0;    // we dont want to calibrate Z because our IMU Sensor fusion stack expects gravity
    debug_printf(" * IMU Calibration factors accelerometer [%f %f %f]\r\n", onboard_imu_cal_ax, onboard_imu_cal_ay, onboard_imu_cal_az);


    // accelerometer
    /*
    imu_cal_ax = 0.407;
    imu_cal_ay = 0.8391;
    imu_cal_az = 0.0;
    */

    // gyro
    //imu_cal_gx = 0.042378;
    //imu_cal_gy = 0.08082;
    //imu_cal_gz = 0.0;
}