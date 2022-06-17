
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

/**
  * @brief  Reads the 3 magnetometer channels and stores them in *x,*y,*z  
  * then applies the correction factors for x/y
  * 
  * units are tesla uncalibrated
  */ 
void IMU_ReadMagnetometer(float *x, float *y, float *z)
{
    float min_x = -0.0000320228;
    float max_x = 0.0000101871;
    float min_y = 0.0000083309;
    float max_y = 0.0000471792;
    float min_z = -0.0000320228;
    float max_z = 0.0000101871;
    float imu_x,imu_y,imu_z;
    
    IMU_ReadMagnetometerRaw(&imu_x, &imu_y, &imu_z);   

    // apply calibration factors
    *x = imu_x - (min_x + (max_x-min_x)/2);
    *y = imu_y - (min_y + (max_y-min_y)/2);
    *z = imu_z - (min_z + (max_z-min_z)/2);   

}