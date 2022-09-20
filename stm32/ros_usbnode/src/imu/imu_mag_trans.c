/**
  ******************************************************************************
  * @file    imu_mag_trans.c
  * @author  Georg Swoboda <cn@warp.at>
  * @brief   Calibrates the IMU with hard (external_imu_mag_bias) and soft (external_imu_mag_cal_matrix) iron calibration values 
  ******************************************************************************
  * @attention
  *
  * details: https://learn.adafruit.com/adafruit-sensorlab-magnetometer-calibration?view=all
  *          https://makersportal.com/blog/calibration-of-a-magnetometer-with-raspberry-pi
  ******************************************************************************
  */

#include "imu/imu.h"

// HARD IRON COMPENSATION
//onboard_imu_mag_bias[3] is the bias
//replace Bx, By, Bz with your bias data
double external_imu_mag_bias[3];          

// SOFT IRON COMPENSATION
//onboard_imu_mag_cal_matrix[3][3] is the transformation matrix
//replace M11, M12,..,M33 with your transformation matrix data
double external_imu_mag_cal_matrix[3][3];


/**
  ******************************************************************************
  * @file    imu_mag_trans.c
  * @author  Georg Swoboda <cn@warp.at>
  * @brief   Mowgli IMU - apply magnetometer transformation matrix 
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
void IMU_ApplyMagTransformation(double x, double y, double z, double *x_cal, double *y_cal, double *z_cal)
{

  double uncalibrated_values[3] = { x, y, z };
/*
  uncalibrated_values[0] = x;
  uncalibrated_values[1] = y;
  uncalibrated_values[2] = z;
*/
  double result[3] = {0, 0, 0};

  // apply transformation matrix (correct for sphere deformations)
  for (int i=0; i<3; ++i)
  {
    for (int j=0; j<3; ++j)
    {     
       result[i] += external_imu_mag_cal_matrix[i][j] * uncalibrated_values[j];
    }
  }

  // apply bias (shift sphere center)
  for (int i=0; i<3; ++i) 
  {
    result[i] = result[i] + external_imu_mag_bias[i];
  }
  
  *x_cal = result[0];      
  *y_cal = result[1];
  *z_cal = result[2];  
}