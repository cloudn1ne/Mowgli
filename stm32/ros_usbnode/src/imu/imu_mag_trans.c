
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
void IMU_ApplyMagTransformation(float x, float y, float z, float *x_cal, float *y_cal, float *z_cal)
{

  float uncalibrated_values[3] = { x, y, z };

  uncalibrated_values[0] = x;
  uncalibrated_values[1] = y;
  uncalibrated_values[2] = z;

#ifdef IMU_MAG_INDOOR_CAL
  //calibration_matrix[3][3] is the transformation matrix
  //replace M11, M12,..,M33 with your transformation matrix data
  double calibration_matrix[3][3] = 
  {
    { 0.9883067693,  0.0698495342,  0.0546047070 },
    { 0.0698495342,  1.0050046331, -0.0188832396 },
    { 0.0546047070, -0.0188832396,  1.1295905077 }  
  };

  //bias[3] is the bias
  //replace Bx, By, Bz with your bias data
  double bias[3] = 
  {
     0.0000250267,
    -0.0000084829,
    -0.0000084829
  };  
#else
  // OUTDOOR
  double calibration_matrix[3][3] = 
  {
    { 0.9774527186,  0.0256655433,  0.0039368572 },
    { 0.0256655433,  1.0372358924,  0.0015237000 },
    { 0.0039368572,  0.0015237000,  1.0210926778 }  
  };

  //bias[3] is the bias
  //replace Bx, By, Bz with your bias data
  double bias[3] = 
  {
     0.0000234468,
    -0.0000086897,
    -0.0000069287
  };  
#endif

  float result[3] = {0, 0, 0};

  // apply transformation matrix (correct for sphere deformations)
  for (int i=0; i<3; ++i)
  {
    for (int j=0; j<3; ++j)
    {     
       result[i] += calibration_matrix[i][j] * uncalibrated_values[j];
    }
  }

  // apply bias (shift sphere center)
  for (int i=0; i<3; ++i) 
  {
    result[i] = result[i] + bias[i];
  }
  


  // apply transformation matrix
  /*
  for (int i=0; i<3; ++i)
  {
    for (int j=0; j<3; ++j)
    {     
       result[i] += calibration_matrix[i][j] * uncalibrated_values[j];
    }
  }  
    */

/*
   *x_cal = uncalibrated_values[0];      
  *y_cal = uncalibrated_values[1];
  *z_cal = uncalibrated_values[2];
  */

  *x_cal = result[0];      
  *y_cal = result[1];
  *z_cal = result[2];

}