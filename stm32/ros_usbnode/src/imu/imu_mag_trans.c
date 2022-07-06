
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

  //calibration_matrix[3][3] is the transformation matrix
  //replace M11, M12,..,M33 with your transformation matrix data
  double calibration_matrix[3][3] = 
  {
    { 0.9520019364,  -0.0069509666, 0.0267812379 },
    { -0.0069509666,  1.1035529495, -0.0175829730 },
    { 0.0267812379, -0.0175829730,  0.9890116196 }  
  };

  //bias[3] is the bias
  //replace Bx, By, Bz with your bias data
  double bias[3] = 
  {
     0.0000237957,
    -0.0000169693,
    -0.0000047586
  };  

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
  

  *x_cal = result[0];      
  *y_cal = result[1];
  *z_cal = result[2];

}