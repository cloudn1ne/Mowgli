/**
  ******************************************************************************
  * @file    i2c.c
  * @author  Georg Swoboda <cn@warp.at>
  * @brief   I2C Driver for the LIS3DH that is onboard GForce
  ******************************************************************************
  * @attention
  *
  * the setup sequence for tilt sensing and triggering INT1 matches the 
  * original firmware  
  ******************************************************************************
  */

#include <stdio.h>
#include <string.h>

#include "board.h"
#include "main.h"
#include "imu/imu.h"
#include "stm32f1xx_hal.h"
#include "i2c_lis3dh.h"

I2C_HandleTypeDef I2C_Handle;

/**
  * @brief I2C Initialization Function
  * @param None
  * @retval None
  */ 
void I2C_Init(void)
{
   GPIO_InitTypeDef GPIO_InitStruct = {0};
   GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
   GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

   /* Peripheral clock enable */
   __HAL_RCC_I2C1_CLK_ENABLE();

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  I2C_Handle.Instance = I2C1;
  I2C_Handle.Init.ClockSpeed = 1000000;
  I2C_Handle.Init.DutyCycle = I2C_DUTYCYCLE_2;
  I2C_Handle.Init.OwnAddress1 = 0;
  I2C_Handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  I2C_Handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  I2C_Handle.Init.OwnAddress2 = 0;
  I2C_Handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  I2C_Handle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&I2C_Handle) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/*
 * I2C send function
 */
int32_t I2C_platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
  reg |= 0x80;
  HAL_I2C_Mem_Write(handle, LIS3DH_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
  return 0;
}

/*
 * I2C receive function
 */
int32_t I2C_platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
  /* Read multiple command */
  reg |= 0x80;
  HAL_I2C_Mem_Read(handle, LIS3DH_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}

/*
 * read onboard acclerometer values
 */
void I2C_ReadAccelerometer(float *x, float *y, float *z)
{
    uint8_t max_tries = 3;

    stmdev_ctx_t dev_ctx;
    dev_ctx.write_reg = I2C_platform_write;
    dev_ctx.read_reg = I2C_platform_read;
    dev_ctx.handle = &I2C_Handle;

    int16_t data_raw_acceleration[3];            
    lis3dh_reg_t reg;        

    lis3dh_xl_data_ready_get(&dev_ctx, &reg.byte);        
    while (!reg.byte && max_tries) {            
      lis3dh_xl_data_ready_get(&dev_ctx, &reg.byte);        
      HAL_Delay(1);
      max_tries--;
    }    
    if (reg.byte) {            
            memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
            lis3dh_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
            *x = lis3dh_from_fs2_hr_to_mg(data_raw_acceleration[0]) / 1000.0 * MS2_PER_G;
            *y = lis3dh_from_fs2_hr_to_mg(data_raw_acceleration[1]) / 1000.0 * MS2_PER_G;
            *z = lis3dh_from_fs2_hr_to_mg(data_raw_acceleration[2]) / 1000.0 * MS2_PER_G;
           // debug_printf("Acceleration [ms^2]: X=%4.2f\tY=%4.2f\tZ=%4.2f\r\n", *x, *y, *z);  
    }    
    else
    {
        debug_printf("WARNING: timeout while waiting for I2C onboard acceleration sensor");
        *z = 0;
        *y = 0;
        *z = 0;
    }
}

/*
 * read onboard acclerometer temperature value
 */
float I2C_ReadAccelerometerTemp(void)
{        
    uint8_t max_tries = 10;

    stmdev_ctx_t dev_ctx;    
    dev_ctx.write_reg = I2C_platform_write;
    dev_ctx.read_reg = I2C_platform_read;
    dev_ctx.handle = &I2C_Handle;

    static float temperature_degC = 0.0;
    static int16_t data_raw_temperature;
    lis3dh_reg_t reg;  

    lis3dh_temp_data_ready_get(&dev_ctx, &reg.byte);
    while (!reg.byte && max_tries) {            
      lis3dh_temp_data_ready_get(&dev_ctx, &reg.byte);
      HAL_Delay(1);
      max_tries--;
    }
    // Read temperature data 
    if (reg.byte)
    {
        memset(&data_raw_temperature, 0x00, sizeof(int16_t));
        lis3dh_temperature_raw_get(&dev_ctx, &data_raw_temperature);
        temperature_degC =lis3dh_from_lsb_hr_to_celsius(data_raw_temperature);            
        // debug_printf("Temperature [degC]:%6.2f\r\n", temperature_degC);
        return(temperature_degC);    
    }
    else
    {
        debug_printf("WARNING: timeout while waiting for I2C onboard temp sensor");
        return(0);
    }
}

/*
 * test if we can talk to the LIS3DH accelerometer onboard the GForce board
 */
uint8_t I2C_Acclerometer_TestDevice(void)
{
    stmdev_ctx_t dev_ctx;
    lis3dh_reg_t reg;

    dev_ctx.write_reg = I2C_platform_write;
    dev_ctx.read_reg = I2C_platform_read;
    dev_ctx.handle = &I2C_Handle;
    HAL_Delay(50);   // wait for bootup
    /* Check device ID */
    lis3dh_device_id_get(&dev_ctx, &reg.byte);    
    if (reg.byte != LIS3DH_ID) {
        return(0);        
    }    
    return(1);
}

/*
 * INT1 will latch if triggered
 * this function will return its state and unlatch the INT
 */
uint8_t I2C_TestZLowINT(void)
{
    stmdev_ctx_t dev_ctx;

    dev_ctx.write_reg = I2C_platform_write;
    dev_ctx.read_reg = I2C_platform_read;
    dev_ctx.handle = &I2C_Handle;

    lis3dh_int1_src_t int1_src;
    lis3dh_int1_gen_source_get(&dev_ctx, &int1_src); 
    
    //if (int1_src.ia == 1)       
    //{
    //    debug_printf("int1_src.ia: %d int1_src.zl: %d\r\n", int1_src.ia, int1_src.zl);        
    //}
    return(int1_src.zl && int1_src.ia);
}

/*
 * Setup Accelerometer and configure the "tilt protection"
 * "tilt protection" works via hardware INT1 that will stop the blade motor if triggered (see Kicad) 
 */
void I2C_Accelerometer_Setup(void)
{
    stmdev_ctx_t dev_ctx;

    dev_ctx.write_reg = I2C_platform_write;
    dev_ctx.read_reg = I2C_platform_read;
    dev_ctx.handle = &I2C_Handle;

    /* Reboot - reset all settings */
    lis3dh_boot_set(&dev_ctx, 1);
    HAL_Delay(50);

    /* Enable Block Data Update. */
    lis3dh_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
    
    /* Set Output Data Rate to 1Hz. */
    lis3dh_data_rate_set(&dev_ctx, LIS3DH_ODR_100Hz);
    
    /* Set full scale to 2g. */
    lis3dh_full_scale_set(&dev_ctx, LIS3DH_2g);
    
    /* Enable temperature sensor. */
    lis3dh_aux_adc_set(&dev_ctx, LIS3DH_AUX_ON_TEMPERATURE);
    
    /* Set device in continuous mode with 12 bit resol. */
    lis3dh_operating_mode_set(&dev_ctx, LIS3DH_HR_12bit);
                
    /* Set INT1 threshold */
    /* triggers below 0.928g (16mg x 0x3A) - stock firmware uses 0x2C (0.71g) */
    lis3dh_int1_gen_threshold_set(&dev_ctx, IMU_ONBOARD_INCLINATION_THRESHOLD);
    
    /* Set INT1 minimum duration (0xFF = 2.55 sec) */
    lis3dh_int1_gen_duration_set(&dev_ctx, 0x1);   // 10ms

    /* PULSE INT1 */
    /* we have to read INT1_SRC (bit 6) to check the status of the INT */
    lis3dh_int1_pin_notification_mode_set(&dev_ctx, LIS3DH_INT1_PULSED);

    /*  Enable interrupt generation on Z low event or on Direction recognition. */
    lis3dh_int1_cfg_t int1_cfg;
    memset(&int1_cfg, 0, 1); // clear all flags
    int1_cfg.zlie = 1; // enable Z low interrupt
    lis3dh_int1_gen_conf_set(&dev_ctx, &int1_cfg);

    /* INT Polarity (active-high) */
    lis3dh_ctrl_reg6_t ctrl_reg6;
    memset(&ctrl_reg6, 0, 1); // clear all flags means active high for INT_POLARITY
    lis3dh_pin_int2_config_set(&dev_ctx, &ctrl_reg6); 

    /* Enable INT1 */    
    lis3dh_ctrl_reg3_t ctrl_reg3;
    memset(&ctrl_reg3, 0, 1); // clear all flags
    ctrl_reg3.i1_ia1 = 1;   // enable INT1
    lis3dh_pin_int1_config_set(&dev_ctx, &ctrl_reg3); 

    /* FIFO control */    
    lis3dh_fifo_mode_set(&dev_ctx, 0);
    lis3dh_fifo_trigger_event_set(&dev_ctx, 0);
    lis3dh_fifo_watermark_set(&dev_ctx, 0x0);
}
