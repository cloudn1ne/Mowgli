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
  I2C_Handle.Init.ClockSpeed = 100000;
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
 * Setup tilt protection
 * if the thresholds are reached INT1 will be triggered
 * 
 */
void I2C_Accelerometer_Setup(void)
{
    stmdev_ctx_t dev_ctx;

    dev_ctx.write_reg = I2C_platform_write;
    dev_ctx.read_reg = I2C_platform_read;
    dev_ctx.handle = &I2C_Handle;

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
    /* Enable INT1 */
    lis3dh_pin_int1_config_set(&dev_ctx, 0x40); 
    /* Set INT1 threshold */
    lis3dh_int1_gen_threshold_set(&dev_ctx, 0x2C);
    /* Set INT1 duration (2.55 sec) */
    lis3dh_int1_gen_duration_set(&dev_ctx, 0xFF);
    /*  Enable interrupt generation on Z low event or on Direction recognition. */
    lis3dh_int1_gen_conf_set(&dev_ctx, 0x10);
    /* FIFO control */
    lis3dh_fifo_watermark_set(&dev_ctx, 0x0);
}


/*
 * test code to interface the LIS accerlometer that is onboard the YF500
 */
void I2C_Test(void)
{
    static int16_t data_raw_acceleration[3];
    static int16_t data_raw_temperature;
    static float acceleration_mg[3];
    static float temperature_degC;
    // static uint8_t whoamI;
    stmdev_ctx_t dev_ctx;
    
    dev_ctx.write_reg = I2C_platform_write;
    dev_ctx.read_reg = I2C_platform_read;
    dev_ctx.handle = &I2C_Handle;
  
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

    /* Read samples in polling mode (no int) */
    while (1) {
        lis3dh_reg_t reg;
        /* Read output only if new value available */
        lis3dh_xl_data_ready_get(&dev_ctx, &reg.byte);        
        if (reg.byte) {
            /* Read accelerometer data */
            memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
            lis3dh_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
            acceleration_mg[0] =
                lis3dh_from_fs2_hr_to_mg(data_raw_acceleration[0]);
            acceleration_mg[1] =
                lis3dh_from_fs2_hr_to_mg(data_raw_acceleration[1]);
            acceleration_mg[2] =
                lis3dh_from_fs2_hr_to_mg(data_raw_acceleration[2]);

            debug_printf("Acceleration [mg]: X=%4.2f\tY=%4.2f\tZ=%4.2f\r\n", acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);        
        }

        lis3dh_temp_data_ready_get(&dev_ctx, &reg.byte);

        if (reg.byte) {            
            // Read temperature data 
            memset(&data_raw_temperature, 0x00, sizeof(int16_t));
            lis3dh_temperature_raw_get(&dev_ctx, &data_raw_temperature);
            temperature_degC =
                lis3dh_from_lsb_hr_to_celsius(data_raw_temperature);
            debug_printf("Temperature [degC]:%6.2f\r\n", temperature_degC);
        }
    }
}
