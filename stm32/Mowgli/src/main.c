/*
 * Project Mowgli - STM32 Proxy Code 
 * (c) Cybernet / cn@warp.at
 * 
 *  Version 1.0 
 *  
 *  compile with -DBOARD_YARDFORCE500 to enable the YF500 GForce pinout
 *  
 */

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_uart.h"
#include "main.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "board.h"
#include "lis3dh_reg.h"

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);


enum master_rx_status_enum { RX_WAIT, RX_VALID, RX_CRC_ERROR};

uint8_t  master_rx_buf[32];
uint32_t master_rx_buf_idx = 0;
uint8_t  master_rx_buf_crc = 0;
uint8_t  master_rx_LENGTH = 0;
uint8_t  master_rx_CRC = 0;
uint8_t  master_rx_STATUS = RX_WAIT;

int    blade_motor = 0;
uint8_t rcvd_data;


/*
 * Master UART receive ISR
 */ 
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{    
       if (huart->Instance == MASTER_USART_INSTANCE)
       {
           if (master_rx_buf_idx == 0 && rcvd_data == 0x55)           /* PREAMBLE */  
           {                                  
               master_rx_buf_crc = rcvd_data;        
               master_rx_buf[master_rx_buf_idx++] = rcvd_data;                                   
           }           
           else if (master_rx_buf_idx == 1 && rcvd_data == 0xAA)        /* PREAMBLE */    
           {                   
               master_rx_buf_crc += rcvd_data;
               master_rx_buf[master_rx_buf_idx++] = rcvd_data;               
           }
           else if (master_rx_buf_idx == 2) /* LEN */
           {    
               master_rx_LENGTH = rcvd_data;
               master_rx_buf[master_rx_buf_idx++] = rcvd_data;               
               master_rx_buf_crc += rcvd_data;                              
           }
           else if (master_rx_buf_idx >= 3 && master_rx_buf_idx <= 2+master_rx_LENGTH) /* DATA bytes */
           {
               master_rx_buf[master_rx_buf_idx] = rcvd_data;
               master_rx_buf_idx++;
               master_rx_buf_crc += rcvd_data;               
           }
           else if (master_rx_buf_idx >= 3+master_rx_LENGTH)    /* CRC byte */
           {
               master_rx_CRC = rcvd_data;
               master_rx_buf[master_rx_buf_idx] = rcvd_data;
               master_rx_buf_idx++;               
               if (master_rx_buf_crc == rcvd_data)
               {                   
                   // message valid, reader must set back STATUS to RX_WAIT
                   master_rx_STATUS = RX_VALID;
                   //master_rx_buf_idx = 0;
               }
               else
               {                   
                   // crc failed, reader must set back STATUS to RX_WAIT
                   master_rx_STATUS = RX_CRC_ERROR;                   
                   master_rx_buf_idx = 0;
               }
           }
           else
           {
               master_rx_STATUS = RX_WAIT;
               master_rx_buf_idx = 0;               
           }
           
           HAL_UART_Receive_IT(&MASTER_USART_Handler, &rcvd_data, 1);   // rearm interrupt
       }
}


static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
  reg |= 0x80;
  HAL_I2C_Mem_Write(handle, LIS3DH_I2C_ADD_L, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
  return 0;
}


static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  /* Read multiple command */
  reg |= 0x80;
  HAL_I2C_Mem_Read(handle, LIS3DH_I2C_ADD_L, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
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
    static uint8_t whoamI;

    stmdev_ctx_t dev_ctx;
    lis3dh_reg_t reg;
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.handle = &I2C_Handle;
    HAL_Delay(50);   // wait for bootup
    /* Check device ID */
    lis3dh_device_id_get(&dev_ctx, &reg.byte);    
    if (reg.byte != LIS3DH_ID) {
        while (1) {
            /* manage here device not found */
            debug_printf("Accelerometer not found on I2C addr 0x%x\r\n", LIS3DH_I2C_ADD_L);
        }
    }
    debug_printf("Accelerometer found\r\n");
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

int main(void)
{    
    uint8_t blademotor_init[] = { 0x55, 0xaa, 0x12, 0x20, 0x80, 0x00, 0xac, 0x0d, 0x00, 0x02, 0x32, 0x50, 0x1e, 0x04, 0x00, 0x15, 0x21, 0x05, 0x0a, 0x19, 0x3c, 0xaa };    
    uint8_t drivemotors_init[] = { 0x55, 0xaa, 0x08, 0x10, 0x80, 0xa0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x37};
    
    HAL_Init();
    SystemClock_Config();

    __HAL_RCC_AFIO_CLK_ENABLE();
    
    LED_Init();
    TF4_Init();
    PAC5223RESET_Init();
    PAC5210RESET_Init();
    MASTER_USART_Init();
    I2C_Init();

    #ifdef DRIVEMOTORS_USART_ENABLED
        DRIVEMOTORS_USART_Init();
    #endif
    #ifdef BLADEMOTOR_USART_ENABLED
        BLADEMOTOR_USART_Init();
    #endif

    HAL_UART_Receive_IT(&MASTER_USART_Handler, &rcvd_data, 1);
    debug_printf("\r\n============== Init Done ==============\r\n");    
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, 1);
    HAL_GPIO_WritePin(TF4_GPIO_PORT, TF4_PIN, 1);
    HAL_GPIO_WritePin(PAC5223RESET_GPIO_PORT, PAC5223RESET_PIN, 1);     // take Blade PAC out of reset if HIGH
    HAL_GPIO_WritePin(PAC5210RESET_GPIO_PORT, PAC5210RESET_PIN, 0);     // take Drive Motor PAC out of reset if LOW

    // send some init messages - needs further investigation, drivemotors are happy without it
    HAL_UART_Transmit(&DRIVEMOTORS_USART_Handler, drivemotors_init, 12, HAL_MAX_DELAY);
    HAL_Delay(100);
    HAL_UART_Transmit(&DRIVEMOTORS_USART_Handler, drivemotors_init, 12, HAL_MAX_DELAY);
    HAL_Delay(100);

    HAL_UART_Transmit(&BLADEMOTOR_USART_Handler, blademotor_init, 22, HAL_MAX_DELAY);
    HAL_Delay(100);
    HAL_UART_Transmit(&BLADEMOTOR_USART_Handler, blademotor_init, 22, HAL_MAX_DELAY);
    HAL_Delay(100);

    // I2C
    debug_printf("I2C Init starting\r\n");    
    //I2C_Test();
    debug_printf("I2C found\r\n");

    while (1)
    {                
        HAL_Delay(10);
            
        if (master_rx_STATUS == RX_VALID)   // valid frame received by MASTER USART
        {
            
                int i;
                //debug_printf("master_rx_buf_crc = 0x%02x\r\n", master_rx_buf_crc);            
                //debug_printf("master_rx_CRC = 0x%02x\r\n", master_rx_CRC);            
                //debug_printf("master_rx_LENGTH = %d\r\n", master_rx_LENGTH);            
                debug_printf("tx: ");
                for (i=0;i<master_rx_buf_idx;i++)
                {
                    debug_printf(" %02x", master_rx_buf[i]);
                }            
                debug_printf("\r\n");
            

            // until we have some kind of protocol we discrimate what goes where simply by message length
            // drive motors always get a 12 byte message relayed
            if (master_rx_buf_idx == 12)
                HAL_UART_Transmit(&DRIVEMOTORS_USART_Handler, master_rx_buf, master_rx_buf_idx, HAL_MAX_DELAY);
            // blade motor always gets a 12 byte message relayed
            if (master_rx_buf_idx == 7)
                HAL_UART_Transmit(&BLADEMOTOR_USART_Handler, master_rx_buf, master_rx_buf_idx, HAL_MAX_DELAY);

            master_rx_STATUS = RX_WAIT; // ready for next message
            master_rx_buf_idx = 0;

            HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);         // flash LED 
            
        }
        HAL_UART_Receive_IT(&MASTER_USART_Handler, &rcvd_data, 1);   // rearm interrupt                 
    }
}

/**
 * @brief Init the Master Serial Port  - this what connects to the upstream controller
 * @retval None
 */
void MASTER_USART_Init()
{
    // enable port and usart clocks
    MASTER_USART_GPIO_CLK_ENABLE();
    MASTER_USART_USART_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct;
    // RX
    GPIO_InitStruct.Pin = MASTER_USART_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(MASTER_USART_RX_PORT, &GPIO_InitStruct);

    // TX
    GPIO_InitStruct.Pin = MASTER_USART_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    //GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(MASTER_USART_TX_PORT, &GPIO_InitStruct);

    MASTER_USART_Handler.Instance = MASTER_USART_INSTANCE;     // USART1 (DEV)
    MASTER_USART_Handler.Init.BaudRate = 115200;               // Baud rate
    MASTER_USART_Handler.Init.WordLength = UART_WORDLENGTH_8B; // The word is  8  Bit format
    MASTER_USART_Handler.Init.StopBits = USART_STOPBITS_1;     // A stop bit
    MASTER_USART_Handler.Init.Parity = UART_PARITY_NONE;       // No parity bit
    MASTER_USART_Handler.Init.HwFlowCtl = UART_HWCONTROL_NONE; // No hardware flow control
    MASTER_USART_Handler.Init.Mode = USART_MODE_TX_RX;         // Transceiver mode

    HAL_UART_Init(&MASTER_USART_Handler); // HAL_UART_Init() Will enable  UART1

    // enable IRQ
    HAL_NVIC_SetPriority(MASTER_USART_IRQ, 0, 1);
	HAL_NVIC_EnableIRQ(MASTER_USART_IRQ);     
}


/**
 * @brief Init the Drive Motor Serial Port (PAC5210)
 * @retval None
 */
void DRIVEMOTORS_USART_Init()
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // enable port and usart clocks
    DRIVEMOTORS_USART_GPIO_CLK_ENABLE();
    DRIVEMOTORS_USART_USART_CLK_ENABLE();
    
    // RX
    GPIO_InitStruct.Pin = DRIVEMOTORS_USART_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(DRIVEMOTORS_USART_RX_PORT, &GPIO_InitStruct);

    // TX
    GPIO_InitStruct.Pin = DRIVEMOTORS_USART_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    // GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(DRIVEMOTORS_USART_TX_PORT, &GPIO_InitStruct);

    // Alternate Pin Set ?
    __HAL_AFIO_REMAP_USART2_ENABLE();

    DRIVEMOTORS_USART_Handler.Instance = DRIVEMOTORS_USART_INSTANCE;// USART2
    DRIVEMOTORS_USART_Handler.Init.BaudRate = 115200;               // Baud rate
    DRIVEMOTORS_USART_Handler.Init.WordLength = UART_WORDLENGTH_8B; // The word is  8  Bit format
    DRIVEMOTORS_USART_Handler.Init.StopBits = USART_STOPBITS_1;     // A stop bit
    DRIVEMOTORS_USART_Handler.Init.Parity = UART_PARITY_NONE;       // No parity bit
    DRIVEMOTORS_USART_Handler.Init.HwFlowCtl = UART_HWCONTROL_NONE; // No hardware flow control
    DRIVEMOTORS_USART_Handler.Init.Mode = USART_MODE_TX_RX;         // Transceiver mode
    

    HAL_UART_Init(&DRIVEMOTORS_USART_Handler); // HAL_UART_Init() Will enable  UART1
}

/**
 * @brief Init the Blade Motor Serial Port (PAC5223)
 * @retval None
 */
void BLADEMOTOR_USART_Init()
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // enable port and usart clocks
    BLADEMOTOR_USART_GPIO_CLK_ENABLE();
    BLADEMOTOR_USART_USART_CLK_ENABLE();
    
    // RX
    GPIO_InitStruct.Pin = BLADEMOTOR_USART_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(BLADEMOTOR_USART_RX_PORT, &GPIO_InitStruct);

    // TX
    GPIO_InitStruct.Pin = BLADEMOTOR_USART_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    // GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(BLADEMOTOR_USART_TX_PORT, &GPIO_InitStruct);

    // Alternate Pin Set ?
    __HAL_AFIO_REMAP_USART2_ENABLE();

    BLADEMOTOR_USART_Handler.Instance = BLADEMOTOR_USART_INSTANCE;// USART2
    BLADEMOTOR_USART_Handler.Init.BaudRate = 115200;               // Baud rate
    BLADEMOTOR_USART_Handler.Init.WordLength = UART_WORDLENGTH_8B; // The word is  8  Bit format
    BLADEMOTOR_USART_Handler.Init.StopBits = USART_STOPBITS_1;     // A stop bit
    BLADEMOTOR_USART_Handler.Init.Parity = UART_PARITY_NONE;       // No parity bit
    BLADEMOTOR_USART_Handler.Init.HwFlowCtl = UART_HWCONTROL_NONE; // No hardware flow control
    BLADEMOTOR_USART_Handler.Init.Mode = USART_MODE_TX_RX;         // Transceiver mode
    

    HAL_UART_Init(&BLADEMOTOR_USART_Handler); // HAL_UART_Init() Will enable  UART1
}

/**
 * @brief Init LED
 * @retval None
 */
void LED_Init()
{
    LED_GPIO_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);
}

/**
 * @brief Init TF4 (24V Power Switch)
 * @retval None
 */
void TF4_Init()
{
    TF4_GPIO_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = TF4_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(TF4_GPIO_PORT, &GPIO_InitStruct);
}

/**
 * @brief PAC 5223 Reset Line (Blade Motor)
 * @retval None
 */
void PAC5223RESET_Init()
{
    PAC5223RESET_GPIO_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = PAC5223RESET_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(PAC5223RESET_GPIO_PORT, &GPIO_InitStruct);
}

/**
 * @brief PAC 5210 Reset Line (Drive Motors)
 * @retval None
 */
void PAC5210RESET_Init()
{
    PAC5210RESET_GPIO_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = PAC5210RESET_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(PAC5210RESET_GPIO_PORT, &GPIO_InitStruct);


    // PD7 (->PAC5210 PC4), PD8 (->PAC5210 PC3)
     __HAL_RCC_GPIOD_CLK_ENABLE();    
    GPIO_InitStruct.Pin = GPIO_PIN_7| GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7 | GPIO_PIN_8 , 1);
}


/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

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
 * Debug print via MASTER USART
 */
void vprint(const char *fmt, va_list argp)
{
    char string[200];
    if(0 < vsprintf(string,fmt,argp)) // build string
    {
        HAL_UART_Transmit(&MASTER_USART_Handler, (uint8_t*)string, strlen(string), 0xffffff); // send message via UART
    }
}

/*
 * Debug print
 */
void debug_printf(const char *fmt, ...) 
{
    va_list argp;
    va_start(argp, fmt);
    vprint(fmt, argp);
    va_end(argp);
}
