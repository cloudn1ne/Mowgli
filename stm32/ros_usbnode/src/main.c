/**
  ******************************************************************************
  * @file    main.c
  * @author  Georg Swoboda <cn@warp.at>
  * @brief   main / bootup and initialization, motor control routines, usb init
  ******************************************************************************
  * Version 1.0 
  * 
  * compile with -DBOARD_YARDFORCE500 to enable the YF500 GForce pinout
  * 
  * ROS integration howto taken from here: https://github.com/Itamare4/ROS_stm32f1_rosserial_USB_VCP (Itamar Eliakim)
  * 
  ******************************************************************************
  */

#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <string.h>
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_uart.h"
#include "stm32f1xx_hal_adc.h"
#include "main.h"
// stm32 custom
#include "board.h"
#include "panel.h"
#include "emergency.h"
#include "soft_i2c.h"
#include "spiflash.h"
#include "i2c.h"
#include "imu/imu.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "nbt.h"

// ros
#include "cpp_main.h"
#include "ringbuffer.h"

static nbt_t main_chargecontroller_nbt;
static nbt_t main_statusled_nbt;
static nbt_t main_emergency_nbt;

enum rx_status_enum { RX_WAIT, RX_VALID, RX_CRC_ERROR};

// DRIVEMOTORS rx buffering
static uint8_t drivemotors_rcvd_data;
uint8_t  drivemotors_rx_buf[32];
volatile uint32_t drivemotors_rx_buf_idx = 0;
uint8_t  drivemotors_rx_buf_crc = 0;
uint8_t  drivemotors_rx_LENGTH = 0;
uint8_t  drivemotors_rx_CRC = 0;
volatile uint8_t  drivemotors_rx_STATUS = RX_WAIT;
// DRIVEMOTORS tx buffering
volatile uint8_t  drivemotors_tx_busy = 0;
static uint8_t drivemotors_tx_buffer_len;
static char drivemotors_tx_buffer[32];

// MASTER rx buffering
static uint8_t master_rcvd_data;
volatile uint8_t  master_rx_buf[32];
volatile uint32_t master_rx_buf_idx = 0;
volatile uint8_t  master_rx_buf_crc = 0;
volatile uint8_t  master_rx_LENGTH = 0;
volatile uint8_t  master_rx_CRC = 0;
volatile uint8_t  master_rx_STATUS = RX_WAIT;
// MASTER tx buffering
volatile uint8_t  master_tx_busy = 0;
static uint8_t master_tx_buffer_len;
static char master_tx_buffer[255];


int blade_motor = 0;

static uint8_t panel_rcvd_data;

// exported via rostopics
float_t battery_voltage;
float_t charge_voltage;
float_t charge_current;
float_t charge_current_offset;
uint16_t chargecontrol_pwm_val = MIN_CHARGE_PWM;
uint8_t  chargecontrol_is_charging = 0;
int32_t right_encoder_ticks = 0;
int32_t left_encoder_ticks = 0;
int8_t left_direction = 0;
int8_t right_direction = 0;
uint16_t right_encoder_val = 0;
uint16_t left_encoder_val = 0;
int16_t right_wheel_speed_val = 0;
int16_t left_wheel_speed_val = 0;
int8_t prev_left_direction = 0;
int8_t prev_right_direction = 0;
uint16_t prev_right_encoder_val = 0;
uint16_t prev_left_encoder_val = 0;
int16_t prev_right_wheel_speed_val = 0;
int16_t prev_left_wheel_speed_val = 0;

UART_HandleTypeDef MASTER_USART_Handler; // UART  Handle
UART_HandleTypeDef DRIVEMOTORS_USART_Handler; // UART  Handle
UART_HandleTypeDef BLADEMOTOR_USART_Handler; // UART  Handle

// SPI3 FLASH
SPI_HandleTypeDef SPI3_Handle;

// Drive Motors DMA
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_uart4_tx;

ADC_HandleTypeDef ADC_Handle;
TIM_HandleTypeDef TIM1_Handle;  // PWM Charge Controller
TIM_HandleTypeDef TIM3_Handle;  // PWM Beeper

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
   // do nothing here
}

/*
 * called when DMA transfer completes
 * update <xxxx>_tx_busy to let XXXX_Transmit function now when the DMA buffer is free 
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{   
    if (huart->Instance == MASTER_USART_INSTANCE)
    {
        if (__HAL_USART_GET_FLAG(&MASTER_USART_Handler, USART_FLAG_TC))        
        {
            master_tx_busy = 0;
        }
    }
    if (huart->Instance == DRIVEMOTORS_USART_INSTANCE)
    {
        if (__HAL_USART_GET_FLAG(&DRIVEMOTORS_USART_Handler, USART_FLAG_TC))        
        {
            drivemotors_tx_busy = 0;
        }
    }
}

void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
{
   // do nothing here
}

/*
 * Master UART receive ISR
 * DriveMotors UART receive ISR
 * PANEL UART receive ISR
 */ 
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{           
       if (huart->Instance == MASTER_USART_INSTANCE)
       {
           /*
            * MASTER Message handling
            */
           if (master_rx_buf_idx == 0 && master_rcvd_data == 0x55)           /* PREAMBLE */  
           {                                  
               master_rx_buf_crc = master_rcvd_data;        
               master_rx_buf[master_rx_buf_idx++] = master_rcvd_data;                                   
           }           
           else if (master_rx_buf_idx == 1 && master_rcvd_data == 0xAA)        /* PREAMBLE */    
           {                   
               master_rx_buf_crc += master_rcvd_data;
               master_rx_buf[master_rx_buf_idx++] = master_rcvd_data;               
           }
           else if (master_rx_buf_idx == 2) /* LEN */
           {    
               master_rx_LENGTH = master_rcvd_data;
               master_rx_buf[master_rx_buf_idx++] = master_rcvd_data;               
               master_rx_buf_crc += master_rcvd_data;                              
           }
           else if (master_rx_buf_idx >= 3 && master_rx_buf_idx <= 2+master_rx_LENGTH) /* DATA bytes */
           {
               master_rx_buf[master_rx_buf_idx] = master_rcvd_data;
               master_rx_buf_idx++;
               master_rx_buf_crc += master_rcvd_data;               
           }
           else if (master_rx_buf_idx >= 3+master_rx_LENGTH)    /* CRC byte */
           {
               master_rx_CRC = master_rcvd_data;
               master_rx_buf[master_rx_buf_idx] = master_rcvd_data;
               master_rx_buf_idx++;               
               if (master_rx_buf_crc == master_rcvd_data)
               {                   
                   // message valid, reader must set back STATUS to RX_WAIT
                   master_rx_STATUS = RX_VALID;
                   //master_rx_buf_idx = 0;
               }
               else
               {                   
                   // crc failed, reader must set back STATUS to RX_WAIT
                   master_rx_STATUS = RX_WAIT;                   
                   master_rx_buf_idx = 0;
               }
           }
           else
           {
               master_rx_STATUS = RX_WAIT;
               master_rx_buf_idx = 0;               
           }           
       //    HAL_UART_Receive_IT(&MASTER_USART_Handler, &master_rcvd_data, 1);   // rearm interrupt           
       }
       else if (huart->Instance == DRIVEMOTORS_USART_INSTANCE)
       {
         /*
            * DRIVE MOTORS Message handling
            */
           if (drivemotors_rx_buf_idx == 0 && drivemotors_rcvd_data == 0x55)           /* PREAMBLE */  
           {                                   
               drivemotors_rx_buf_crc = drivemotors_rcvd_data;        
               drivemotors_rx_buf[drivemotors_rx_buf_idx++] = drivemotors_rcvd_data;                                   
           }           
           else if (drivemotors_rx_buf_idx == 1 && drivemotors_rcvd_data == 0xAA)        /* PREAMBLE */    
           {                                  
               drivemotors_rx_buf_crc += drivemotors_rcvd_data;
               drivemotors_rx_buf[drivemotors_rx_buf_idx++] = drivemotors_rcvd_data;               
           }
           else if (drivemotors_rx_buf_idx == 2) /* LEN */
           {    
               drivemotors_rx_LENGTH = drivemotors_rcvd_data;
               drivemotors_rx_buf[drivemotors_rx_buf_idx++] = drivemotors_rcvd_data;               
               drivemotors_rx_buf_crc += drivemotors_rcvd_data;                              
           }
           else if (drivemotors_rx_buf_idx >= 3 && drivemotors_rx_buf_idx <= 2+drivemotors_rx_LENGTH) /* DATA bytes */
           {
               drivemotors_rx_buf[drivemotors_rx_buf_idx] = drivemotors_rcvd_data;
               drivemotors_rx_buf_idx++;
               drivemotors_rx_buf_crc += drivemotors_rcvd_data;               
           }
           else if (drivemotors_rx_buf_idx >= 3+drivemotors_rx_LENGTH)    /* CRC byte */
           {
               drivemotors_rx_CRC = drivemotors_rcvd_data;
               drivemotors_rx_buf[drivemotors_rx_buf_idx] = drivemotors_rcvd_data;
               drivemotors_rx_buf_idx++;               
               if (drivemotors_rx_buf_crc == drivemotors_rcvd_data)
               {                   
                   // message valid, reader must set back STATUS to RX_WAIT
                   drivemotors_rx_STATUS = RX_VALID;
                   //drivemotors_rx_buf_idx = 0;
               }
               else
               {                   
                   // crc failed, reader must set back STATUS to RX_WAIT
                   drivemotors_rx_STATUS = RX_CRC_ERROR;                   
                   drivemotors_rx_buf_idx = 0;
               }
           }
           else
           {
               drivemotors_rx_STATUS = RX_WAIT;
               drivemotors_rx_buf_idx = 0;               
           }       
       }       
       else if(huart->Instance == PANEL_USART_INSTANCE)
       {
           PANEL_Handle_Received_Data(panel_rcvd_data);
           HAL_UART_Receive_IT(&PANEL_USART_Handler, &panel_rcvd_data, 1);   // rearm interrupt               
       }
}

int main(void)
{    
    uint8_t blademotor_init[] =  { 0x55, 0xaa, 0x12, 0x20, 0x80, 0x00, 0xac, 0x0d, 0x00, 0x02, 0x32, 0x50, 0x1e, 0x04, 0x00, 0x15, 0x21, 0x05, 0x0a, 0x19, 0x3c, 0xaa };    
    uint8_t drivemotors_init[] = { 0x55, 0xaa, 0x08, 0x10, 0x80, 0xa0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x37};

    HAL_Init();
    SystemClock_Config();

    __HAL_RCC_AFIO_CLK_ENABLE();    
    __HAL_RCC_PWR_CLK_ENABLE();

    MX_DMA_Init();
    
    MASTER_USART_Init();

    debug_printf("\r\n");
    debug_printf("    __  ___                    ___\r\n");
    debug_printf("   /  |/  /___ _      ______ _/ (_)\r\n");
    debug_printf("  / /|_/ / __ \\ | /| / / __ `/ / / \r\n");
    debug_printf(" / /  / / /_/ / |/ |/ / /_/ / / /  \r\n");
    debug_printf("/_/  /_/\\____/|__/|__/\\__, /_/_/   \r\n");
    debug_printf("                     /____/        \r\n");
    debug_printf("\r\n\r\n");
    debug_printf(" * Master USART (debug) initialized\r\n");
    LED_Init();
    debug_printf(" * LED initialized\r\n");
    TIM3_Init();
    HAL_TIM_PWM_Start(&TIM3_Handle, TIM_CHANNEL_4);    
    debug_printf(" * Timer3 (Beeper) initialized\r\n");
    TF4_Init();
    debug_printf(" * 24V switched on\r\n");
    RAIN_Sensor_Init();
    debug_printf(" * RAIN Sensor enable\r\n");
    PAC5223RESET_Init();
    debug_printf(" * PAC 5223 out of reset\r\n");
    PAC5210RESET_Init();
    debug_printf(" * PAC 5210 out of reset\r\n");    
    
    if (SPIFLASH_TestDevice())
    {        
        SPIFLASH_Config();
        SPIFLASH_IncBootCounter();
    }
    else
    {
         debug_printf(" * SPIFLASH: unable to locate SPI Flash\r\n");    
    }    
    debug_printf(" * SPIFLASH initialized\r\n");

    I2C_Init();
    debug_printf(" * Hard I2C initialized\r\n");
    if (I2C_Acclerometer_TestDevice())
    {
         I2C_Accelerometer_Setup();          
    }
    else
    {
        chirp(3);        
        debug_printf(" * WARNING: initalization of onboard accelerometer for tilt protection failed !\r\n");
    }    
    debug_printf(" * Accelerometer (onboard/tilt safety) initialized\r\n");    
    SW_I2C_Init();
    debug_printf(" * Soft I2C (J18) initialized\r\n");
    debug_printf(" * Testing supported IMUs:\r\n");
    IMU_TestDevice();
    IMU_Init();
    IMU_Calibrate();
    Emergency_Init();
    debug_printf(" * Emergency sensors initialized\r\n");
    ADC1_Init();    
    debug_printf(" * ADC1 initialized\r\n");    
    TIM1_Init();   
    debug_printf(" * Timer1 (Charge PWM) initialized\r\n");    
    MX_USB_DEVICE_Init();
    debug_printf(" * USB CDC initialized\r\n");
    PANEL_Init();
    debug_printf(" * Panel initialized\r\n");

    // Charge CH1/CH1N PWM Timer
    HAL_TIM_PWM_Start(&TIM1_Handle, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&TIM1_Handle, TIM_CHANNEL_1);
    debug_printf(" * Charge Controler PWM Timers initialized\r\n");
    
    // Init Drive Motors and Blade Motor
    #ifdef DRIVEMOTORS_USART_ENABLED
        DRIVEMOTORS_USART_Init();
        debug_printf(" * Drive Motors USART initialized\r\n");        
    #endif
    #ifdef BLADEMOTOR_USART_ENABLED
        BLADEMOTOR_USART_Init();
        debug_printf(" * Blade Motor USART initialized\r\n");
    #endif
    

   // HAL_UART_Receive_IT(&MASTER_USART_Handler, &master_rcvd_data, 1);
    debug_printf(" * Master Interrupt enabled\r\n");
    
    //HAL_UART_Receive_IT(&DRIVEMOTORS_USART_Handler, &drivemotors_rcvd_data, 1);    
    HAL_UART_Receive_DMA(&DRIVEMOTORS_USART_Handler, &drivemotors_rcvd_data, 1);
    debug_printf(" * Drive Motors UART DMX (TX/TX) enabled\r\n");

    HAL_UART_Receive_IT(&PANEL_USART_Handler, &panel_rcvd_data, 1);   // rearm interrupt               
    debug_printf(" * Panel Interrupt enabled\r\n");

    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, 0);
    HAL_GPIO_WritePin(TF4_GPIO_PORT, TF4_PIN, 1);                       // turn on 28V supply
    HAL_GPIO_WritePin(PAC5223RESET_GPIO_PORT, PAC5223RESET_PIN, 1);     // take Blade PAC out of reset if HIGH
    HAL_GPIO_WritePin(PAC5210RESET_GPIO_PORT, PAC5210RESET_PIN, 0);     // take Drive Motor PAC out of reset if LOW

    // send some init messages - needs further investigation, drivemotors are happy without it
    HAL_UART_Transmit(&DRIVEMOTORS_USART_Handler, drivemotors_init, 12, HAL_MAX_DELAY);
    HAL_Delay(100);
    HAL_UART_Transmit(&DRIVEMOTORS_USART_Handler, drivemotors_init, 12, HAL_MAX_DELAY);
    HAL_Delay(100);
    debug_printf(" * Drive Motors initialized\r\n");

    HAL_UART_Transmit(&BLADEMOTOR_USART_Handler, blademotor_init, 22, HAL_MAX_DELAY);
    HAL_Delay(100);
    HAL_UART_Transmit(&BLADEMOTOR_USART_Handler, blademotor_init, 22, HAL_MAX_DELAY);
    HAL_Delay(100);
    debug_printf(" * Blade Motor initialized\r\n");
    debug_printf(" * HW Init completed\r\n");    
    
    // read zero offset for charge current
    charge_current_offset = ADC_ChargeCurrent(10);
    debug_printf(" * Charge Current Offset: %2.2fA\r\n", charge_current_offset);

    // Initialize Main Timers
	NBT_init(&main_chargecontroller_nbt, 20);
    NBT_init(&main_statusled_nbt, 1000);
	NBT_init(&main_emergency_nbt, 10);
    debug_printf(" * NBT Main timers initialized\r\n");     

 #ifdef I_DONT_NEED_MY_FINGERS
    debug_printf("\r\n");
    debug_printf("=========================================================\r\n");
    debug_printf(" EMERGENCY/SAFETY FEATURES ARE DISABLED IN board.h ! \r\n");
    debug_printf("=========================================================\r\n");
    debug_printf("\r\n");
 #endif
    // Initialize ROS
    init_ROS();
    debug_printf(" * ROS serial node initialized\r\n");     
    debug_printf("\r\n >>> entering main loop ...\r\n\r\n"); 
    // <chirp><chirp> means we are in the main loop 
    chirp(2);    
    while (1)
    {        
        chatter_handler();
        motors_handler();    
        panel_handler();
        spinOnce();                       
        if (drivemotors_rx_STATUS == RX_VALID)                    // valid frame received from DRIVEMOTORS USART
        {
            uint8_t direction = drivemotors_rx_buf[5];

            // we need to adjust for direction (+/-) !
            if ((direction & 0xc0) == 0xc0)
            {            
                left_direction = 1;
            }
            else if ((direction & 0x80) == 0x80)
            {
                left_direction = -1;
            }
            else
            {
                left_direction = 0;
            }
            if ( (direction & 0x30) == 0x30)
            {            
                right_direction = 1;
            }
            else if ( (direction & 0x20) == 0x20)
            {
                right_direction = -1;
            }
            else
            {
                right_direction = 0;
            }
                        
            left_encoder_val = (drivemotors_rx_buf[14]<<8)+drivemotors_rx_buf[13];
            right_encoder_val = (drivemotors_rx_buf[16]<<8)+drivemotors_rx_buf[15];

            /*
              Encoder value can reset to zero twice when changing direction
              2nd reset occurs when the speed changes from zero to non-zero
            */
            left_wheel_speed_val =  left_direction * drivemotors_rx_buf[6];
            if( left_direction == 0 || (left_direction != prev_left_direction) || (prev_left_wheel_speed_val == 0 && left_wheel_speed_val != 0) )
            {
                prev_left_encoder_val = 0;
            }
            left_encoder_ticks += left_direction * (left_encoder_val - prev_left_encoder_val);
            prev_left_encoder_val = left_encoder_val;
            prev_left_wheel_speed_val = left_wheel_speed_val;
            prev_left_direction = left_direction;

            right_wheel_speed_val =  right_direction * drivemotors_rx_buf[7];
            if( right_direction == 0 || (right_direction != prev_right_direction) || (prev_right_wheel_speed_val == 0 && right_wheel_speed_val != 0) )
            {
                prev_right_encoder_val = 0;
            }
            right_encoder_ticks += right_direction * (right_encoder_val - prev_right_encoder_val);
            prev_right_encoder_val = right_encoder_val;
            prev_right_wheel_speed_val = right_wheel_speed_val;
            prev_right_direction = right_direction;


            //if (drivemotors_rx_buf[5]>>4)       // stuff is moving
           // {
           //    msgPrint(drivemotors_rx_buf, drivemotors_rx_buf_idx);             
           // }                    
            drivemotors_rx_buf_idx = 0;
            drivemotors_rx_STATUS = RX_WAIT;                    // ready for next message                        
            HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);         // flash LED                         
        }     
    
        broadcast_handler();   
        if (NBT_handler(&main_chargecontroller_nbt))
	    {            
			ChargeController();
	    }
        if (NBT_handler(&main_statusled_nbt))
	    {            
			StatusLEDUpdate();                                 
            // debug_printf("master_rx_STATUS: %d  drivemotors_rx_buf_idx: %d  cnt_usart2_overrun: %x\r\n", master_rx_STATUS, drivemotors_rx_buf_idx, cnt_usart2_overrun);           
	    }
#ifndef I_DONT_NEED_MY_FINGERS
		if (NBT_handler(&main_emergency_nbt))
		{
			EmergencyController();
		}
#endif        
    }
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // we never get here ...
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
    
    /* UART4 DMA Init */
    /* UART4_TX Init */
    hdma_uart4_tx.Instance = DMA2_Channel5;
    hdma_uart4_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_uart4_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart4_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart4_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart4_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart4_tx.Init.Mode = DMA_NORMAL;
    hdma_uart4_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_uart4_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(&MASTER_USART_Handler,hdmatx,hdma_uart4_tx);

    // enable IRQ
    HAL_NVIC_SetPriority(MASTER_USART_IRQ, 0, 0);
	HAL_NVIC_EnableIRQ(MASTER_USART_IRQ);     

    __HAL_UART_ENABLE_IT(&MASTER_USART_Handler, UART_IT_TC);
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
    
    HAL_UART_Init(&DRIVEMOTORS_USART_Handler); 

    /* USART2 DMA Init */
    /* USART2_RX Init */    
    hdma_usart2_rx.Instance = DMA1_Channel6;
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(&DRIVEMOTORS_USART_Handler,hdmarx,hdma_usart2_rx);

    // USART2_TX Init */
    hdma_usart2_tx.Instance = DMA1_Channel7;
    hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_tx.Init.Mode = DMA_NORMAL;
    hdma_usart2_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK)
    {
      Error_Handler();
    }
    __HAL_LINKDMA(&DRIVEMOTORS_USART_Handler,hdmatx,hdma_usart2_tx);

    // enable IRQ      
    HAL_NVIC_SetPriority(DRIVEMOTORS_USART_IRQ, 0, 0);
    HAL_NVIC_EnableIRQ(DRIVEMOTORS_USART_IRQ);   

    __HAL_UART_ENABLE_IT(&DRIVEMOTORS_USART_Handler, UART_IT_TC);
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
//    __HAL_AFIO_REMAP_USART2_ENABLE();

    BLADEMOTOR_USART_Handler.Instance = BLADEMOTOR_USART_INSTANCE;// USART3
    BLADEMOTOR_USART_Handler.Init.BaudRate = 115200;               // Baud rate
    BLADEMOTOR_USART_Handler.Init.WordLength = UART_WORDLENGTH_8B; // The word is  8  Bit format
    BLADEMOTOR_USART_Handler.Init.StopBits = USART_STOPBITS_1;     // A stop bit
    BLADEMOTOR_USART_Handler.Init.Parity = UART_PARITY_NONE;       // No parity bit
    BLADEMOTOR_USART_Handler.Init.HwFlowCtl = UART_HWCONTROL_NONE; // No hardware flow control
    BLADEMOTOR_USART_Handler.Init.Mode = USART_MODE_TX_RX;         // Transceiver mode
    
    HAL_UART_Init(&BLADEMOTOR_USART_Handler); 
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
 * @brief Poll RAIN Sensor
 * @retval 1 if rain is detected, 0 if no rain
 */
int RAIN_Sense(void)
{
  return(!HAL_GPIO_ReadPin(RAIN_SENSOR_PORT, RAIN_SENSOR_PIN)); // pullup, active low
}

/**
 * @brief Init RAIN Sensor (PE2) Input
 * @retval None
 */
void RAIN_Sensor_Init()
{
    RAIN_SENSOR_GPIO_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = RAIN_SENSOR_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(RAIN_SENSOR_PORT, &GPIO_InitStruct);
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
 * @brief SPI3 Bus (onboard FLASH)
 * @retval None
 */
void SPI3_Init()
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // Disable JTAG only to free PA15, PB3* and PB4. SWD remains active
    MODIFY_REG(AFIO->MAPR, AFIO_MAPR_SWJ_CFG, AFIO_MAPR_SWJ_CFG_JTAGDISABLE);

    __HAL_RCC_SPI3_CLK_ENABLE();
    FLASH_SPI_CLK_ENABLE();
    FLASH_SPICS_CLK_ENABLE();

    GPIO_InitStruct.Pin = FLASH_CLK_PIN | FLASH_MOSI_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(FLASH_SPI_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = FLASH_MISO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(FLASH_SPI_PORT, &GPIO_InitStruct);
  
    GPIO_InitStruct.Pin = FLASH_nCS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(FLASH_SPICS_PORT, &GPIO_InitStruct);
 
    // now init HAL SPI3_Handle
    SPI3_Handle.Instance = SPI3;
    SPI3_Handle.Init.Mode = SPI_MODE_MASTER;
    SPI3_Handle.Init.Direction = SPI_DIRECTION_2LINES;
    SPI3_Handle.Init.DataSize = SPI_DATASIZE_8BIT;
    SPI3_Handle.Init.CLKPolarity = SPI_POLARITY_LOW;
    SPI3_Handle.Init.CLKPhase = SPI_PHASE_1EDGE;
    SPI3_Handle.Init.NSS = SPI_NSS_HARD_OUTPUT;
    SPI3_Handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    SPI3_Handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    SPI3_Handle.Init.TIMode = SPI_TIMODE_DISABLE;
    SPI3_Handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    SPI3_Handle.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&SPI3_Handle) != HAL_OK)
    {
        Error_Handler();
    }    
}

/**
 * @brief Deinit SPI3 Bus (onboard FLASH)
 * @retval None
 */
void SPI3_DeInit()
{        
    __HAL_RCC_SPI3_CLK_DISABLE();
    
    HAL_GPIO_DeInit(FLASH_SPI_PORT,FLASH_CLK_PIN | FLASH_MISO_PIN | FLASH_nCS_PIN);
    HAL_GPIO_DeInit(FLASH_SPICS_PORT,FLASH_nCS_PIN);
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
        debug_printf("Error Handler reached, oops\r\n");
        chirp(1);        
    }
    /* USER CODE END Error_Handler_Debug */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/*
 * VREF+ = 3.3v
 */
void ADC1_Init(void)
{
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /**ADC1 GPIO Configuration
    PA1     ------> Charge Current
    PA2     ------> Charge Voltage
    PA3     ------> Battery Voltage
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USER CODE BEGIN ADC1_Init 0 */

    /* USER CODE END ADC1_Init 0 */

   // ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC1_Init 1 */

    /* USER CODE END ADC1_Init 1 */

    /** Common config
     */
    ADC_Handle.Instance = ADC1;
    ADC_Handle.Init.ScanConvMode = ADC_SCAN_DISABLE;
    ADC_Handle.Init.ContinuousConvMode = DISABLE;
    ADC_Handle.Init.DiscontinuousConvMode = DISABLE;
    ADC_Handle.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    ADC_Handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    ADC_Handle.Init.NbrOfConversion = 1;
    if (HAL_ADC_Init(&ADC_Handle) != HAL_OK)
    {
        Error_Handler();
    }

    // calibrate  - important for accuracy !
    HAL_ADCEx_Calibration_Start(&ADC_Handle); 
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
 void TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  __HAL_RCC_TIM1_CLK_ENABLE();
  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  TIM1_Handle.Instance = TIM1;
  TIM1_Handle.Init.Prescaler = 0;
  TIM1_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  TIM1_Handle.Init.Period = 1400;
  TIM1_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  TIM1_Handle.Init.RepetitionCounter = 0;
  TIM1_Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&TIM1_Handle) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&TIM1_Handle, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&TIM1_Handle, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 120;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&TIM1_Handle, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_1;
  sBreakDeadTimeConfig.DeadTime = 40;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&TIM1_Handle, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

  GPIO_InitTypeDef GPIO_InitStruct = {0};  
  CHARGE_GPIO_CLK_ENABLE();
  /** TIM1 GPIO Configuration
  PA7 or PE8     -----> TIM1_CH1N
  PA8 oe PE9    ------> TIM1_CH1
  */
  GPIO_InitStruct.Pin = CHARGE_LOWSIDE_PIN|CHARGE_HIGHSIDE_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CHARGE_GPIO_PORT, &GPIO_InitStruct);
  #ifdef BOARD_BLUEPILL
    __HAL_AFIO_REMAP_TIM1_PARTIAL();        // to use PA7/8 it is a partial remap
  #endif
  #ifdef BOARD_YARDFORCE500 
     __HAL_AFIO_REMAP_TIM1_ENABLE();        // to use PE8/9 it is a full remap
  #endif  
}


/**
  * @brief TIM3 Initialization Function
  *
  * Beeper is on PB1 (PWM)
  * 
  * @param None
  * @retval None
  */
void TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */
  __HAL_RCC_TIM3_CLK_ENABLE();

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  TIM3_Handle.Instance = TIM3;
  TIM3_Handle.Init.Prescaler = 36000; // 72Mhz -> 2khz
  TIM3_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  TIM3_Handle.Init.Period = 50;
  TIM3_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  TIM3_Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&TIM3_Handle) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&TIM3_Handle, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&TIM3_Handle) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&TIM3_Handle, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&TIM3_Handle, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  /**TIM3 GPIO Configuration
  PB1     ------> TIM3_CH4
  */
  GPIO_InitTypeDef GPIO_InitStruct = {0};  
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


}

/**
  * Enable DMA controller clock
  */
void MX_DMA_Init(void)
{

   /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
  /* DMA2_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel4_5_IRQn);

}


/*
 * Charge Current
 */
float ADC_ChargeCurrent(uint8_t adc_conversions)
{
    float_t adc_current;
    uint8_t i;
    uint32_t adc_val_sum;
    uint16_t adc_val;
    ADC_ChannelConfTypeDef sConfig = {0};

    // switch channel
    sConfig.Channel = ADC_CHANNEL_1; // PA1 Charge Current
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
    if (HAL_ADC_ConfigChannel(&ADC_Handle, &sConfig) != HAL_OK)
    {
       Error_Handler();
    }
    // do adc conversion
    adc_val_sum = 0;    
    for (i=0; i<adc_conversions; i++)
    {
        HAL_ADC_Start(&ADC_Handle);
        HAL_ADC_PollForConversion(&ADC_Handle, 200);
        adc_val_sum += (uint16_t) HAL_ADC_GetValue(&ADC_Handle);
        HAL_ADC_Stop(&ADC_Handle);
    }    
    adc_val = adc_val_sum/adc_conversions;
    // guessed from experiments, close enough to at least determine when the battery is charging
    adc_current= ((float)(adc_val/4095.0f)*3.3f - 2.5f) * 100/12.0;
    return(adc_current);
}


/*
 * Charge Voltage
 */
float ADC_ChargeVoltage(uint8_t adc_conversions)
{
    float_t adc_volt;
    uint8_t i;
    uint32_t adc_val_sum;
    uint16_t adc_val;

    ADC_ChannelConfTypeDef sConfig = {0};

    // switch channel
    sConfig.Channel = ADC_CHANNEL_2; // PA2 Charge Voltage
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
    if (HAL_ADC_ConfigChannel(&ADC_Handle, &sConfig) != HAL_OK)
    {
       Error_Handler();
    }
    // do adc conversion
    adc_val_sum = 0;    
    for (i=0; i<adc_conversions; i++)
    {
        HAL_ADC_Start(&ADC_Handle);
        HAL_ADC_PollForConversion(&ADC_Handle, 200);
        adc_val_sum += (uint16_t) HAL_ADC_GetValue(&ADC_Handle);
        HAL_ADC_Stop(&ADC_Handle);
    }    
    adc_val = adc_val_sum/adc_conversions;
    adc_volt= (float)(adc_val/4095.0f)*3.3f*16;     //PA2 has a 1:16 divider
    return(adc_volt);
}


/*
 * Battery Voltage
 */
float ADC_BatteryVoltage(uint8_t adc_conversions)
{
    float_t adc_volt;
    uint8_t i;
    uint32_t adc_val_sum;
    uint16_t adc_val;

    ADC_ChannelConfTypeDef sConfig = {0};

    // switch channel
    sConfig.Channel = ADC_CHANNEL_3; // PA3 Battery
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
    if (HAL_ADC_ConfigChannel(&ADC_Handle, &sConfig) != HAL_OK)
    {
       Error_Handler();
    }
    // do adc conversion
    adc_val_sum = 0;    
    for (i=0; i<adc_conversions; i++)
    {
        HAL_ADC_Start(&ADC_Handle);
        HAL_ADC_PollForConversion(&ADC_Handle, 200);
        adc_val_sum += (uint16_t) HAL_ADC_GetValue(&ADC_Handle);
        HAL_ADC_Stop(&ADC_Handle);
    }    
    adc_val = adc_val_sum/adc_conversions;
    adc_volt= (float)(adc_val/4095.0f)*3.3f*10 + 0.3;  //PA3 has a 1:10 divider - and there is a 0.3V drop between Bat and ADC
    return(adc_volt);
}

/*
 * ADC test code to sample PA2 (Charge) and PA3 (Battery) Voltage
 */
void ADC_Test()
{
    float_t adc_volt;
    uint16_t adc_val;
    ADC_ChannelConfTypeDef sConfig = {0};

        // switch channel
        sConfig.Channel = ADC_CHANNEL_3; // PA3 Battery
        sConfig.Rank = ADC_REGULAR_RANK_1;
        sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
        if (HAL_ADC_ConfigChannel(&ADC_Handle, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }
        // do adc conversion
        HAL_ADC_Start(&ADC_Handle);
        HAL_ADC_PollForConversion(&ADC_Handle, 200);
        adc_val = (uint16_t) HAL_ADC_GetValue(&ADC_Handle);
        HAL_ADC_Stop(&ADC_Handle);
        adc_volt= (float)(adc_val/4095.0f)*3.3f*10;  //PA3 has a 1:10 divider
        debug_printf("Battery Voltage: %2.2fV (adc:%d)\r\n", adc_volt, adc_val);

        // switch channel
        sConfig.Channel = ADC_CHANNEL_2; // PA2 Charge
        sConfig.Rank = ADC_REGULAR_RANK_1;
        sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
        if (HAL_ADC_ConfigChannel(&ADC_Handle, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }
        // do adc conversion
        HAL_ADC_Start(&ADC_Handle);
        HAL_ADC_PollForConversion(&ADC_Handle, 200);
        adc_val = (uint16_t) HAL_ADC_GetValue(&ADC_Handle);
        HAL_ADC_Stop(&ADC_Handle);
        adc_volt= (float)(adc_val/4095.0f)*3.3f*16;     //PA2 has a 1:16 divider
        debug_printf(" Charge Voltage: %2.2fV (adc:%d)\r\n", adc_volt, adc_val);
        debug_printf("\r\n");
  
}

/*
 * manaes the charge voltage, and charge, lowbat LED
 * needs to be called frequently
 */
void ChargeController(void)
{                        
        charge_voltage =  ADC_ChargeVoltage(5);                    
        battery_voltage = ADC_BatteryVoltage(5);
        charge_current = ADC_ChargeCurrent(5) - charge_current_offset;
        if (charge_current < 0)
            charge_current = 0;
        
        if (charge_voltage >= MIN_CHARGE_VOLTAGE ) {
            //HAL_GPIO_WritePin(TF4_GPIO_PORT, TF4_PIN, 0);                       // turn off 28V supply while charging
            // set PWM to approach 29.4V charge voltage
            if ((charge_voltage < 29.2) && (chargecontrol_pwm_val < 1350))
            {
                chargecontrol_pwm_val++;
            }            
            if ((charge_voltage > 29.2) && (chargecontrol_pwm_val > 50))
            {
                chargecontrol_pwm_val--;
            }
            // cap charge current at 1.0 Amps
            if (charge_current > MAX_CHARGE_CURRENT)
            {
                chargecontrol_pwm_val--;
            }
        }
        else {
             //HAL_GPIO_WritePin(TF4_GPIO_PORT, TF4_PIN, 1);                       // turn on 28V supply if we are not charging
            // chargecontrol_pwm_val = MIN_CHARGE_PWM;    
             // constantly get fresh offet if we are not charging      
             if (charge_voltage == 0)
             {
                charge_current_offset = ADC_ChargeCurrent(5);
             }
        }        
        TIM1->CCR1 = chargecontrol_pwm_val;  
}

/*
 * Update the states for the Emergency, Charge and Low Bat LEDs
 */
void StatusLEDUpdate(void)
{
        if (Emergency_State()) {
            debug_printf("Emergency !");
            PANEL_Set_LED(PANEL_LED_LIFTED, PANEL_LED_FLASH_FAST);
        } else {
            PANEL_Set_LED(PANEL_LED_LIFTED, PANEL_LED_OFF);
        }

        if ((charge_voltage >= MIN_CHARGE_VOLTAGE) && (charge_current >= MIN_CHARGE_CURRENT))         // we are charging ...
        {
            // indicate charging by flashing fast if we are plugged in
            PANEL_Set_LED(PANEL_LED_CHARGING, PANEL_LED_FLASH_FAST);
            chargecontrol_is_charging = 1;
        }
        else
        {
            PANEL_Set_LED(PANEL_LED_CHARGING, PANEL_LED_OFF);
            chargecontrol_is_charging = 0;
        }
            
        // show a lowbat warning if battery voltage drops below LOW_BAT_THRESHOLD ? (random guess, needs more testing or a compare to the stock firmware)            
        // if goes below LOW_BAT_THRESHOLD-1 we increase led flash frequency        
        if (battery_voltage <= LOW_BAT_THRESHOLD)
        {
            PANEL_Set_LED(PANEL_LED_BATTERY_LOW, PANEL_LED_FLASH_SLOW); // low
        }
        else if (battery_voltage <= LOW_BAT_THRESHOLD-1.0)
        {
            PANEL_Set_LED(PANEL_LED_BATTERY_LOW, PANEL_LED_FLASH_FAST); // really low
        }
        else
        {
            PANEL_Set_LED(PANEL_LED_BATTERY_LOW, PANEL_LED_OFF); // bat ok
        }                
        debug_printf(" > Chg Voltage: %2.2fV | Chg Current: %2.2fA (offset: %2.2fA) | PWM: %d | Bat Voltage %2.2fV\r\n", charge_voltage, charge_current, charge_current_offset,  chargecontrol_pwm_val, battery_voltage);                           
}

/*
 * Send message to DriveMotors PAC
 *
 * <xxx>_speed = 0x - 0xFF
 * <xxx>_dir = 1 = CW, 1 != CCW
 */
void setDriveMotors(uint8_t left_speed, uint8_t right_speed, uint8_t left_dir, uint8_t right_dir)
{
    uint8_t direction = 0x0;            
    static uint8_t drivemotors_msg[DRIVEMOTORS_MSG_LEN] =  { 0x55, 0xaa, 0x8, 0x10, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
    
    drivemotors_msg[5] = direction;
    drivemotors_msg[6] = left_speed;
    drivemotors_msg[7] = right_speed;

    // calc direction bits
    if (right_dir == 1)
    {        
        direction |= (0x20 + 0x10);
    }
    else
    {
        direction |= 0x20;
    }
    if (left_dir == 1)
    {   
        direction |= (0x40 + 0x80);                             
    }
    else
    {
        direction |= 0x80;
    }
    // update direction byte in message
    drivemotors_msg[5] = direction;
    // calc crc
    drivemotors_msg[DRIVEMOTORS_MSG_LEN-1] = crcCalc(drivemotors_msg, DRIVEMOTORS_MSG_LEN-1);    
  // msgPrint(drivemotors_msg, DRIVEMOTORS_MSG_LEN);
    // transmit via UART
    DRIVEMOTORS_Transmit(drivemotors_msg, DRIVEMOTORS_MSG_LEN);
    // HAL_UART_Transmit(&DRIVEMOTORS_USART_Handler, drivemotors_msg, DRIVEMOTORS_MSG_LEN, HAL_MAX_DELAY);
     //HAL_UART_Transmit_DMA(&DRIVEMOTORS_USART_Handler, drivemotors_msg, DRIVEMOTORS_MSG_LEN);
}

/*
 * Send message to Blade PAC
 *
 * <on_off> - no speed settings available
 */
void setBladeMotor(uint8_t on_off)
{
    uint8_t blademotor_on[] =  { 0x55, 0xaa, 0x03, 0x20, 0x80, 0x80, 0x22};
    uint8_t blademotor_off[] = { 0x55, 0xaa, 0x03, 0x20, 0x80, 0x0, 0xa2};

    if (on_off)
    {
        HAL_UART_Transmit(&BLADEMOTOR_USART_Handler, blademotor_on, sizeof(blademotor_on), HAL_MAX_DELAY);    
    }
    else
    {
        HAL_UART_Transmit(&BLADEMOTOR_USART_Handler, blademotor_off, sizeof(blademotor_off), HAL_MAX_DELAY);
    }
        
}

/*
 * print hex bytes
 */
void msgPrint(uint8_t *msg, uint8_t msg_len)
{
     int i;
     debug_printf("msg: ");
     for (i=0;i<msg_len;i++)
     {
        debug_printf(" %02x", msg[i]);
     }            
     debug_printf("\r\n");
}

/*
 * calc crc byte
 */
uint8_t crcCalc(uint8_t *msg, uint8_t msg_len)
{
    uint8_t crc = 0x0;
    uint8_t i;

    for (i=0;i<msg_len;i++)
    {
        crc += msg[i];
    }
    return(crc);
}

/*
 * 2khz chirps
 */
void chirp(uint8_t count)
{    
    uint8_t i;

    for (i=0;i<count;i++)
    {
        TIM3_Handle.Instance->CCR4 = 10;   
        HAL_Delay(100);
        TIM3_Handle.Instance->CCR4 = 0;  
        HAL_Delay(50);
    }
}

/*
 * Debug print via MASTER USART
 */
void vprint(const char *fmt, va_list argp)
{
    char string[200];    
    if(0 < vsprintf(string,fmt,argp)) // build string
    {
        MASTER_Transmit((unsigned char*)string, strlen(string));

        // HAL_UART_Transmit(&MASTER_USART_Handler, (uint8_t*)string, strlen(string), HAL_MAX_DELAY); // send message via UART               
        //while (master_tx_busy) {  }
        //master_tx_busy = 1;        
        //master_tx_buffer_len = strlen(string);
        //memcpy(master_tx_buffer, string, master_tx_buffer_len);
        //HAL_UART_Transmit_DMA(&MASTER_USART_Handler, (uint8_t*)master_tx_buffer, master_tx_buffer_len); // send message via UART                
        // HAL_Delay(10);
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

/*
 * Send message via MASTER USART (DMA Normal Mode)
 */
void MASTER_Transmit(uint8_t *buffer, uint8_t len)
{    
    // wait until tx buffers are free (send complete)
    while (master_tx_busy) {  }
    master_tx_busy = 1;  
    // copy into our master_tx_buffer 
    master_tx_buffer_len = len;
    memcpy(master_tx_buffer, buffer, master_tx_buffer_len);
    HAL_UART_Transmit_DMA(&MASTER_USART_Handler, (uint8_t*)master_tx_buffer, master_tx_buffer_len); // send message via UART       
}

/*
 * Send message via MASTER USART (DMA Normal Mode)
 */
void DRIVEMOTORS_Transmit(uint8_t *buffer, uint8_t len)
{    
    // wait until tx buffers are free (send complete)
    while (drivemotors_tx_busy) {  }
    drivemotors_tx_busy = 1;  
    // copy into our master_tx_buffer 
    drivemotors_tx_buffer_len = len;
    memcpy(drivemotors_tx_buffer, buffer, drivemotors_tx_buffer_len);
    HAL_UART_Transmit_DMA(&DRIVEMOTORS_USART_Handler, (uint8_t*)drivemotors_tx_buffer, drivemotors_tx_buffer_len); // send message via UART       
}