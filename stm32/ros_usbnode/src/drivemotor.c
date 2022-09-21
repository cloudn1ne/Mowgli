/****************************************************************************
* Title                 :   drive motor module
* Filename              :   drivemotor.c
* Author                :   Nekraus
* Origin Date           :   18/08/2022
* Version               :   1.0.0

*****************************************************************************/
/** \file drivemotor.c
*  \brief drive motor module
*
*/
/******************************************************************************
* Includes
*******************************************************************************/
#include <string.h>

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_uart.h"

#include "main.h"
#include "board.h"

#include "drivemotor.h" 

/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/
#define DRIVEMOTOR_LENGTH_INIT_MSG 38
#define DRIVEMOTOR_LENGTH_RQST_MSG 12
#define DRIVEMOTOR_LENGTH_RECEIVED_MSG 20
/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/

/******************************************************************************
* Module Typedefs
*******************************************************************************/
typedef enum {
    DRIVEMOTOR_INIT_1,
    DRIVEMOTOR_INIT_2,
    DRIVEMOTOR_RUN
}DRIVEMOTOR_STATE_e;

typedef struct 
{
    /* 0*/ uint16_t u16_preambule;
    /* 2*/ uint8_t u8_length;
    /* 3*/ uint16_t u16_id;
    /* 5*/ uint8_t u8_direction;
    /* 6*/ uint8_t u8_left_speed;
    /* 7*/ uint8_t u8_right_speed;
    /* 8*/ uint16_t u16_ukndata0;
    /*10*/ uint8_t u8_left_power;
    /*11*/ uint8_t u8_right_power;
    /*12*/ uint8_t u8_notused1;
    /*13*/ uint16_t u16_left_ticks;
    /*15*/ uint16_t u16_right_ticks;
    /*17*/ uint8_t u8_left_ukn;
    /*18*/ uint8_t u8_right_ukn;
    /*19*/ uint8_t u8_CRC;
} __attribute__((__packed__))DRIVEMOTORS_data_t;

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/
UART_HandleTypeDef DRIVEMOTORS_USART_Handler; // UART  Handle

DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

static DRIVEMOTOR_STATE_e drivemotor_eState = DRIVEMOTOR_INIT_1;
static rx_status_e drivemotors_eRxFlag = RX_WAIT;

static DRIVEMOTORS_data_t drivemotor_psReceivedData = {0};
static uint8_t drivemotor_pu8RqstMessage[DRIVEMOTOR_LENGTH_RQST_MSG]  = { 0x55, 0xaa, 0x08, 0x10, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

const uint8_t drivemotor_pcu8Preamble[5]  = {0x55,0xAA,0x10,0x01,0xE0};
//const uint8_t drivemotor_pcu8InitMsg[DRIVEMOTOR_LENGTH_INIT_MSG] = { 0x55, 0xaa, 0x08, 0x10, 0x80, 0xa0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x37};
const uint8_t drivemotor_pcu8InitMsg[DRIVEMOTOR_LENGTH_INIT_MSG] =  { 0x55, 0xaa, 0x22, 0x10, 0x80, 0x00, 0x00, 0x00, 0x00, 0x02, 0xC8, 0x46, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x05, 0x0F, 0x14, 0x96, 0x0A, 0x1E, 0x5a, 0xfa, 0x05, 0x0A, 0x14, 0x32, 0x40, 0x04, 0x20, 0x01, 0x00, 0x00, 0x2C, 0x01, 0xEE};


int8_t prev_left_direction = 0;
int8_t prev_right_direction = 0;
uint16_t prev_right_encoder_val = 0;
uint16_t prev_left_encoder_val = 0;
int16_t prev_right_wheel_speed_val = 0;
int16_t prev_left_wheel_speed_val = 0;
int32_t right_encoder_ticks = 0;
int32_t left_encoder_ticks = 0;
int8_t left_direction = 0;
int8_t right_direction = 0;
uint16_t right_encoder_val = 0;
uint16_t left_encoder_val = 0;
int16_t right_wheel_speed_val = 0;
int16_t left_wheel_speed_val = 0;
uint8_t right_power = 0;
uint8_t left_power = 0;
    
/******************************************************************************
* Function Prototypes
*******************************************************************************/

/******************************************************************************
*  Public Functions
*******************************************************************************/

/// @brief Initialize STM32 hardware UART to control drive motors
/// @param  
void DRIVEMOTOR_Init(void){
        PAC5210RESET_GPIO_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = PAC5210RESET_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(PAC5210RESET_GPIO_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(PAC5210RESET_GPIO_PORT, PAC5210RESET_PIN, 0);     // take Drive Motor PAC out of reset if LOW


    // PD7 (->PAC5210 PC4), PD8 (->PAC5210 PC3)
     __HAL_RCC_GPIOD_CLK_ENABLE();    
    GPIO_InitStruct.Pin = GPIO_PIN_7| GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7 | GPIO_PIN_8 , 1);

    // enable port and usart clocks
    DRIVEMOTORS_USART_GPIO_CLK_ENABLE();
    DRIVEMOTORS_USART_USART_CLK_ENABLE();
    
    // RX
    GPIO_InitStruct.Pin = DRIVEMOTORS_USART_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_INPUT;
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
    hdma_usart2_rx.Init.Mode = DMA_NORMAL;
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
    hdma_usart2_tx.Init.Priority = DMA_PRIORITY_HIGH;
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

/// @brief handle drive motor messages
/// @param  
void DRIVEMOTOR_App_10ms(void){

    switch (drivemotor_eState)
    {
        case DRIVEMOTOR_INIT_1:

            HAL_UART_Transmit_DMA(&DRIVEMOTORS_USART_Handler, (uint8_t*)drivemotor_pcu8InitMsg, DRIVEMOTOR_LENGTH_INIT_MSG);
            drivemotor_eState = DRIVEMOTOR_RUN;
            debug_printf(" * Drive Motor Controller initialized\r\n");        
            break;
        
        case DRIVEMOTOR_RUN:
            
            /* prepare to receive the message before to launch the command */
            HAL_UART_Receive_DMA(&DRIVEMOTORS_USART_Handler, (uint8_t*)&drivemotor_psReceivedData, sizeof(DRIVEMOTORS_data_t));
            HAL_UART_Transmit_DMA(&DRIVEMOTORS_USART_Handler, (uint8_t*)drivemotor_pu8RqstMessage, DRIVEMOTOR_LENGTH_RQST_MSG);
            break;
        
        default:
            break;
    }

    /* TODO error management */
    switch (drivemotors_eRxFlag)
    {
        case RX_VALID:
            break;
        
        case RX_WAIT:
            
            /* todo check for timeout */

            break;

        case RX_CRC_ERROR:
        case RX_INVALID_ERROR:
        case RX_TIMEOUT_ERROR:
        default:
            /* inform for error */
            break;
    }
}

/// @brief Decode received drive motor messages 
/// @param  
void DRIVEMOTOR_App_Rx(void){
    if(drivemotors_eRxFlag == RX_VALID )
    {
            /* decode */
            uint8_t direction = drivemotor_psReceivedData.u8_direction;
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
            left_encoder_val = drivemotor_psReceivedData.u16_left_ticks;
            right_encoder_val = drivemotor_psReceivedData.u16_right_ticks;
    
            // power consumption
            left_power = drivemotor_psReceivedData.u8_left_power;
            right_power = drivemotor_psReceivedData.u8_right_power;         
            
            /*
              Encoder value can reset to zero twice when changing direction
              2nd reset occurs when the speed changes from zero to non-zero
            */
           
            left_wheel_speed_val = left_direction * drivemotor_psReceivedData.u8_left_speed;
            if( left_direction == 0 || (left_direction != prev_left_direction) || (prev_left_wheel_speed_val == 0 && left_wheel_speed_val != 0) )
            {
                prev_left_encoder_val = 0;
            }
            left_encoder_ticks += left_direction * (left_encoder_val - prev_left_encoder_val);
            prev_left_encoder_val = left_encoder_val;
            prev_left_wheel_speed_val = left_wheel_speed_val;
            prev_left_direction = left_direction;

            right_wheel_speed_val =  right_direction * drivemotor_psReceivedData.u8_right_speed;
            if( right_direction == 0 || (right_direction != prev_right_direction) || (prev_right_wheel_speed_val == 0 && right_wheel_speed_val != 0) )
            {
                prev_right_encoder_val = 0;
            }
            right_encoder_ticks += right_direction * (right_encoder_val - prev_right_encoder_val);
            prev_right_encoder_val = right_encoder_val;
            prev_right_wheel_speed_val = right_wheel_speed_val;
            prev_right_direction = right_direction;
            
            drivemotors_eRxFlag = RX_WAIT;                    // ready for next message      
    }
}


/// @brief Set drive motor speeds
/// @param left_speed left motor speed byte
/// @param right_speed right motor speed byte
/// @param left_dir left motor direction bit
/// @param right_dir  left motor direction bit
void DRIVEMOTOR_SetSpeed(uint8_t left_speed, uint8_t right_speed, uint8_t left_dir, uint8_t right_dir)
{
    uint8_t direction = 0x0;            

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

    drivemotor_pu8RqstMessage[0]= 0x55 ;
    drivemotor_pu8RqstMessage[1]= 0xaa ;
    drivemotor_pu8RqstMessage[2]= 0x08 ;
    drivemotor_pu8RqstMessage[3]= 0x10 ;
    drivemotor_pu8RqstMessage[4]= 0x80 ;
    drivemotor_pu8RqstMessage[5] = direction;
    drivemotor_pu8RqstMessage[6] = left_speed;
    drivemotor_pu8RqstMessage[7] = right_speed;
    drivemotor_pu8RqstMessage[9]= 0x00 ;
    drivemotor_pu8RqstMessage[8] = 0x00;
    drivemotor_pu8RqstMessage[10] = 0x00;
    drivemotor_pu8RqstMessage[11] = crcCalc(drivemotor_pu8RqstMessage, DRIVEMOTOR_LENGTH_RQST_MSG-1);
}


/// @brief drive motor receive interrupt handler
/// @param  
void DRIVEMOTOR_ReceiveIT(void)
{    
    /* decode the frame */
    if(memcmp(drivemotor_pcu8Preamble,(uint8_t*)&drivemotor_psReceivedData,5) == 0){
        uint8_t l_u8crc = crcCalc((uint8_t*)&drivemotor_psReceivedData,DRIVEMOTOR_LENGTH_RECEIVED_MSG-1);
        if(drivemotor_psReceivedData.u8_CRC == l_u8crc )
        {
            drivemotors_eRxFlag = RX_VALID;                  
        }
        else
        {
            drivemotors_eRxFlag = RX_CRC_ERROR; 
        }
    }
    else
    {
         drivemotors_eRxFlag = RX_INVALID_ERROR;
    }  
}

/******************************************************************************
*  Private Functions
*******************************************************************************/