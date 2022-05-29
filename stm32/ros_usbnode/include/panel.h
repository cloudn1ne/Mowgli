#ifndef __PANEL_H
#define __PANEL_H

#include "board.h"
#include "stm32f1xx_hal.h"


#ifdef __cplusplus
extern "C"
{
#endif

typedef enum
{
        PANEL_LED_OFF,
        PANEL_LED_ON,
        PANEL_LED_FLASH_SLOW,
        PANEL_LED_FLASH_FAST
} PANEL_LED_STATE;


/* 
 * different yardforce models have different keyboard/led panels 
 */

#ifdef PANEL_TYPE_YARDFORCE_500_ECO         // YardForce SA500ECO
    #define PANEL_LED_LIFTED 0
    #define PANEL_LED_SIGNAL 1
    #define PANEL_LED_BATTERY_LOW 2
    #define PANEL_LED_CHARGING 3
    #define PANEL_LED_4H 4
    #define PANEL_LED_6H 5
    #define PANEL_LED_8H 6
    #define PANEL_LED_10H 7
    #define PANEL_LED_S1 8
    #define PANEL_LED_S2 9
    #define PANEL_LED_LOCK 10
    
    #define LED_STATE_SIZE 12       // model has 12-1 different leds to control ?   
    #define LED_CMD 0x508b
#endif

#ifdef PANEL_TYPE_YARDFORCE_500_CLASSIC   // Yardforce 500 CLASSIC
    #define PANEL_LED_LIFTED 0
    #define PANEL_LED_SIGNAL 1
    #define PANEL_LED_BATTERY_LOW 2
    #define PANEL_LED_CHARGING 3
    #define PANEL_LED_2H 4
    #define PANEL_LED_4H 5
    #define PANEL_LED_6H 6
    #define PANEL_LED_8H 7
    #define PANEL_LED_S1 8
    #define PANEL_LED_S2 9
    #define PANEL_LED_LOCK 10
    #define PANEL_LED_MON 11
    #define PANEL_LED_TUE 12
    #define PANEL_LED_WED 13
    #define PANEL_LED_THR 14
    #define PANEL_LED_FRI 15
    #define PANEL_LED_SAT 16
    #define PANEL_LED_SUN 17
    #define PANEL_LED_UNKNOWN 18
    
    #define LED_STATE_SIZE 19       // model has 19-2 different leds to control ?
    #define LED_CMD 0x508e
#endif
   

UART_HandleTypeDef PANEL_USART_Handler;

void PANEL_Init(void);
void PANEL_Tick(void);

void PANEL_Set_LED(uint8_t led, PANEL_LED_STATE state);
int PANEL_Get_Key_Pressed(void);

void PANEL_Handle_Received_Data(uint8_t rcvd_data);

#ifdef __cplusplus
}
#endif

#endif
