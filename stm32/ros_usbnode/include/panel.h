#include "stm32f1xx_hal.h"

#ifndef __PANEL_H
#define __PANEL_H

#ifdef __cplusplus
extern "C"
{
#endif

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

    typedef enum
    {
        PANEL_LED_OFF,
        PANEL_LED_ON,
        PANEL_LED_FLASH_SLOW,
        PANEL_LED_FLASH_FAST
    } PANEL_LED_STATE;

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
