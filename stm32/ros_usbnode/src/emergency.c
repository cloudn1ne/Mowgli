
/*
 * Project Mowgli - Emergency Controller 
 * (c) Cybernet / cn@warp.at
 */
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <string.h>
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_uart.h"
#include "stm32f1xx_hal_adc.h"
// stm32 custom
#include "board.h"
#include "main.h"
#include "i2c.h"

// #define EMERGENCY_DEBUG 1

static uint8_t emergency_state = 0;
static uint32_t stop_emergency_started = 0;
static uint32_t wheel_lift_emergency_started = 0;
static uint32_t tilt_emergency_started = 0;
static uint32_t accelerometer_int_emergency_started = 0;
static uint32_t play_button_started = 0;


/**
 * @brief return Emergency State bits
 * @retval >0 if there is an emergency, 0 if all i good
 */
uint8_t Emergency_State(void)
{
    return(emergency_state);
}

/**
 * @brief Poll mechanical Tilt Sensor
 * @retval 1 if tilt is detected, 0 if all is good
 */
int Emergency_Tilt(void)
{
   return(HAL_GPIO_ReadPin(TILT_PORT, TILT_PIN));
}

/**
 * @brief Poll yellow connector stop button
 * @retval 1 if press is detected, 0 if not pressed
 */
int Emergency_StopButtonYellow(void)
{
   return(HAL_GPIO_ReadPin(STOP_BUTTON_YELLOW_PORT, STOP_BUTTON_YELLOW_PIN));
}

/**
 * @brief Poll yellow connector stop button
 * @retval 1 if press is detected, 0 if not pressed
 */
int Emergency_StopButtonWhite(void)
{
   return(HAL_GPIO_ReadPin(STOP_BUTTON_WHITE_PORT, STOP_BUTTON_WHITE_PIN));
}

/**
 * @brief Wheel lift blue sensor
 * @retval 1 if lift is detected, 0 if not lifted
 */
int Emergency_WheelLiftBlue(void)
{
   return(HAL_GPIO_ReadPin(WHEEL_LIFT_BLUE_PORT, WHEEL_LIFT_BLUE_PIN));
}

/**
 * @brief Wheel lift red sensor
 * @retval 1 if lift is detected, 0 if not lifted
 */
int Emergency_WheelLiftRed(void)
{
   return(HAL_GPIO_ReadPin(WHEEL_LIFT_RED_PORT, WHEEL_LIFT_RED_PIN));
}


/**
 * @brief Wheel lift red sensor
 * @retval 1 if lift is detected, 0 if not lifted
 */
int Emergency_LowZAccelerometer(void)
{
   return(I2C_TestZLowINT());
}

/*
 * Manages the emergency sensors
 */
void EmergencyController(void)
{
    uint8_t stop_button_yellow = Emergency_StopButtonYellow();
    uint8_t stop_button_white = Emergency_StopButtonWhite();
    uint8_t wheel_lift_blue = Emergency_WheelLiftBlue();
    uint8_t wheel_lift_red = Emergency_WheelLiftRed();
    uint8_t tilt = Emergency_Tilt();
    GPIO_PinState play_button = !HAL_GPIO_ReadPin(PLAY_BUTTON_PORT, PLAY_BUTTON_PIN); // pullup, active low    
    uint8_t accelerometer_int_triggered = Emergency_LowZAccelerometer();

    uint32_t now = HAL_GetTick();

#ifdef EMERGENCY_DEBUG
    debug_printf("EmergencyController()\r\n");
    debug_printf("  >> stop_button_yellow: %d\r\n", Emergency_StopButtonYellow());
    debug_printf("  >> stop_button_white: %d\r\n", Emergency_StopButtonWhite());
    debug_printf("  >> wheel_lift_blue: %d\r\n", Emergency_WheelLiftBlue());
    debug_printf("  >> wheel_lift_red: %d\r\n", Emergency_WheelLiftRed());
    debug_printf("  >> tilt: %d\r\n", Emergency_Tilt());
    debug_printf("  >> accelerometer_int_triggered: %d\r\n", Emergency_LowZAccelerometer());
    debug_printf("  >> play_button: %d\r\n",play_button);
#endif    

    if (stop_button_yellow || stop_button_white)
    {
        if (stop_emergency_started == 0)
        {
            stop_emergency_started = now;
        }
        else
        {
            if (now - stop_emergency_started >= STOP_BUTTON_EMERGENCY_MILLIS)
            {
                if (stop_button_yellow)
                {
                    emergency_state |= 0b00010;
                    debug_printf(" ## EMERGENCY ## - STOP BUTTON (yellow) triggered\r\n");
                }
                if (stop_button_white) {
                    emergency_state |= 0b00100;
                    debug_printf(" ## EMERGENCY ## - STOP BUTTON (white) triggered\r\n");
                }
            }
        }
    }
    else
    {
        stop_emergency_started = 0;
    }

    if (wheel_lift_blue || wheel_lift_red)
    {
        if (wheel_lift_emergency_started == 0)
        {
            wheel_lift_emergency_started = now;
        }
        else
        {
            if (now - wheel_lift_emergency_started >= WHEEL_LIFT_EMERGENCY_MILLIS)
            {
                if (wheel_lift_blue)
                {
                    emergency_state |= 0b01000;
                    debug_printf(" ## EMERGENCY ## - WHEEL LIFT (blue) triggered\r\n");
                }
                if (wheel_lift_red)
                {
                    emergency_state |= 0b10000;
                    debug_printf(" ## EMERGENCY ## - WHEEL LIFT (red) triggered\r\n");
                }
            }
        }
    }
    if (accelerometer_int_triggered)
    {
        if(accelerometer_int_emergency_started == 0)
        {
            accelerometer_int_emergency_started = now;
        }
        else
        {
            if (now - accelerometer_int_emergency_started >= TILT_EMERGENCY_MILLIS) {
                emergency_state |= 0b100000;
                debug_printf(" ## EMERGENCY ## - ACCELEROMETER TILT triggered\r\n");
            }
        }     
    }
    else
    {
        accelerometer_int_emergency_started = 0;
    }
    
    if (tilt)
    {
        if(tilt_emergency_started == 0)
        {
            tilt_emergency_started = now;
        }
        else
        {
            if (now - tilt_emergency_started >= TILT_EMERGENCY_MILLIS) {
                emergency_state |= 0b100000;
                debug_printf(" ## EMERGENCY ## - MECHANICAL TILT triggered\r\n");
            }
        }
    }
    else
    {
        tilt_emergency_started = 0;
    }

    if (emergency_state && play_button)
    {
        if(play_button_started == 0)
        {
            play_button_started = now;
        }
        else
        {
            if (now - play_button_started >= PLAY_BUTTON_CLEAR_EMERGENCY_MILLIS) {
                emergency_state = 0;
                debug_printf(" ## EMERGENCY ## - manual reset\r\n");
				StatusLEDUpdate();
                chirp(1);
            }
        }
    }
    else
    {
        play_button_started = 0;
    }
}

/**
 * @brief Emergency sensors
 * @retval None
 */
void Emergency_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    STOP_BUTTON_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin = STOP_BUTTON_YELLOW_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(STOP_BUTTON_YELLOW_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = STOP_BUTTON_WHITE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(STOP_BUTTON_WHITE_PORT, &GPIO_InitStruct);

    TILT_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin = TILT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(TILT_PORT, &GPIO_InitStruct);

    WHEEL_LIFT_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin = WHEEL_LIFT_BLUE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(WHEEL_LIFT_BLUE_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = WHEEL_LIFT_RED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(WHEEL_LIFT_RED_PORT, &GPIO_InitStruct);

    PLAY_BUTTON_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin = PLAY_BUTTON_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(PLAY_BUTTON_PORT, &GPIO_InitStruct);
}