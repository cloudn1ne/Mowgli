#include <stdio.h>
#include <string.h>

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_uart.h"

#include "panel.h"
#include "board.h"

#define LED_STATE_SIZE 12

static uint8_t Led_States[LED_STATE_SIZE];
static uint8_t SendBuffer[256];
static uint8_t ReceiveBuffer[256];
static uint8_t ReceiveIndex;
static uint8_t ReceiveLength;
static uint8_t ReceiveCRC;
static uint8_t Key_Pressed;

void PANEL_Send_Message(uint8_t *data, uint8_t dataLength, uint16_t command)
{
    uint8_t ptr = 0;

    SendBuffer[ptr++] = 0x55;
    SendBuffer[ptr++] = 0xaa;
    SendBuffer[ptr++] = dataLength + 0x02;
    SendBuffer[ptr++] = command >> 8;
    SendBuffer[ptr++] = command & 0xff;

    for (int i = 0; i < dataLength; ++i)
    {
        SendBuffer[ptr++] = data[i];
    }

    uint8_t crc = 0;
    for (int i = 0; i < dataLength + 5; ++i)
    {
        crc += SendBuffer[i];
    }
    SendBuffer[dataLength + 5] = crc;
#ifdef PANEL_USART_ENABLED
    HAL_UART_Transmit(&PANEL_USART_Handler, SendBuffer, dataLength + 6, 250);
#endif
}

void PANEL_Handle_Received_Data(uint8_t rcvd_data)
{
    if (ReceiveIndex == 0 && rcvd_data == 0x55) /* PREAMBLE */
    {
        ReceiveCRC = rcvd_data;
        ReceiveBuffer[ReceiveIndex++] = rcvd_data;
    }
    else if (ReceiveIndex == 1 && rcvd_data == 0xAA) /* PREAMBLE */
    {
        ReceiveCRC += rcvd_data;
        ReceiveBuffer[ReceiveIndex++] = rcvd_data;
    }
    else if (ReceiveIndex == 2) /* LEN */
    {
        ReceiveLength = rcvd_data;
        ReceiveBuffer[ReceiveIndex++] = rcvd_data;
        ReceiveCRC += rcvd_data;
    }
    else if (ReceiveIndex >= 3 && ReceiveIndex <= 2 + ReceiveLength) /* DATA bytes */
    {
        ReceiveBuffer[ReceiveIndex++] = rcvd_data;
        ReceiveCRC += rcvd_data;
    }
    else if (ReceiveIndex >= 3 + ReceiveLength) /* CRC byte */
    {
        if (ReceiveCRC == rcvd_data)
        {
            Key_Pressed = 1;
        }
        ReceiveIndex = 0;
    }
}

void PANEL_Init(void)
{
    uint8_t Key_Init_Msg[] = {0x03, 0x90, 0x28};

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    memset(Led_States, 0x00, LED_STATE_SIZE);
    ReceiveIndex = 0;

#ifdef PANEL_USART_ENABLED
    // enable port and usart clocks
    PANEL_USART_GPIO_CLK_ENABLE();

    // RX
    GPIO_InitStruct.Pin = PANEL_USART_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(PANEL_USART_RX_PORT, &GPIO_InitStruct);

    // TX
    GPIO_InitStruct.Pin = PANEL_USART_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    // GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(PANEL_USART_TX_PORT, &GPIO_InitStruct);

    PANEL_USART_USART_CLK_ENABLE();

    PANEL_USART_Handler.Instance = PANEL_USART_INSTANCE;      // USART1
    PANEL_USART_Handler.Init.BaudRate = 115200;               // Baud rate
    PANEL_USART_Handler.Init.WordLength = UART_WORDLENGTH_8B; // The word is  8  Bit format
    PANEL_USART_Handler.Init.StopBits = USART_STOPBITS_1;     // A stop bit
    PANEL_USART_Handler.Init.Parity = UART_PARITY_NONE;       // No parity bit
    PANEL_USART_Handler.Init.HwFlowCtl = UART_HWCONTROL_NONE; // No hardware flow control
    PANEL_USART_Handler.Init.Mode = USART_MODE_TX_RX;         // Transceiver mode

    HAL_UART_Init(&PANEL_USART_Handler); // HAL_UART_Init() Will enable UART1

    HAL_NVIC_SetPriority(PANEL_USART_IRQ, 0, 0);
	HAL_NVIC_EnableIRQ(PANEL_USART_IRQ);     

    PANEL_Send_Message(NULL, 0, 0xffff);
    HAL_Delay(100);
    PANEL_Send_Message(NULL, 0, 0xfffe);
    HAL_Delay(100);
    PANEL_Send_Message(Key_Init_Msg, sizeof(Key_Init_Msg), 0xfffd);
    HAL_Delay(100);
    PANEL_Send_Message(NULL, 0, 0xfffb);
    HAL_Delay(100);
#endif
}

void PANEL_Set_LED(uint8_t led, PANEL_LED_STATE state)
{
    if (led >= 0 && led < LED_STATE_SIZE)
    {
        switch (state)
        {
        case PANEL_LED_OFF:
            Led_States[led] = 0x00;
            break;

        case PANEL_LED_ON:
            Led_States[led] = 0x10;
            break;

        case PANEL_LED_FLASH_SLOW:
            Led_States[led] = 0x20;
            break;

        case PANEL_LED_FLASH_FAST:
            Led_States[led] = 0x22;
            break;
        }
    }
}

void PANEL_Tick(void)
{
    uint8_t Key_Activate[] = {0x00, 0xFF, 0x00};

#ifdef PANEL_USART_ENABLED
    PANEL_Send_Message(Led_States, sizeof(Led_States), 0x508b);
    PANEL_Send_Message(Key_Activate, sizeof(Key_Activate), 0x5084);
#endif
}

int PANEL_Get_Key_Pressed(void)
{
    uint8_t result = Key_Pressed;
    Key_Pressed = 0;
    return result;
}