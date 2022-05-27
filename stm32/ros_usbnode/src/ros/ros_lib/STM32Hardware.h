#ifndef _STM32_HARDWARE_H_
#define _STM32_HARDWARE_H_

#include "main.h"
#include "stm32f1xx_hal.h"
#include "usbd_cdc_if.h"
#include "ringbuffer.h"

extern struct ringbuffer rb;

class STM32Hardware
{
public:
	STM32Hardware() {
	}

	void init() {
	}

	// Read a byte of data from ROS connection.
	// If no data , returns -1
	int read()
	{
		uint32_t r;
		uint8_t ch = -1;

		r = ringbuffer_getchar(&rb, &ch);

		if (1 == r)
			return ch;
		else
			return -1;
	}


	// Send a byte of data to ROS connection
	void write(uint8_t* data, int length)
	{
		// make CDC_Transmit_FS blocking
		while (CDC_Transmit_FS(data, length) != USBD_OK) {}
		HAL_Delay(1);
	}

	// Returns milliseconds since start of program
	unsigned long time(void)
	{
		return HAL_GetTick();
	}

};

#endif

