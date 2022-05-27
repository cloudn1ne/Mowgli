/*
 * non_blocking_timer.c
 *
 *  Created on: Jun 10, 2018
 *      Author: Itamar Eliakim
 */

#include "stm32f1xx_hal.h"
#include "nbt.h"

//NBT - Non Blocking Timer
void NBT_init(nbt_t * nbt, uint32_t interval)
{
	nbt->timeout = interval;
	nbt->previousMillis = HAL_GetTick();
}

bool NBT_handler(nbt_t * nbt)
{
	if(HAL_GetTick()-nbt->previousMillis>nbt->timeout){
		nbt->previousMillis = HAL_GetTick();
		return true;
	}

	return false;
}


