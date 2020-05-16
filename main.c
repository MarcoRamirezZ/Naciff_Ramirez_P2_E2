/*
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    Practica 2.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include <stdint.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK66F18.h"
#include "fsl_debug_console.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "GPIO.h"

typedef struct
{
	uint8_t readyWS;
	uint8_t readySD;

} data_t;

void SCK( void*data );
void WS( void*data );
void SD( void*data );

int main(void) {

  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
    BOARD_InitDebugConsole();

	gpio_pin_control_register_t input_intr_config = GPIO_MUX1|GPIO_PE|GPIO_PS|INTR_RISING_EDGE;

	GPIO_clock_gating( GPIO_D );
	GPIO_clock_gating( GPIO_E );

	GPIO_pin_control_register( GPIO_D, bit_12, &input_intr_config );//SCK
	GPIO_pin_control_register( GPIO_D, bit_13, &input_intr_config );//WS
	GPIO_pin_control_register( GPIO_E, bit_25, &input_intr_config );//SD

	GPIO_data_direction_pin( GPIO_D, GPIO_OUTPUT, bit_12 );
	GPIO_data_direction_pin( GPIO_D, GPIO_OUTPUT, bit_13 );
	GPIO_data_direction_pin( GPIO_E, GPIO_OUTPUT, bit_25 );


    static data_t sync;

    xTaskCreate(SCK, "SCK", 200, (void*)&sync, 1, NULL );
    xTaskCreate(WS,  "WS",  200, (void*)&sync, 1, NULL );
    xTaskCreate(SD,  "SD",  200, (void*)&sync, 1, NULL );

    vTaskStartScheduler();

    while(1);
    return 0 ;
}

void SCK( void*data )
{
	data_t sync = *((data_t*)data);
	uint64_t clock = 0;//reloj de salida
	uint8_t counter = 0;//indicar cambio para word select

	while(1)
	{
		clock = ( clock ) ? 0 : 1;//invierte de forma periodica el clock
		if( clock )
		{
			GPIO_set_pin(GPIO_D, bit_12);
		}
		else
		{
			GPIO_clear_pin(GPIO_D, bit_12);
		}
		if ( counter == 15 )
		{
			//cada 8 flancos de clk invierte WS
			sync.readyWS = !sync.readyWS;
			counter = 0;
		}
		counter ++;
		vTaskDelay(1);
	}
}
void WS( void*data )
{
	data_t sync = *((data_t*)data);
	while(1)
	{
		if( sync.readyWS )
		{
			GPIO_set_pin(GPIO_D, bit_13);
		}
		else
		{
			GPIO_clear_pin(GPIO_D, bit_13);
		}
		vTaskDelay(1);
	}
}

void SD( void*data )
{
	data_t sync = *((data_t*)data);

	while(1)
	{

	}
}

