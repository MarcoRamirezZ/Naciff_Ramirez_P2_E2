/*
 * Practica 2: Simulacion del protocolo I2S
 * Marco Antonio Ramirez Zepeda
 * Jorge Karim Naciff Maldonatt
 * Sistemas Embebidos Basados en Microcontroladores 2
 * Edgardo Serna
 * 16 de mayo del 2020
 * Instituto Tecnologico y de Estudios Superiores de Occidennte
 *
 * Notas:
 * Puerto D bit 12 es SCK
 * Puerto D bit 13 es WS
 * Puerto E bit 25 es SD
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

uint8_t Word_select = 0;
uint8_t clock = 0;
uint8_t first_time = 1;//espera para poder sincronizar con WS

void SCK( void );
void WS( void );
void SD( void );

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

    xTaskCreate(SCK, "SCK", 200, NULL, 1, NULL );
    xTaskCreate(WS,  "WS",  200, NULL, 1, NULL );
    xTaskCreate(SD,  "SD",  200, NULL, 1, NULL );

    vTaskStartScheduler();

    while(1);
    return 0 ;
}

void SCK( void )
{
	uint8_t counter = 0;//indicar cambio para word select

	while(1)
	{
		clock = ( clock ) ? 0 : 1;//invierte de forma periodica el clock
		if( clock )
		{	//clock = 1
			GPIO_set_pin(GPIO_D, bit_12);
		}
		else
		{   //clock = 0
			GPIO_clear_pin(GPIO_D, bit_12);
			if ( counter == 8 )
			{
				//cada 8 flancos de bajada de clk invierte Word_Select
				Word_select = ( Word_select ) ? 0 : 1;
				counter = 0;
			}
			counter ++;
		}
		vTaskDelay(1);
	}
}
void WS( void )
{
	while(1)//Word_Select está determinado en el task de clock, se sincroniza para cada 8 flancos de bajada del clock
	{
		if( Word_select )
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

void SD( void )
{
	uint8_t buffer[8] = {0};//este buffer tiene la información que se va a enviar por el PTE 25
	uint8_t data_to_send[10] = {0x00, 0xFF, 0x00, 0xFF, 0x88, 0x15, 0xD3, 0x32, 0x01, 0x10 };//numeros aleatorios para mandar
	uint8_t i = 0;//Para almacenar en buffer el byte i del arreglo data_to_send
	uint8_t j = 0;//Para mandar el bit j del buffer
	while(1)
	{
		//lenar el buffer para enviar datos, hacer shamt para tener 1 o 0, MSB se manda primero en I2S
		buffer[0] = ( (data_to_send[i] & 0x80) >> 0x7 );//MSB
		buffer[1] = ( (data_to_send[i] & 0x40) >> 0x6 );
		buffer[2] = ( (data_to_send[i] & 0x20) >> 0x5 );
		buffer[3] = ( (data_to_send[i] & 0x10) >> 0x4 );
		buffer[4] = ( (data_to_send[i] & 0x08) >> 0x3 );
		buffer[5] = ( (data_to_send[i] & 0x04) >> 0x2 );
		buffer[6] = ( (data_to_send[i] & 0x02) >> 0x1 );
		buffer[7] = data_to_send[i] & 0x01;             //LSB
		//queremos mandar el buffer antes de volver a llenarlo con la siguiente palabra
		for( j = 0; j < 8; j++ )
		{
			if( !clock )//solo flanco de bajada del clock puede enviar información
			{
				if( ( first_time == 1 ))//sincronizate con WS ignorando el primer ciclo del reloj
				{
					first_time = 0;
					j--;
				}
				else
				{
					if( buffer[j] )//si el dato j que se va a mandar es 1, prender puerto
					{
						GPIO_set_pin(GPIO_E, bit_25);
					}
					else
					{
						GPIO_clear_pin(GPIO_E, bit_25);
					}
				}
			}
			else
			{
				j--;//el ciclo for continua tanto para cuando clock = 1 o 0, queremos que
					//j sea consistete con el dato del buffer que está mandando entonces
				    //en los flancos altos de clock, no incrementar j
			}
		vTaskDelay(1);
		}
		i++;
		j = 0;
		i = ( i == 10 ) ? 0 : i;//si i llega a 10, volver a 0 para enviar el dato 0 de nuevo al buffer
	}
}

