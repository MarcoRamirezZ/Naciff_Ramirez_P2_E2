
/*
 * GPIO.c
 *
 *  Created on: 17 sep 2019
 *      Author: Chencho
 */
#include "MK66F18.h"
#include "GPIO.h"

static gpio_interrupt_flags_t g_intr_status_flag = {0};

uint32_t GPIO_read_interrupt_status_flag_register( gpio_port_name_t port_name)
{
	uint32_t ISFR_status = 0;

	switch(port_name)
	{
	case GPIO_A: /** GPIO A is selected*/
		ISFR_status = PORTA->ISFR;
		return(ISFR_status);
		break;

	case GPIO_B: /** GPIO B is selected*/
		ISFR_status = PORTB->ISFR;
		return(ISFR_status);
		break;

	case GPIO_C: /** GPIO C is selected*/
		ISFR_status = PORTC->ISFR;
		return(ISFR_status);
		break;

	case GPIO_D: /** GPIO D is selected*/
		ISFR_status = PORTD->ISFR;
		return(ISFR_status);
		break;

	default: /** GPIO E is selected*/
		ISFR_status = PORTE->ISFR;
		return(ISFR_status);
		break;
	}

}

uint8_t GPIO_read_interrupt_status_flag_register_pin ( gpio_port_name_t port_name, uint8_t pin)
{
	uint32_t ISFR_status_pin = 0;

	switch(port_name)
	{
	case GPIO_A:/** GPIO A is selected*/
		ISFR_status_pin = PORTA->ISFR;
		ISFR_status_pin = (uint32_t)((0x01)<<pin);
		ISFR_status_pin = ISFR_status_pin>>pin;
		ISFR_status_pin = (uint8_t)(ISFR_status_pin);
		return(ISFR_status_pin);
		break;

	case GPIO_B:/** GPIO B is selected*/
		ISFR_status_pin = PORTB->ISFR;
		ISFR_status_pin = (uint32_t)((0x01)<<pin);
		ISFR_status_pin = ISFR_status_pin>>pin;
		ISFR_status_pin = (uint8_t)(ISFR_status_pin);
		return(ISFR_status_pin);
		break;

	case GPIO_C:/** GPIO C is selected*/
		ISFR_status_pin = PORTC->ISFR;
		ISFR_status_pin &= (uint32_t)((0x01)<<pin);
		ISFR_status_pin = ISFR_status_pin>>pin;
		ISFR_status_pin = (uint8_t)(ISFR_status_pin);
		return(ISFR_status_pin);
		break;

	case GPIO_D:/** GPIO D is selected*/
		ISFR_status_pin = PORTD->ISFR;
		ISFR_status_pin = (uint32_t)((0x01)<<pin);
		ISFR_status_pin = ISFR_status_pin>>pin;
		ISFR_status_pin = (uint8_t)(ISFR_status_pin);
		return(ISFR_status_pin);
		break;

	default: GPIO_E: /** GPIO E is selected*/
		ISFR_status_pin = PORTE->ISFR;
		ISFR_status_pin = (uint32_t)((0x01)<<pin);
		ISFR_status_pin = ISFR_status_pin>>pin;
		ISFR_status_pin = (uint8_t)(ISFR_status_pin);
		return(ISFR_status_pin);
		break;
	}
}
//////////////////////////////////////////////////////////////////////////////////////////

void GPIO_clear_irq_status(gpio_port_name_t gpio)
{
	if(GPIO_A == gpio)
	{
		g_intr_status_flag.flag_port_a = FALSE;
	}
	else
	{
		g_intr_status_flag.flag_port_c = FALSE;
	}
}

uint8_t GPIO_get_irq_status(gpio_port_name_t gpio)
{
	uint8_t status = 0;

	if(GPIO_A == gpio)
	{
		status = g_intr_status_flag.flag_port_a;
	}
	else
	{
		status = g_intr_status_flag.flag_port_c;
	}

	return(status);
}

void GPIO_clear_interrupt(gpio_port_name_t port_name)
{
	switch(port_name)/** Selecting the GPIO for clock enabling*/
	{
		case GPIO_A: /** GPIO A is selected*/
			PORTA->ISFR=0xFFFFFFFF;
			break;
		case GPIO_B: /** GPIO B is selected*/
			PORTB->ISFR=0xFFFFFFFF;
			break;
		case GPIO_C: /** GPIO C is selected*/
			PORTC->ISFR = 0xFFFFFFFF;
			break;
		case GPIO_D: /** GPIO D is selected*/
			PORTD->ISFR=0xFFFFFFFF;
			break;
		default: /** GPIO E is selected*/
			PORTE->ISFR=0xFFFFFFFF;
			break;

	}// end switch
}

/********************************************************************************************/
/********************************************************************************************/


uint8_t GPIO_clock_gating(gpio_port_name_t port_name)
{
	switch(port_name)/** Selecting the GPIO for clock enabling*/
	{
	case GPIO_A: /** GPIO A is selected*/
		SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTA; /** Bit 9 of SIM_SCGC5 is  set*/
		break;
	case GPIO_B: /** GPIO B is selected*/
		SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTB; /** Bit 10 of SIM_SCGC5 is set*/
		break;
	case GPIO_C: /** GPIO C is selected*/
		SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTC; /** Bit 11 of SIM_SCGC5 is set*/
		break;
	case GPIO_D: /** GPIO D is selected*/
		SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTD; /** Bit 12 of SIM_SCGC5 is set*/
		break;
	case GPIO_E: /** GPIO E is selected*/
		SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTE; /** Bit 13 of SIM_SCGC5 is set*/
		break;
	default: /**If doesn't exist the option*/
		return(FALSE);
	}// end switch
	/**Successful configuration*/
	return(TRUE);
}// end function

uint8_t GPIO_pin_control_register(gpio_port_name_t port_name, uint8_t pin,const gpio_pin_control_register_t*  pin_control_register)
{

	switch(port_name)
	{
	case GPIO_A:/** GPIO A is selected*/
		PORTA->PCR[pin] = *pin_control_register;
		break;
	case GPIO_B:/** GPIO B is selected*/
		PORTB->PCR[pin] = *pin_control_register;
		break;
	case GPIO_C:/** GPIO C is selected*/
		PORTC->PCR[pin] = *pin_control_register;
		break;
	case GPIO_D:/** GPIO D is selected*/
		PORTD->PCR[pin] = *pin_control_register;
		break;
	case GPIO_E: /** GPIO E is selected*/
		PORTE->PCR[pin]= *pin_control_register;
		break;
	default:/**If doesn't exist the option*/
		return(FALSE);
		break;
	}
	/**Successful configuration*/
	return(TRUE);
}
/********************************************************************************************/
/********************************************************************************************/
void GPIO_data_direction_pin(gpio_port_name_t port_name, uint8_t state, uint8_t pin)
{
	switch(port_name)
	{
	case GPIO_A:/** GPIO A is selected*/
		if(state == GPIO_OUTPUT) // si es OUTPUT se aplica mascara de salida 1 al bit correspondiente
		{
			GPIOA->PDDR |= (uint32_t)((0x01)<<pin);
		} else
		{// si es INPUT se aplica mascara de entrada 0 al bit correspondiente
			GPIOA->PDDR &= ~(uint32_t)((0x01)<<pin);
		}
		break;
	case GPIO_B:/** GPIO B is selected*/
		if(state == GPIO_OUTPUT) // si es OUTPUT se aplica mascara de salida 1 al bit correspondiente
		{
			GPIOB->PDDR |= (uint32_t)((0x01)<<pin);
		} else
		{// si es INPUT se aplica mascara de entrada 0 al bit correspondiente
			GPIOB->PDDR &= ~(uint32_t)((0x01)<<pin);
		}
		break;
	case GPIO_C:/** GPIO C is selected*/
		if(state == GPIO_OUTPUT) // si es OUTPUT se aplica mascara de salida 1 al bit correspondiente
		{
			GPIOC->PDDR |= (uint32_t)((0x01)<<pin);
		} else
		{// si es INPUT se aplica mascara de entrada 0 al bit correspondiente
			GPIOC->PDDR &= ~(uint32_t)((0x01)<<pin);
		}
		break;
	case GPIO_D:/** GPIO D is selected*/
		if(state == GPIO_OUTPUT) /// si es OUTPUT se aplica mascara de salida 1 al bit correspondiente
		{
			GPIOD->PDDR |= (uint32_t)((0x01)<<pin);
		} else
		{// si es INPUT se aplica mascara de entrada 0 al bit correspondiente
			GPIOD->PDDR &= ~(uint32_t)((0x01)<<pin);
		}
		break;
	case GPIO_E: /** GPIO E is selected*/
		if(state == GPIO_OUTPUT) // si es OUTPUT se aplica mascara de salida 1 al bit correspondiente
		{
			GPIOE->PDDR |= (uint32_t)((0x01)<<pin);
		} else
		{// si es INPUT se aplica mascara de entrada 0 al bit correspondiente
			GPIOE->PDDR &= ~(uint32_t)((0x01)<<pin);
		}
		break;
	}
}

void GPIO_set_pin(gpio_port_name_t port_name, uint8_t pin)
{
	switch(port_name)
	{
	case GPIO_A:/** GPIO A is selected*/
		GPIOA->PSOR = (uint32_t)((0x01)<<pin); // bit correspondiente en PDOR cambia a 1
		break;
	case GPIO_B:/** GPIO B is selected*/
		GPIOB->PSOR = (uint32_t)((0x01)<<pin); // bit correspondiente en PDOR cambia a 1
		break;
	case GPIO_C:/** GPIO C is selected*/
		GPIOC->PSOR = (uint32_t)((0x01)<<pin); // bit correspondiente en PDOR cambia a 1
		break;
	case GPIO_D:/** GPIO D is selected*/
		GPIOD->PSOR = (uint32_t)((0x01)<<pin); // bit correspondiente en PDOR cambia a 1
		break;
	case GPIO_E: /** GPIO E is selected*/
		GPIOE->PSOR = (uint32_t)((0x01)<<pin); // bit correspondiente en PDOR cambia a 1
		break;
	}
}

void GPIO_clear_pin(gpio_port_name_t port_name, uint8_t pin)
{
	switch(port_name)
	{
	case GPIO_A:/** GPIO A is selected*/
		GPIOA->PCOR = (uint32_t)((0x01)<<pin); // bit correspondiente en PDOR cambia a 0
		break;
	case GPIO_B:/** GPIO B is selected*/
		GPIOB->PCOR = (uint32_t)((0x01)<<pin); // bit correspondiente en PDOR cambia a 0
		break;
	case GPIO_C:/** GPIO C is selected*/
		GPIOC->PCOR = (uint32_t)((0x01)<<pin); // bit correspondiente en PDOR cambia a 0
		break;
	case GPIO_D:/** GPIO D is selected*/
		GPIOD->PCOR = (uint32_t)((0x01)<<pin); // bit correspondiente en PDOR cambia a 0
		break;
	case GPIO_E: /** GPIO E is selected*/
		GPIOE->PCOR = (uint32_t)((0x01)<<pin); // bit correspondiente en PDOR cambia a 0
		break;
	}
}

uint32_t GPIO_read_port(gpio_port_name_t port_name)
{
	uint32_t input_port_value;
	switch(port_name)
	{
	case GPIO_A:/** GPIO A is selected*/
		input_port_value = GPIOA->PDIR; // lee valor del puerto de 32 bits y retorna el valor
		return(input_port_value);
		break;
	case GPIO_B:/** GPIO B is selected*/
		input_port_value = GPIOB->PDIR; // lee valor del puerto de 32 bits y retorna el valor
		return(input_port_value);
		break;
	case GPIO_C:/** GPIO C is selected*/
		input_port_value = GPIOC->PDIR; // lee valor del puerto de 32 bits y retorna el valor
		return(input_port_value);
		break;
	case GPIO_D:/** GPIO D is selected*/
		input_port_value = GPIOD->PDIR; // lee valor del puerto de 32 bits y retorna el valor
		return(input_port_value);
		break;
	case GPIO_E: /** GPIO E is selected*/
		input_port_value = GPIOE->PDIR; // lee valor del puerto de 32 bits y retorna el valor
		return(input_port_value);
		break;
	}
	return(FALSE);
}

uint8_t GPIO_read_pin(gpio_port_name_t port_name, uint8_t pin)
{
	uint32_t input_pin_value;
	switch(port_name)
	{
	case GPIO_A:/** GPIO A is selected*/
		input_pin_value = GPIOA->PDIR; // lee valor del puerto de 32 bits
		input_pin_value &= (uint32_t)((0x01)<<pin); // lee solamente el pin deseado
		input_pin_value = (uint8_t)(input_pin_value);
		return(input_pin_value);
		break;
	case GPIO_B:/** GPIO B is selected*/
		input_pin_value = GPIOB->PDIR; // lee valor del puerto de 32 bits
		input_pin_value &= (uint32_t)((0x01)<<pin); // lee solamente el pin deseado
		input_pin_value = (uint8_t)(input_pin_value);
		return(input_pin_value);
		break;
	case GPIO_C:/** GPIO C is selected*/
		input_pin_value = GPIOC->PDIR; // lee valor del puerto de 32 bits
		input_pin_value &= (uint32_t)((0x01)<<pin); // lee solamente el pin deseado
		input_pin_value = (uint8_t)(input_pin_value);
		return(input_pin_value);
		break;
	case GPIO_D:/** GPIO D is selected*/
		input_pin_value = GPIOD->PDIR; // lee valor del puerto de 32 bits
		input_pin_value &= (uint32_t)((0x01)<<pin); // lee solamente el pin deseado
		input_pin_value = (uint8_t)(input_pin_value);
		return(input_pin_value);
		break;
	case GPIO_E: /** GPIO E is selected*/
		input_pin_value = GPIOE->PDIR; // lee valor del puerto de 32 bits
		input_pin_value &= (uint32_t)((0x01)<<pin); // lee solamente el pin deseado
		input_pin_value = (uint8_t)(input_pin_value);
		return(input_pin_value);
		break;
	}
	return(FALSE);
}

void GPIO_toogle_pin(gpio_port_name_t port_name, uint8_t pin)
{
	switch(port_name)
	{
	case GPIO_A:/** GPIO A is selected*/
		GPIOA->PTOR = (uint32_t)((0x01)<<pin); // bit correspondiente en PDOR invierte su lógica
		break;
	case GPIO_B:/** GPIO B is selected*/
		GPIOB->PTOR = (uint32_t)((0x01)<<pin); // bit correspondiente en PDOR invierte su lógica
		break;
	case GPIO_C:/** GPIO C is selected*/
		GPIOC->PTOR = (uint32_t)((0x01)<<pin); // bit correspondiente en PDOR invierte su lógica
		break;
	case GPIO_D:/** GPIO D is selected*/
		GPIOD->PTOR = (uint32_t)((0x01)<<pin); // bit correspondiente en PDOR invierte su lógica
		break;
	case GPIO_E: /** GPIO E is selected*/
		GPIOE->PTOR = (uint32_t)((0x01)<<pin); // bit correspondiente en PDOR invierte su lógica
		break;
	}
}

void GPIO_data_direction_port(gpio_port_name_t port_name, uint32_t direction)
{
	switch(port_name)
	{
	case GPIO_A:/** GPIO A is selected*/
		if(direction == GPIO_OUTPUT) // si es OUTPUT se aplica mascara de salida 1 al puerto
		{
			GPIOA->PDDR |= 0xFFFFFFFF;
		} else
		{// si es INPUT se aplica mascara de entrada 0 al puerto
			GPIOA->PDDR &= ~0xFFFFFFFF;
		}
		break;
	case GPIO_B:/** GPIO B is selected*/
		if(direction == GPIO_OUTPUT) // si es OUTPUT se aplica mascara de salida 1 al puerto
		{
			GPIOB->PDDR |= 0xFFFFFFFF;
		} else
		{// si es INPUT se aplica mascara de entrada 0 al puerto
			GPIOB->PDDR &= ~0xFFFFFFFF;
		}
		break;
	case GPIO_C:/** GPIO C is selected*/
		if(direction == GPIO_OUTPUT) // si es OUTPUT se aplica mascara de salida 1 al puerto
		{
			GPIOC->PDDR |= 0xFFFFFFFF;
		} else
		{// si es INPUT se aplica mascara de entrada 0 al puerto
			GPIOC->PDDR &= ~0xFFFFFFFF;
		}
		break;
	case GPIO_D:/** GPIO D is selected*/
		if(direction == GPIO_OUTPUT) // si es OUTPUT se aplica mascara de salida 1 al puerto
		{
			GPIOD->PDDR |= 0xFFFFFFFF;
		} else
		{// si es INPUT se aplica mascara de entrada 0 al puerto
			GPIOD->PDDR &= ~0xFFFFFFFF;
		}
		break;
	case GPIO_E: /** GPIO E is selected*/
		if(direction == GPIO_OUTPUT) // si es OUTPUT se aplica mascara de salida 1 al puerto
		{
			GPIOE->PDDR |= 0xFFFFFFFF;
		} else
		{// si es INPUT se aplica mascara de entrada 0 al puerto
			GPIOE->PDDR &= ~0xFFFFFFFF;
		}
		break;
	}
}

void GPIO_write_port(gpio_port_name_t port_name, uint32_t data)
{
	switch(port_name)
	{
	case GPIO_A:/** GPIO A is selected*/
		GPIOA->PDOR = data; // se escribe data en el puerto
		break;
	case GPIO_B:/** GPIO B is selected*/
		GPIOB->PDOR =  data; // se escribe data en el puerto
		break;
	case GPIO_C:/** GPIO C is selected*/
		GPIOC->PDOR =  data; // se escribe data en el puerto
		break;
	case GPIO_D:/** GPIO D is selected*/
		GPIOD->PDOR =  data; // se escribe data en el puerto
		break;
	case GPIO_E: /** GPIO E is selected*/
		GPIOE->PDOR =  data; // se escribe data en el puerto
		break;
	}
}
