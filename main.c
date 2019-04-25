/*
 * main.c
 *
 *  Created on: 04-07-2014
 *      Author: Miccaldo
 */


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "HD44780/HD44780.h"
#include "STEROWANIE/sterowanie.h"


volatile uint16_t adc1, adc2;


int main(void){

	initalize();
	uint8_t targetPosition = 0;

	while(1){


		if(!(PINB & KEY_PIN)){
			targetPosition = 1;
		}
		key_on(targetPosition);
		targetPosition = 0;
		adc1 = pomiar(JOYSTICK_AXIS_2);
		adc2 = pomiar(VOLTAGE_S1);
		_delay_ms(10);
		step_counter(adc1, adc2);

	}
}
