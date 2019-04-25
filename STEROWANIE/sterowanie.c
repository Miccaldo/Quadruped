/*
 * sterowanie.c
 *
 *  Created on: 04-07-2014
 *      Author: Miccaldo
 */


#include <avr/io.h>
#include <avr/interrupt.h>
#include "sterowanie.h"
#include "../HD44780/HD44780.h"


volatile uint8_t legFront, counter, initial_position_cntA, initial_position_cntB;
volatile uint8_t counter = 1;
volatile uint8_t half13 = 1;
volatile uint8_t half24 = 0;
volatile uint8_t firstStep, secondStep, stepBack_A, stepBack_B, back_target_position_A, back_target_position_B;
volatile uint8_t position, targetPosition, back_backward_flag_A, back_backward_flag_B, right_rotate_A, right_rotate_B, left_rotate_A, left_rotate_B;
volatile uint8_t target_leg, gait_mode, whole;
volatile uint8_t step_forward, step_backward, GM_leg;
volatile uint8_t rotateRightFlag, rotateLeftFlag;


void initalize(){

	TCCR1B |= (1<<WGM12) |(1<<CS12);	// CTC, preskaler 1024, przerwania co 20 ms
	TIMSK |= (1<<OCIE1A) | (1<<OCIE1B);
	OCR1A = 624;
	OCR1B = 624;

	// INICJALIZACJA adc1
	JOYSTICK_DIR |= JOYSTICK_PIN;
	JOYSTICK_PORT &= ~JOYSTICK_PIN;

	VOLTAGE_S1_DIR |= VOLTAGE_S1_PIN;
	VOLTAGE_S1_PORT &= ~VOLTAGE_S1_PIN;

	WARNING_LED_OUT;

	ADCSRA |= (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
	ADMUX |= REF256;

	KEY_PORT |= KEY_PIN;

	sei();

	//TCCR2 |= (1<<WGM20)|(1<<COM21)|(1<<CS22)|(1<<CS21)|(1<<CS20);
	//OCR2 = 255;

}

void key_on(uint8_t data){
	if(data == 1) position = 1;
}

uint16_t pomiar(uint8_t kanal){

	ADMUX = (ADMUX & 0xF8) | kanal;

	DDRC &= ~(1<<kanal);
	ADCSRA |= (1<<ADSC);

	while(ADCSRA & (1<<ADSC));

	return ADCW;
}

void main_TimeCounter(volatile uint8_t leg, volatile int16_t time_us1, volatile int16_t time_us2, volatile int16_t time_us3){

	uint16_t time_buf[2];

	typedef struct{
		int16_t timing_offset_value1;		// UP
		int16_t timing_offset_value2;		// SIDE
		int16_t timing_offset_value3;		// TOP
	} t_time;

	t_time time1;
	t_time time2;
	t_time time3;
	t_time time4;

	typedef struct{
		uint8_t up_side_top:1;
		uint8_t up_top_side:1;
		uint8_t side_up_top:1;
		uint8_t side_top_up:1;
		uint8_t top_up_side:1;
		uint8_t top_side_up:1;
	} t_order_servos;

	t_order_servos order_servos1;
	t_order_servos order_servos2;
	t_order_servos order_servos3;
	t_order_servos order_servos4;

	if(leg == 1){

		// Obliczenie czasu trwania ka¿dego impulsu
		time1.timing_offset_value1 = TARGET_POSITION_L1U + time_us1;		// UP
		time1.timing_offset_value2 = TARGET_POSITION_L1U + TARGET_POSITION_L1S + time_us2;		// SIDE
		time1.timing_offset_value3 = TARGET_POSITION_L1U + TARGET_POSITION_L1S + TARGET_POSITION_L1T + time_us3;

		time_buf[0] = time1.timing_offset_value1;
		time_buf[1] = time1.timing_offset_value2;
		time_buf[2] = time1.timing_offset_value3;

		// Wyznaczenie kolejnosci impulsow od najkrotszego do najdluzszego
		order_servos1.up_side_top = (time1.timing_offset_value1 < time1.timing_offset_value2 && time1.timing_offset_value2 < time1.timing_offset_value3) ? 1:0;
		order_servos1.up_top_side = (time1.timing_offset_value1 < time1.timing_offset_value3 && time1.timing_offset_value3 < time1.timing_offset_value2) ? 1:0;
		order_servos1.side_up_top = (time1.timing_offset_value2 < time1.timing_offset_value1 && time1.timing_offset_value1 < time1.timing_offset_value3) ? 1:0;
		order_servos1.side_top_up = (time1.timing_offset_value2 < time1.timing_offset_value3 && time1.timing_offset_value3 < time1.timing_offset_value1) ? 1:0;
		order_servos1.top_up_side = (time1.timing_offset_value3 < time1.timing_offset_value1 && time1.timing_offset_value1 < time1.timing_offset_value2) ? 1:0;
		order_servos1.top_side_up = (time1.timing_offset_value3 < time1.timing_offset_value2 && time1.timing_offset_value2 < time1.timing_offset_value1) ? 1:0;

		// Ustawienie pinow na wyjscie i stan wysoki
		LEG_1_DIR_A |= LEG_1_PIN_A;
		LEG_1_DIR_B |= LEG_1_PIN_B;
		LEG_1_PORT_A |= LEG_1_PIN_A;
		LEG_1_PORT_B |= LEG_1_PIN_B;

		if(order_servos1.up_side_top == 1){		// Jesli kolejnosc UP, SIDE, TOP

			time_buf[1] -= time1.timing_offset_value1;	// Obliczenie roznicy czasu trwania drugiego impulsu i pierwszego impulsu
			time1.timing_offset_value3 -= time1.timing_offset_value2;	// Obliczenie roznicy czasu trwania trzeciego impulsu i drugiego impulsu

			up_1s(time1.timing_offset_value1);	// pierwszy impuls w calosci
			side_1s(time_buf[1]);		// roznica drugiego i pierwszego
			top_1s(time1.timing_offset_value3);	// roznica trzeciego i drugiego
		}
		else if(order_servos1.up_top_side == 1){

			time_buf[2] -= time1.timing_offset_value1;
			time1.timing_offset_value2 -= time1.timing_offset_value3;

			up_1s(time1.timing_offset_value1);
			top_1s(time_buf[2]);
			side_1s(time1.timing_offset_value2);
		}
		else if(order_servos1.side_up_top == 1){

			time_buf[0] -= time1.timing_offset_value2;
			time1.timing_offset_value3 -= time1.timing_offset_value1;

			side_1s(time1.timing_offset_value2);
			up_1s(time_buf[0]);
			top_1s(time1.timing_offset_value3);
		}
		else if(order_servos1.side_top_up == 1){

			time_buf[2] -= time1.timing_offset_value2;
			time1.timing_offset_value1 -= time1.timing_offset_value3;

			side_1s(time1.timing_offset_value2);
			top_1s(time_buf[2]);
			up_1s(time1.timing_offset_value1);
		}
		else if(order_servos1.top_up_side == 1){
			time_buf[0] -= time1.timing_offset_value3;
			time1.timing_offset_value2 -= time1.timing_offset_value1;

			top_1s(time1.timing_offset_value3);
			up_1s(time_buf[0]);
			side_1s(time1.timing_offset_value2);
		}
		else if(order_servos1.top_side_up == 1){

			time_buf[1] -= time1.timing_offset_value3;
			time1.timing_offset_value1 -= time1.timing_offset_value2;

			top_1s(time1.timing_offset_value3);
			side_1s(time_buf[1]);
			up_1s(time1.timing_offset_value1);
		}
	}

	if(leg == 2){

		time2.timing_offset_value1 = TARGET_POSITION_L2U + time_us1;		// UP
		time2.timing_offset_value3 = TARGET_POSITION_L2U + TARGET_POSITION_L2T + time_us2;		// SIDE
		time2.timing_offset_value2 = TARGET_POSITION_L2U + TARGET_POSITION_L2T + TARGET_POSITION_L2S + time_us3;	// TOP

		time_buf[0] = time2.timing_offset_value1;
		time_buf[1] = time2.timing_offset_value2;
		time_buf[2] = time2.timing_offset_value3;

		order_servos2.up_side_top = (time2.timing_offset_value1 < time2.timing_offset_value2 && time2.timing_offset_value2 < time2.timing_offset_value3) ? 1:0;
		order_servos2.up_top_side = (time2.timing_offset_value1 < time2.timing_offset_value3 && time2.timing_offset_value3 < time2.timing_offset_value2) ? 1:0;
		order_servos2.side_up_top = (time2.timing_offset_value2 < time2.timing_offset_value1 && time2.timing_offset_value1 < time2.timing_offset_value3) ? 1:0;
		order_servos2.side_top_up = (time2.timing_offset_value2 < time2.timing_offset_value3 && time2.timing_offset_value3 < time2.timing_offset_value1) ? 1:0;
		order_servos2.top_up_side = (time2.timing_offset_value3 < time2.timing_offset_value1 && time2.timing_offset_value1 < time2.timing_offset_value2) ? 1:0;
		order_servos2.top_side_up = (time2.timing_offset_value3 < time2.timing_offset_value2 && time2.timing_offset_value2 < time2.timing_offset_value1) ? 1:0;


		LEG_2_DIR |= LEG_2_PIN;
		LEG_2_PORT |= (SERVO_SIDE_2 | SERVO_UP_2 | SERVO_TOP_2);

		if(order_servos2.up_side_top == 1){

			time_buf[1] -= time2.timing_offset_value1;
			time2.timing_offset_value3 -= time2.timing_offset_value2;

			up_2s(time2.timing_offset_value1);
			side_2s(time_buf[1]);
			top_2s(time2.timing_offset_value3);
		}
		else if(order_servos2.up_top_side == 1){

			time_buf[2] -= time2.timing_offset_value1;
			time2.timing_offset_value2 -= time2.timing_offset_value3;

			up_2s(time2.timing_offset_value1);
			top_2s(time_buf[2]);
			side_2s(time2.timing_offset_value2);
		}
		else if(order_servos2.side_up_top == 1){

			time_buf[0] -= time2.timing_offset_value2;
			time2.timing_offset_value3 -= time2.timing_offset_value1;

			side_2s(time2.timing_offset_value2);
			up_2s(time_buf[0]);
			top_2s(time2.timing_offset_value3);
		}
		else if(order_servos2.side_top_up == 1){

			time_buf[2] -= time2.timing_offset_value2;
			time2.timing_offset_value1 -= time2.timing_offset_value3;

			side_2s(time2.timing_offset_value2);
			top_2s(time_buf[2]);
			up_2s(time2.timing_offset_value1);
		}
		else if(order_servos2.top_up_side == 1){
			time_buf[0] -= time2.timing_offset_value3;
			time2.timing_offset_value2 -= time2.timing_offset_value1;

			top_2s(time2.timing_offset_value3);
			up_2s(time_buf[0]);
			side_2s(time2.timing_offset_value2);
		}
		else if(order_servos2.top_side_up == 1){

			time_buf[1] -= time2.timing_offset_value3;
			time2.timing_offset_value1 -= time2.timing_offset_value2;

			top_2s(time2.timing_offset_value3);
			side_2s(time_buf[1]);
			up_2s(time2.timing_offset_value1);
		}
	}

	if(leg == 3){

		time3.timing_offset_value3 = TARGET_POSITION_L3T + time_us1;		// UP
		time3.timing_offset_value2 = TARGET_POSITION_L3T + TARGET_POSITION_L3S + time_us2;		// SIDE
		time3.timing_offset_value1 = TARGET_POSITION_L3T + TARGET_POSITION_L3S + TARGET_POSITION_L3U + time_us3;

		time_buf[0] = time3.timing_offset_value1;
		time_buf[1] = time3.timing_offset_value2;
		time_buf[2] = time3.timing_offset_value3;

		order_servos3.up_side_top = (time3.timing_offset_value1 < time3.timing_offset_value2 && time3.timing_offset_value2 < time3.timing_offset_value3) ? 1:0;
		order_servos3.up_top_side = (time3.timing_offset_value1 < time3.timing_offset_value3 && time3.timing_offset_value3 < time3.timing_offset_value2) ? 1:0;
		order_servos3.side_up_top = (time3.timing_offset_value2 < time3.timing_offset_value1 && time3.timing_offset_value1 < time3.timing_offset_value3) ? 1:0;
		order_servos3.side_top_up = (time3.timing_offset_value2 < time3.timing_offset_value3 && time3.timing_offset_value3 < time3.timing_offset_value1) ? 1:0;
		order_servos3.top_up_side = (time3.timing_offset_value3 < time3.timing_offset_value1 && time3.timing_offset_value1 < time3.timing_offset_value2) ? 1:0;
		order_servos3.top_side_up = (time3.timing_offset_value3 < time3.timing_offset_value2 && time3.timing_offset_value2 < time3.timing_offset_value1) ? 1:0;


		LEG_3_DIR |= LEG_3_PIN;
		LEG_3_PORT |= (SERVO_SIDE_3 | SERVO_UP_3 | SERVO_TOP_3);

		if(order_servos3.up_side_top == 1){

			time_buf[1] -= time3.timing_offset_value1;
			time3.timing_offset_value3 -= time3.timing_offset_value2;

			up_3s(time3.timing_offset_value1);
			side_3s(time_buf[1]);
			top_3s(time3.timing_offset_value3);
		}
		else if(order_servos3.up_top_side == 1){

			time_buf[2] -= time3.timing_offset_value1;
			time3.timing_offset_value2 -= time3.timing_offset_value3;

			up_3s(time3.timing_offset_value1);
			top_3s(time_buf[2]);
			side_3s(time3.timing_offset_value2);
		}
		else if(order_servos3.side_up_top == 1){

			time_buf[0] -= time3.timing_offset_value2;
			time3.timing_offset_value3 -= time3.timing_offset_value1;

			side_3s(time3.timing_offset_value2);
			up_3s(time_buf[0]);
			top_3s(time3.timing_offset_value3);
		}
		else if(order_servos3.side_top_up == 1){

			time_buf[2] -= time3.timing_offset_value2;
			time3.timing_offset_value1 -= time3.timing_offset_value3;

			side_3s(time3.timing_offset_value2);
			top_3s(time_buf[2]);
			up_3s(time3.timing_offset_value1);
		}
		else if(order_servos3.top_up_side == 1){
			time_buf[0] -= time3.timing_offset_value3;
			time3.timing_offset_value2 -= time3.timing_offset_value1;

			top_3s(time3.timing_offset_value3);
			up_3s(time_buf[0]);
			side_3s(time3.timing_offset_value2);
		}
		else if(order_servos3.top_side_up == 1){

			time_buf[1] -= time3.timing_offset_value3;
			time3.timing_offset_value1 -= time3.timing_offset_value2;

			top_3s(time3.timing_offset_value3);
			side_3s(time_buf[1]);
			up_3s(time3.timing_offset_value1);
		}
	}

	else if(leg == 4){

		time4.timing_offset_value3 = TARGET_POSITION_L4T + time_us1;		// UP
		time4.timing_offset_value2 = TARGET_POSITION_L4T + TARGET_POSITION_L4S + time_us2;		// SIDE
		time4.timing_offset_value1 = TARGET_POSITION_L4T + TARGET_POSITION_L4S + TARGET_POSITION_L4U + time_us3;

		time_buf[0] = time4.timing_offset_value1;
		time_buf[1] = time4.timing_offset_value2;
		time_buf[2] = time4.timing_offset_value3;

		order_servos4.up_side_top = (time4.timing_offset_value1 < time4.timing_offset_value2 && time4.timing_offset_value2 < time4.timing_offset_value3) ? 1:0;
		order_servos4.up_top_side = (time4.timing_offset_value1 < time4.timing_offset_value3 && time4.timing_offset_value3 < time4.timing_offset_value2) ? 1:0;
		order_servos4.side_up_top = (time4.timing_offset_value2 < time4.timing_offset_value1 && time4.timing_offset_value1 < time4.timing_offset_value3) ? 1:0;
		order_servos4.side_top_up = (time4.timing_offset_value2 < time4.timing_offset_value3 && time4.timing_offset_value3 < time4.timing_offset_value1) ? 1:0;
		order_servos4.top_up_side = (time4.timing_offset_value3 < time4.timing_offset_value1 && time4.timing_offset_value1 < time4.timing_offset_value2) ? 1:0;
		order_servos4.top_side_up = (time4.timing_offset_value3 < time4.timing_offset_value2 && time4.timing_offset_value2 < time4.timing_offset_value1) ? 1:0;


		LEG_4_DIR |= LEG_4_PIN;
		LEG_4_PORT |= (SERVO_SIDE_4 | SERVO_UP_4 | SERVO_TOP_4);

		if(order_servos4.up_side_top == 1){

			time_buf[1] -= time4.timing_offset_value1;
			time4.timing_offset_value3 -= time4.timing_offset_value2;

			up_4s(time4.timing_offset_value1);
			side_4s(time_buf[1]);
			top_4s(time4.timing_offset_value3);
		}
		else if(order_servos4.up_top_side == 1){

			time_buf[2] -= time4.timing_offset_value1;
			time4.timing_offset_value2 -= time4.timing_offset_value3;

			up_4s(time4.timing_offset_value1);
			top_4s(time_buf[2]);
			side_4s(time4.timing_offset_value2);
		}
		else if(order_servos4.side_up_top == 1){

			time_buf[0] -= time4.timing_offset_value2;
			time4.timing_offset_value3 -= time4.timing_offset_value1;

			side_4s(time4.timing_offset_value2);
			up_4s(time_buf[0]);
			top_4s(time4.timing_offset_value3);
		}
		else if(order_servos4.side_top_up == 1){

			time_buf[2] -= time4.timing_offset_value2;
			time4.timing_offset_value1 -= time4.timing_offset_value3;

			side_4s(time4.timing_offset_value2);
			top_4s(time_buf[2]);
			up_4s(time4.timing_offset_value1);
		}
		else if(order_servos4.top_up_side == 1){
			time_buf[0] -= time4.timing_offset_value3;
			time4.timing_offset_value2 -= time4.timing_offset_value1;

			top_4s(time4.timing_offset_value3);
			up_4s(time_buf[0]);
			side_4s(time4.timing_offset_value2);
		}
		else if(order_servos4.top_side_up == 1){

			time_buf[1] -= time4.timing_offset_value3;
			time4.timing_offset_value1 -= time4.timing_offset_value2;

			top_4s(time4.timing_offset_value3);
			side_4s(time_buf[1]);
			up_4s(time4.timing_offset_value1);
		}
	}
}

void Main_Move_Control(){

	if(step_forward == 1){
		position = 0;
		whole = 0;
		step_backward = 0;
		rotateLeftFlag = 0;
		rotateRightFlag = 0;

		switch(GM_leg){
		case 13:
			if(!counter){
				main_TimeCounter(1,LEG_UP_UP_1,0,0);
				main_TimeCounter(3,30,LEG_GAIT_MODE_SIDE_3,LEG_UP_UP_3);
			}
			else if(counter == 1){
				main_TimeCounter(1,LEG_UP_UP_1,LEG_FORWARD_1,LEG_TOP_UP_1);
				main_TimeCounter(3,LEG_TOP_UP_3,LEG_FORWARD_3,LEG_UP_UP_3);
			}
			else if(counter == 2){
				main_TimeCounter(1,LEG_UP_DOWN_1,LEG_FORWARD_1,LEG_TOP_DOWN_1);
				main_TimeCounter(3,LEG_TOP_DOWN_3,LEG_FORWARD_3,LEG_UP_DOWN_3);
			}
			break;
		case 24:
			if(!counter){
				main_TimeCounter(2,LEG_UP_UP_2,20,LEG_GAIT_MODE_SIDE_2);
				main_TimeCounter(4,LEG_UP_UP_4,0,0);
			}
			else if(counter == 1){
				main_TimeCounter(2,LEG_UP_UP_2,LEG_TOP_UP_2,LEG_FORWARD_2);
				main_TimeCounter(4,LEG_UP_UP_4,LEG_FORWARD_4,LEG_TOP_UP_4);
			}
			else if(counter == 2){
				main_TimeCounter(2,LEG_UP_DOWN_2,LEG_TOP_DOWN_2,LEG_FORWARD_2);
				main_TimeCounter(4,LEG_UP_DOWN_4,LEG_FORWARD_4,LEG_TOP_DOWN_4);
			}
			break;
		}
	}
}



void rotate_right(volatile uint8_t data){

	switch(data){
	case 1:
		if(!counter){
			main_TimeCounter(1,LEG_UP_UP_1,0,0);
			main_TimeCounter(3,30,0,LEG_UP_UP_3);
		}
		else if(counter == 1){
			main_TimeCounter(1,LEG_UP_UP_1,LEG_SIDE_RIGHT_1,LEG_TOP_UP_1);
			main_TimeCounter(3,LEG_TOP_UP_3,LEG_SIDE_RIGHT_3,LEG_UP_UP_3);
		}

		else if(counter == 2){
			main_TimeCounter(1,LEG_UP_DOWN_1,LEG_SIDE_RIGHT_1,LEG_TOP_DOWN_1);
			main_TimeCounter(3,LEG_TOP_DOWN_3,LEG_SIDE_RIGHT_3,LEG_UP_DOWN_3);
		}
		break;
	case 2:
		if(!counter){
			main_TimeCounter(2,LEG_UP_UP_2,20,0);
			main_TimeCounter(4,LEG_UP_UP_4,0,0);
		}
		else if(counter == 1){
			main_TimeCounter(2,LEG_UP_UP_2,LEG_TOP_UP_2,LEG_SIDE_RIGHT_2);
			main_TimeCounter(4,LEG_UP_UP_4,LEG_SIDE_RIGHT_4,LEG_TOP_UP_4);
		}

		else if(counter == 2){
			main_TimeCounter(2,LEG_UP_DOWN_2,LEG_TOP_DOWN_2,LEG_SIDE_RIGHT_2);
			main_TimeCounter(4,LEG_UP_DOWN_4,LEG_SIDE_RIGHT_4,LEG_TOP_DOWN_4);
		}
		break;
	}
}

void rotate_flag(volatile uint8_t data){

	switch(data){
	case 1:
		if(!counter){
			main_TimeCounter(1,LEG_UP_UP_1,0,0);
			main_TimeCounter(3,30,0,LEG_UP_UP_3);
		}
		else if(counter == 1){
			main_TimeCounter(1,LEG_UP_UP_1,LEG_SIDE_LEFT_1,LEG_TOP_UP_1);
			main_TimeCounter(3,LEG_TOP_UP_3,LEG_SIDE_LEFT_3,LEG_UP_UP_3);
		}

		else if(counter == 2){
			main_TimeCounter(1,LEG_UP_DOWN_1,LEG_SIDE_LEFT_1,LEG_TOP_DOWN_1);
			main_TimeCounter(3,LEG_TOP_DOWN_3,LEG_SIDE_LEFT_3,LEG_UP_DOWN_3);
		}
		else if(counter == 3){
			counter = 0;
		}
		break;
	case 2:
		if(!counter){
			main_TimeCounter(2,LEG_UP_UP_2,20,0);
			main_TimeCounter(4,LEG_UP_UP_4,0,0);
		}
		else if(counter == 1){
			main_TimeCounter(2,LEG_UP_UP_2,LEG_TOP_UP_2,LEG_SIDE_LEFT_2);
			main_TimeCounter(4,LEG_UP_UP_4,LEG_SIDE_LEFT_4,LEG_TOP_UP_4);
		}

		else if(counter == 2){
			main_TimeCounter(2,LEG_UP_DOWN_2,LEG_TOP_DOWN_2,LEG_SIDE_LEFT_2);
			main_TimeCounter(4,LEG_UP_DOWN_4,LEG_SIDE_LEFT_4,LEG_TOP_DOWN_4);
		}
		break;
	}
}

void Target_Position(uint8_t leg, uint8_t gait_mode, uint8_t whole){

	if(target_leg == 13 && gait_mode == 0 && whole == 0){
		main_TimeCounter(1,LEG_UP_DOWN_1,TARGET_SIDE_1,LEG_TOP_DOWN_1);
		main_TimeCounter(3,LEG_TOP_DOWN_3,TARGET_SIDE_3,LEG_UP_DOWN_3);
	}
	else if(target_leg == 24 && gait_mode == 0 && whole == 0){
		main_TimeCounter(2,LEG_UP_DOWN_2,LEG_TOP_DOWN_2,TARGET_SIDE_2);
		main_TimeCounter(4,LEG_UP_DOWN_4,TARGET_SIDE_4,LEG_TOP_DOWN_4);
	}

	if(target_leg == 13 && gait_mode == 1 && whole == 0){
		main_TimeCounter(1,LEG_UP_DOWN_1,TARGET_SIDE_1,LEG_TOP_DOWN_1);
		main_TimeCounter(3,LEG_GAIT_MODE_TOP_3,LEG_GAIT_MODE_SIDE_3,LEG_GAIT_MODE_UP_3);
	}
	else if(target_leg == 24 && gait_mode == 1 && whole == 0){
		main_TimeCounter(4,LEG_UP_DOWN_4,TARGET_SIDE_4,LEG_TOP_DOWN_4);
		main_TimeCounter(2,LEG_GAIT_MODE_UP_2,LEG_GAIT_MODE_TOP_2,LEG_GAIT_MODE_SIDE_2);
	}
	if(target_leg == 0 && gait_mode == 0 && whole == 1){
		main_TimeCounter(1,LEG_UP_DOWN_1,TARGET_SIDE_1,LEG_TOP_DOWN_1);
		main_TimeCounter(2,LEG_UP_DOWN_2,LEG_TOP_DOWN_2,TARGET_SIDE_2);
		main_TimeCounter(3,LEG_TOP_DOWN_3,TARGET_SIDE_3,LEG_UP_DOWN_3);
		main_TimeCounter(4,LEG_UP_DOWN_4,TARGET_SIDE_4,LEG_TOP_DOWN_4);
	}
}

void transtition_initial_position(uint8_t data){

	switch(data){

	case 1:
		if(!initial_position_cntA){
			main_TimeCounter(1,-250,0,140);//moving_leg(1,-130,0,50);		// up side top	
			main_TimeCounter(3,-100,0,260);//moving_leg(3,-60,0,255);		// top side up
		}
		else if(initial_position_cntA == 1){
			main_TimeCounter(1,-250,0,-150);//moving_leg(1,-130,0,-150);
			main_TimeCounter(3,140,0,260);//moving_leg(3,110,0,255);
		}
		else if(initial_position_cntA == 2){
			main_TimeCounter(1,-250,0,140);//moving_leg(1,-130,0,140);
			main_TimeCounter(3,-100,0,260);//moving_leg(3,-130,0,255);
		}
		else if(initial_position_cntA == 3){
			main_TimeCounter(1,-120,0,120);//moving_leg(1,0,0,140);
			main_TimeCounter(3,-100,0,110);//moving_leg(3,-130,0,120);
		}
		else if(initial_position_cntA == 4){
			main_TimeCounter(1,-80,0,120);//moving_leg(1,0,0,140);
			main_TimeCounter(3,-100,0,80);//moving_leg(3,-130,0,120);
		}
		else if(initial_position_cntA == 5){
			main_TimeCounter(1,0,0,0);//main_TimeCounter(4,260,0,170);
			main_TimeCounter(3,0,0,0);//main_TimeCounter(1,-120,0,110);
		}
		else if(initial_position_cntA == 6){
			main_TimeCounter(1,90,0,-30);//main_TimeCounter(1,-120,0,110);
			main_TimeCounter(3,0,0,-90);//main_TimeCounter(3,-120,0,110);
		}
		break;

	case 2:
		if(!initial_position_cntB){
			main_TimeCounter(2,-260,-130,0);//moving_leg(2,0,0,0);
			main_TimeCounter(4,260,0,130);
		}
		else if(initial_position_cntB == 1){
			main_TimeCounter(2,-260,140,0);
			main_TimeCounter(4,260,0,-200);
		}
		else if(initial_position_cntB == 2){
			main_TimeCounter(2,-260,-110,0);
			main_TimeCounter(4,260,0,160);
		}
		else if(initial_position_cntB == 3){
			main_TimeCounter(2,-100,-110,0);
			main_TimeCounter(4,100,0,130);
		}
		else if(initial_position_cntB == 4){
			main_TimeCounter(2,-260,0,0);//main_TimeCounter(4,260,0,170);
			main_TimeCounter(4,260,0,-30);
		}
		else if(initial_position_cntB == 5){
			main_TimeCounter(2,0,-20,0);
			main_TimeCounter(4,0,0,50);
		}
		else if(initial_position_cntB == 6){
			main_TimeCounter(2,65,50,0);
			main_TimeCounter(4,-20,0,-30);
		}
		break;
	}
}

void step_counter(uint16_t adc1, uint16_t adc2){

	while(adc1 >= 800 && half13 == 1){
		if(counter == 1){
			step_forward = 1;
			GM_leg = 13;	// gait mode leg
			target_leg = 24;
			counter = 0;

			_delay_ms(60);
			counter++;	// 1

			_delay_ms(60);
			counter++; 		// 2

			half13 = 0;
			half24 = 1;
		}
		else if(counter == 2){
			step_forward = 1;
			GM_leg = 13;	// gait mode leg
			target_leg = 24;
			gait_mode = 1;
			counter = 0;

			_delay_ms(60);
			counter++;	// 1

			_delay_ms(60);
			counter++; 		// 2

			half13 = 0;
			half24 = 1;
		}
		adc1 = pomiar(PC1);	

		if(adc1 >= 800){
			step_forward = 1;
			GM_leg = 24;	
			target_leg = 13;
			counter = 0;

			_delay_ms(60);	//1
			counter++;

			_delay_ms(60);	// 2
			counter++;

			_delay_ms(60);	// 2

			half13 = 1;
			half24 = 0;
			adc1 = pomiar(PC1);
		}
	}
	if(adc1 >= 800 && half24 == 1){
		step_forward = 1;
		GM_leg = 24;	// gait mode leg
		target_leg = 13;
		gait_mode = 1;
		counter = 0;

		_delay_ms(60);	//1
		counter++;

		_delay_ms(60);	// 2
		counter++;

		half13 = 1;
		half24 = 0;
		counter = 2;
		_delay_ms(60);
	}

	while(adc2 <= 400 && half13 == 1){
		if(counter == 1){
			left_rotate_A = 0;
			left_rotate_B = 0;
			stepBack_A = 0;
			stepBack_B = 0;
			firstStep = 0;
			secondStep = 0;
			counter = 0;
			targetPosition = 0;
			position = 0;
			right_rotate_B = 1;
			right_rotate_A = 2;

			_delay_ms(70);
			counter++;	// 1

			_delay_ms(70);
			counter++; 		// 2

			half13 = 0;
			half24 = 1;
		}
		adc2 = pomiar(VOLTAGE_S1);

		if(adc2 <= 400){
			_delay_ms(70);
			right_rotate_B = 2;
			right_rotate_A = 1;
			counter = 0;		//0

			_delay_ms(70);	//1
			counter++;

			_delay_ms(70);	// 2
			counter++;

			_delay_ms(70);	// 2

			half13 = 1;
			half24 = 0;
			adc2 = pomiar(VOLTAGE_S1);
		}
	}
	if(adc2 <= 400 && half24 == 1){
		left_rotate_A = 0;
		left_rotate_B = 0;
		firstStep = 0;
		secondStep = 0;
		stepBack_A = 0;
		stepBack_B = 0;
		targetPosition = 0;
		right_rotate_B = 2;
		right_rotate_A = 1;
		counter = 0;

		_delay_ms(70);	//1
		counter++;

		_delay_ms(70);	// 2
		counter++;

		half13 = 1;
		half24 = 0;
		_delay_ms(70);
	}

	while(adc2 >= 800 && half13 == 1){
		if(counter == 1){
			right_rotate_A = 0;
			right_rotate_B = 0;
			stepBack_A = 0;
			stepBack_B = 0;
			firstStep = 0;
			secondStep = 0;
			counter = 0;
			targetPosition = 0;
			position = 0;
			left_rotate_A = 1;
			left_rotate_B = 2;

			_delay_ms(70);
			counter++;	// 1

			_delay_ms(70);
			counter++; 		// 2

			half13 = 0;
			half24 = 1;
		}
		adc2 = pomiar(VOLTAGE_S1);

		if(adc2 >= 800){
			_delay_ms(70);
			left_rotate_A = 2;
			left_rotate_B = 1;
			counter = 0;		//0

			_delay_ms(70);	//1
			counter++;

			_delay_ms(70);	// 2
			counter++;

			_delay_ms(70);	// 2

			half13 = 1;
			half24 = 0;
			adc2 = pomiar(VOLTAGE_S1);
		}
	}
	if(adc2 >= 800 && half24 == 1){
		right_rotate_A = 0;
		right_rotate_B = 0;
		firstStep = 0;
		secondStep = 0;
		stepBack_A = 0;
		stepBack_B = 0;
		targetPosition = 0;
		counter = 0;
		left_rotate_A = 2;
		left_rotate_B = 1;

		_delay_ms(70);	//1
		counter++;

		_delay_ms(70);	// 2
		counter++;

		half13 = 1;
		half24 = 0;
		_delay_ms(70);
	}

	if(position == 1){
		whole = 1;
		gait_mode = 0;
		target_leg = 0;

		/*initial_position_cntA = 0;
		initial_position_cntB = 0;
		_delay_ms(200);
		initial_position_cntA = 1;
		initial_position_cntB = 1;
		_delay_ms(400);
		initial_position_cntA = 2;
		initial_position_cntB = 2;
		_delay_ms(300);
		initial_position_cntA = 3;
		initial_position_cntB = 3;
		_delay_ms(500);
		initial_position_cntB = 4;
		initial_position_cntA = 4;
		_delay_ms(250);
		initial_position_cntB = 5;
		initial_position_cntA = 4;
		_delay_ms(80);
		initial_position_cntB = 6;
		initial_position_cntA = 5;*/
	}
}

void side_1s(uint16_t time){
	czekaj_us(time);
	LEG_1_PORT_B &= ~SERVO_SIDE_1;
}

void up_1s(uint16_t time){
	czekaj_us(time);
	LEG_1_PORT_B &= ~SERVO_UP_1;
}

void top_1s(uint16_t time){
	czekaj_us(time);
	LEG_1_PORT_A &= ~SERVO_TOP_1;
}

void side_2s(uint16_t time){
	czekaj_us(time);
	LEG_2_PORT &= ~SERVO_SIDE_2;
}

void up_2s(uint16_t time){
	czekaj_us(time);
	LEG_2_PORT &= ~SERVO_UP_2;
}

void top_2s(uint16_t time){
	czekaj_us(time);
	LEG_2_PORT &= ~SERVO_TOP_2;
}

void side_3s(uint16_t time){
	czekaj_us(time);
	LEG_3_PORT &= ~SERVO_SIDE_3;
}

void up_3s(uint16_t time){
	czekaj_us(time);
	LEG_3_PORT &= ~SERVO_UP_3;
}

void top_3s(uint16_t time){
	czekaj_us(time);
	LEG_3_PORT &= ~SERVO_TOP_3;
}

void side_4s(uint16_t time){
	czekaj_us(time);
	LEG_4_PORT &= ~SERVO_SIDE_4;
}

void up_4s(uint16_t time){
	czekaj_us(time);
	LEG_4_PORT &= ~SERVO_UP_4;
}

void top_4s(uint16_t time){
	czekaj_us(time);
	LEG_4_PORT &= ~SERVO_TOP_4;
}

void czekaj_us(volatile uint16_t usekundy){
	while(usekundy >= 1){
		_delay_us(1);
		usekundy--;
	}
}

void voltage_charging(int32_t voltages){

	int32_t voltage_s3;
	voltages = voltages*100;

	voltage_s3 = ((voltages*VREF/100024)*2)-14;

	if(voltage_s3 <= 310) WARNING_LED_ON;
	else WARNING_LED_OFF;
}

ISR(TIMER1_COMPA_vect){

	Target_Position(target_leg, gait_mode, whole);
	Main_Move_Control();
}
