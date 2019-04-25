/*
 * sterowanie.h
 *
 *  Created on: 04-07-2014
 *      Author: Miccaldo
 */

#ifndef STEROWANIE_H_
#define STEROWANIE_H_

#include <avr/io.h>

		// ustawianie TOP, linia prosta z laczeniem i 500us
//	*********	DEFINICJE SERWOMECHANIZMOW    *********

#define TARGET_POSITION_L1U	630		// UP
#define TARGET_POSITION_L1S	100		// SIDE
#define TARGET_POSITION_L1T	260	// TOP

#define TARGET_POSITION_L2U	630			//650//550	800// UP
#define TARGET_POSITION_L2T	20			//100//300		// TOP
#define TARGET_POSITION_L2S	170

#define TARGET_POSITION_L3T	590		// TOP
#define TARGET_POSITION_L3S	160		// SIDE
#define TARGET_POSITION_L3U	60		// UP

#define TARGET_POSITION_L4T	630	//UP
#define TARGET_POSITION_L4S	260 //SIDE
#define TARGET_POSITION_L4U	110	//TOP

// PIERWSZA NOGA

#define LEG_2_DIR		DDRC
#define LEG_2_PORT		PORTC
#define LEG_2_PIN		(1<<PC2)|(1<<PC3)|(1<<PC4)

#define SERVO_SIDE_2	(1<<PC4)
#define SERVO_UP_2		(1<<PC3)
#define SERVO_TOP_2		(1<<PC2)

// DRUGA NOGA

#define LEG_1_DIR_A		DDRB
#define LEG_1_PORT_A	PORTB
#define LEG_1_PIN_A		(1<<PB5)

#define LEG_1_DIR_B		DDRD
#define LEG_1_PORT_B	PORTD
#define LEG_1_PIN_B		(1<<PD0)|(1<<PD1)

#define SERVO_SIDE_1	(1<<PD1)
#define SERVO_UP_1		(1<<PD0)
#define SERVO_TOP_1		(1<<PB5)

// CZWARTA NOGA

#define LEG_4_DIR		DDRD
#define LEG_4_PORT		PORTD
#define LEG_4_PIN		(1<<PD2)|(1<<PD3)|(1<<PD4)

#define SERVO_SIDE_4	(1<<PD2)
#define SERVO_UP_4		(1<<PD4)
#define SERVO_TOP_4		(1<<PD3)

// TRZECIA NOGA

#define LEG_3_DIR		DDRD
#define LEG_3_PORT		PORTD
#define LEG_3_PIN		(1<<PD5)|(1<<PD6)|(1<<PD7)

#define SERVO_SIDE_3	(1<<PD5)
#define SERVO_UP_3		(1<<PD6)
#define SERVO_TOP_3		(1<<PD7)

// ******************************************

//	********* ADC *********

#define JOYSTICK_DIR	DDRC
#define JOYSTICK_PORT	PORTC
#define JOYSTICK_PIN	((1<<PC5)|(1<<PC1))

#define JOYSTICK_AXIS_1	PC5
#define JOYSTICK_AXIS_2	PC1

#define VOLTAGE_S1_DIR	DDRC
#define VOLTAGE_S1_PORT	PORTC
#define VOLTAGE_S1_PIN	(1<<PC0)

#define VOLTAGE_S1	(PC0)

#define VREF 256
#define REF256 (1<<REFS0) | (1<<REFS1)

// ******************************************

#define WARNING_LED_OUT		DDRB |= (1<<PB4)
#define WARNING_LED_ON		PORTB |= (1<<PB4)
#define WARNING_LED_OFF		PORTB &= ~(1<<PB4)

// **************** PRZYCISK ******************

#define KEY_PORT	PORTB
#define KEY_PIN		(1<<PB2)



// **************** CZASY IMPULSÓW ******************

// **************** DO £A¯ENIA DO PRZODU ******************
#define LEG_FORWARD_1		-60
#define LEG_BACKWARD_1		 40

#define LEG_UP_UP_1			30//-90
#define LEG_UP_DOWN_1		120// 90

#define LEG_TOP_UP_1		-80//-110
#define LEG_TOP_DOWN_1		-50//-30
// ---------------------------------------------------
#define LEG_FORWARD_3		 20
#define LEG_BACKWARD_3		-40

#define LEG_UP_UP_3			 -30//90
#define LEG_UP_DOWN_3		-150//-140

#define LEG_TOP_UP_3		50 //100
#define LEG_TOP_DOWN_3		 20
// ---------------------------------------------------
#define LEG_FORWARD_2		-20
#define LEG_BACKWARD_2		40

#define LEG_UP_UP_2			30// -140
#define LEG_UP_DOWN_2		100// 100

#define LEG_TOP_UP_2		40// 100
#define LEG_TOP_DOWN_2		 55
// ---------------------------------------------------
#define LEG_FORWARD_4		 80
#define LEG_BACKWARD_4		-40

#define LEG_UP_UP_4			50// 140
#define LEG_UP_DOWN_4		-30// -10

#define LEG_TOP_UP_4		-80 //-160
#define LEG_TOP_DOWN_4		  -50 //-10
// ---------------------------------------------------
// **************** DO POZYCJI DOCELOWEJ ******************
#define TARGET_SIDE_1		0
#define TARGET_SIDE_2		60
#define TARGET_SIDE_3		-40
#define TARGET_SIDE_4		0
// ---------------------------------------------------

// **************** DO OBROTU ******************
#define LEG_SIDE_RIGHT_1 -50
#define LEG_SIDE_RIGHT_2 -50
#define LEG_SIDE_RIGHT_3 -50
#define LEG_SIDE_RIGHT_4 -100

#define LEG_SIDE_LEFT_1 0
#define LEG_SIDE_LEFT_2 0
#define LEG_SIDE_LEFT_3 0
#define LEG_SIDE_LEFT_4 0
// ---------------------------------------------------
//**************** DO ODEPCHNIECIA TYLNEGO******************
#define LEG_GAIT_MODE_SIDE_3	-60
#define LEG_GAIT_MODE_UP_3		-170
#define LEG_GAIT_MODE_TOP_3	  	130

#define LEG_GAIT_MODE_SIDE_2	60
#define LEG_GAIT_MODE_UP_2		100
#define LEG_GAIT_MODE_TOP_2	  	150



void initalize();
void main_TimeCounter(volatile uint8_t leg, volatile int16_t time_us1, volatile int16_t time_us2, volatile int16_t time_us3);
void rotate_right(volatile uint8_t data);
void rotate_left(volatile uint8_t data);
void Main_Move_Control();
void Target_Position(uint8_t leg, uint8_t step, uint8_t whole);
void key_on(uint8_t data);
void voltage_charging(int32_t voltages);
void czekaj_us(volatile uint16_t usekundy);
void step_counter(uint16_t adc1, uint16_t adc2);
uint16_t pomiar(uint8_t kanal);
void side_1s(uint16_t time);
void up_1s(uint16_t time);
void top_1s(uint16_t time);
void side_2s(uint16_t time);
void up_2s(uint16_t time);
void top_2s(uint16_t time);
void side_3s(uint16_t time);
void up_3s(uint16_t time);
void top_3s(uint16_t time);
void side_4s(uint16_t time);
void up_4s(uint16_t time);
void top_4s(uint16_t time);

#endif /* STEROWANIE_H_ */





