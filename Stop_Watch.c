/*
 * Stop_Watch.c
 *
 *  Created on: Sep 20, 2021
 *      Author: Dell
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

unsigned char second =0;
unsigned char minute=0;
unsigned char hour=0;

ISR (INT0_vect){
	second = 0;
	minute = 0;
	hour = 0;
}

ISR (INT1_vect){
	TCCR1B = (~(1<<CS12)) & (~(1<<CS11)) & (~(1<<CS10));
}

ISR (INT2_vect){
	TCCR1B = (1<<WGM12) | (1<<CS11) | (11<<CS10);
}

ISR (TIMER1_COMPA_vect){
	second++;
	if (second == 60){
		second = 0;
		minute++;
	}
	if (minute == 60){
		second = 0;
		minute = 0;
		hour++;
	}
	if (hour == 23){
		hour = 0;
		second = 0;
		minute = 0;
	}
}

void INT0_Init(void){
	DDRD &= (~(1<<PD2));
	PORTD |= (1<<PD2);
	GICR |= (1<<INT0);
	MCUCR |= (1<<ISC01);
	MCUCR &=(~(1<<ISC00));
}

void INT1_Init(void) {
	DDRD &= (~(1 << PD3));
	GICR |= (1 << INT1);
	MCUCR |= (1<<ISC11) |(1<<ISC10);
}

void INT2_Init(void){
	DDRB &= (~(1<<PB2));
	PORTB |= (1<<PB2);
	GICR |= (1<<INT2);
	MCUCSR &= (~(1<<ISC2));
}

void Timer1_CTC_Init(void){
	TCCR1A = (1<<COM1A0) | (1<<COM1B0) | (1<<FOC1A) | (1<<FOC1B);
	TCCR1B = (1<<WGM12) | (1<<CS11) | (1<<CS10);
	TCNT1 = 0;
	OCR1A = 15625;
	TIMSK |= (1<<OCIE1A);
}

int main(){
	DDRA = 0xFF;
	PORTA = 0xFF;
	DDRC = 0x0F;
	PORTC = 0xF0;
    SREG |= (1<<7);

    INT0_Init();
    INT1_Init();
    INT2_Init();
    Timer1_CTC_Init();

    while (1) {
		PORTA = (1 << 5);
		PORTC = second % 10;
		_delay_ms(5);
		PORTA = (1 << 4);
		PORTC = second / 10;
		_delay_ms(5);
		PORTA = (1 << 3);
		PORTC = minute % 10;
		_delay_ms(5);
		PORTA = (1 << 2);
		PORTC = minute / 10;
		_delay_ms(5);
		PORTA = (1 << 1);
		PORTC = hour % 10;
		_delay_ms(5);
		PORTA = (1 << 0);
		PORTC = hour / 10;
		_delay_ms(5);
	}
}
