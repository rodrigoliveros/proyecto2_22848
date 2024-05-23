#include "ServoControl.h"

void PWM_init(void){
	// PB1 | PB2
	DDRB |= ( 1 << DDB2)|(1 << DDB1);
	TCNT1 = 0; // reset
	ICR1 = 39999; // TOP
	TCCR1A =  (1 << COM1A1) | (1 << COM1B1) | (0 << COM1A0) ; // low --> Compare Match
	TCCR1A |=  (1 << WGM11) | (0 << WGM10) ; // Fast PWM TOP ICR1
	TCCR1B = (1 << WGM13) | (1 << WGM12); // Fast PWM TOP ICR1
	TCCR1B |= (0 << CS12) | (1 << CS11) | ( 0 << CS10 ); // Prescaler 8
}
void servo_writeA(float adc_Value){
	OCR1A = map(adc_Value, 0, 1023, 1000, 2000);
}
void servo_writeB(float adc_Value){
	OCR1B = map(adc_Value, 0, 1023, 2800, 4000);
}

void PWM0_init(void){
	// PD5 | PD6
	DDRD |= (1 << DDD6)|(1 << DDD5);
	TCCR0A |= (1 << WGM01)|(1 << WGM00); //PWM MODO FAST
	TCCR0A |= (1 << COM0A1);//PWM NO INVERTIDO A
	TCCR0A |= (1 << COM0B1);//PWM NO INVERTIDO B
	TCCR0B |= (1 << CS02)|(1 << CS00); //prescaler 1024
}
	
void servo_writeC(float adc_Value){
	OCR0A = map(adc_Value, 0, 1023, 6, 50);
}

void servo_writeD(float adc_Value){
	OCR0B = map(adc_Value, 0, 1023, 30, 50);
}
float map(float x, float in_min, float in_max, float out_min, float out_max){
	return ((x - in_min)*(out_max - out_min)/(in_max - in_min)) + out_min;
}