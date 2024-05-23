/*
 * proyecto_2_22848.c
 *
 * Created: 19/04/2024 14:05:09
 * Author : rodri
 */ 

#define F_CPU 16000000
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "ServoControl/ServoControl.h"
#include "PWM0/PWM0.h"
#include <avr/eeprom.h>

float adcValue1 = 0;
float adcValue2 = 0;
float adcValue3 = 0;
float adcValue4 = 0;
void ADC_init(void);
void ExtInt_init(void);
void Botones_init(void);
void MManual(void);
uint16_t adcRead(uint8_t);

int state = 0;
uint8_t next_state = 0;
int flag = 0;

int main(void)
{
	
	DDRD = 0xFF;
	DIDR0 |= (1 << ADC3D) | (1 << ADC2D) | (1 << ADC1D) | (1 << ADC0D);
	DDRB |= (1 << DDB3) | (1 << DDB4);
	PORTB &= ~(1 << PORTB3);
	PORTB &= ~(1 << PORTB4);
	state = 0;
	ADC_init();
	PWM_init();
	PWM0_init();
	ExtInt_init();
	Botones_init();
	sei();

	while (1)
	{	
		
		switch(state){
			
			case 0:
			
				PORTB &= ~(1 << PORTB3);
				PORTB &= ~(1 << PORTB4);
				
				adcValue1 = adcRead(0);
				servo_writeA(adcValue1);
				_delay_ms(10);
				adcValue2 = adcRead(1);
				servo_writeB(adcValue2);
				_delay_ms(10);
				adcValue3 = adcRead(2);
				servo_writeC(adcValue3);
				_delay_ms(10);
				adcValue4 = adcRead(3);
				servo_writeD(adcValue4);
				_delay_ms(10);
				
				break;
			
			case 1: //Guardar
				PORTB |= (1 << PORTB3);
				break;	
				
			case 2: //Reproducir
				PORTB |= (1 << PORTB4);
				PORTB &= ~(1 << PORTB3);
				break;
			
			case 3: //Borrar
				PORTB |= (1 << PORTB3);
				PORTB |= (1 << PORTB4);
				break;
				
			case 4: //Adafruit
				PORTB |= (1 << PORTB4);
				PORTB |= (1 << PORTB3);
				_delay_ms(500);
				PORTB &= ~(1 << PORTB4);
				PORTB &= ~(1 << PORTB3);
				_delay_ms(500);
				break;
				
			default:
				PORTB &= ~(1 << PORTB3);
				PORTB &= ~(1 << PORTB4);
				break;
		
		}//switch_case
	}//while
}//main
void ADC_init(void){
	ADMUX |= (1<<REFS0);	// VCC REF
	ADMUX &= ~(1<<REFS1);
	ADMUX &= ~(1<<ADLAR);	// 10 bits
	// PRESCALER 128 > 16M/128 = 125KHz
	ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
	ADCSRA |= (1<<ADEN);	// ADC ON
}
uint16_t adcRead(uint8_t canal){
	ADMUX = (ADMUX & 0xF0)|canal;	// selección de canal
	ADCSRA |= (1<<ADSC);	// inicia conversión
	while((ADCSRA)&(1<<ADSC));	// hasta finalizar conversión
	return(ADC);
}
void ExtInt_init(void){
	//CONFIGURADO PARA PD2
	EICRA |= (1<< ISC00)|(1 << ISC01);
	DDRD &= ~(1 << DDD2);
	PORTD |= (1 << DDD2);
	EIMSK |= (1 << INT0);
}
void Botones_init(void) {//de entrada
	
	DDRD &= ~(1<<DDD3);	
	DDRD &= ~(1<<DDD4);		
	PORTD |= (1<<DDD3)|(1<<DDD4);		//PD3 y PD5 se inicializan en 0

	PCMSK2 |= (1<<DDD3)|(1<<DDD4);
	PCICR |= (1<<PCIE2);
}
ISR(INT0_vect)	{
	state++;
	if (state == 5) {
		 state = 0;
	}
}
ISR(PCINT2_vect) {
	
	switch(state){
		
		case 1:
		if(flag ==0){
			if((PIND & (1<<PIND3))==0){
				eeprom_write_byte((uint8_t*)0x00, adcRead(0));
				eeprom_write_byte((uint8_t*)0x01, adcRead(1));
				eeprom_write_byte((uint8_t*)0x02, adcRead(2));
				eeprom_write_byte((uint8_t*)0x03, adcRead(3));
				flag++;
			}
		}
		if(flag	== 1){
			if((PIND & (1<<PIND3))==0){
				eeprom_write_byte((uint8_t*)0x04, adcRead(0));
				eeprom_write_byte((uint8_t*)0x05, adcRead(1));
				eeprom_write_byte((uint8_t*)0x06, adcRead(2));
				eeprom_write_byte((uint8_t*)0x07, adcRead(3));
				flag++;
			}
		}
		if(flag == 2){
			if((PIND & (1<<PIND3))==0){
				eeprom_write_byte((uint8_t*)0x08, adcRead(0));
				eeprom_write_byte((uint8_t*)0x09, adcRead(1));
				eeprom_write_byte((uint8_t*)0x0A, adcRead(2));
				eeprom_write_byte((uint8_t*)0x0B, adcRead(3));
				flag++;
			}
		}
		if(flag == 3){
			if((PIND & (1<<PIND3))==0){
				eeprom_write_byte((uint8_t*)0x0C, adcRead(0));
				eeprom_write_byte((uint8_t*)0x0D, adcRead(1));
				eeprom_write_byte((uint8_t*)0x0E, adcRead(2));
				eeprom_write_byte((uint8_t*)0x0F, adcRead(3));
				flag = 0;
			}
		}
		break;
		case 2:
		
			if((PIND & (1<<PIND3))==0){
				//_delay_ms(10);
				OCR1A = eeprom_read_byte((uint8_t*)0x00);
				OCR1B = eeprom_read_byte((uint8_t*)0x01);
				OCR0A = eeprom_read_byte((uint8_t*)0x02);
				OCR0B = eeprom_read_byte((uint8_t*)0x03);
				_delay_ms(10);
				OCR1A = eeprom_read_byte((uint8_t*)0x04);
				OCR1B = eeprom_read_byte((uint8_t*)0x05);
				OCR0A = eeprom_read_byte((uint8_t*)0x06);
				OCR0B = eeprom_read_byte((uint8_t*)0x07);
				_delay_ms(10);
				OCR1A = eeprom_read_byte((uint8_t*)0x08);
				OCR1B = eeprom_read_byte((uint8_t*)0x09);
				OCR0A = eeprom_read_byte((uint8_t*)0x0A);
				OCR0B = eeprom_read_byte((uint8_t*)0x0B);
				_delay_ms(10);
				OCR1A = eeprom_read_byte((uint8_t*)0x0C);
				OCR1B = eeprom_read_byte((uint8_t*)0x0D);
				OCR0A = eeprom_read_byte((uint8_t*)0x0E);
				OCR0B = eeprom_read_byte((uint8_t*)0x0F);
				_delay_ms(10);
			}
	}//state
}