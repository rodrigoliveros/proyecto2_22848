#ifndef SERVOCONTROL_H_
#define SERVOCONTROL_H_

#include <avr/io.h>

void PWM_init(void);
void PWM0_init(void);
void servo_writeA(float valADC);
void servo_writeB(float valADC);
void servo_writeC(float adc_Value);
void servo_writeD(float adc_Value);

float map(float x, float in_min, float in_max, float out_min, float out_max);

#endif /* SERVOCONTROL_H_ */