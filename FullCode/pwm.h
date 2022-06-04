
/*
 * pwm.h
 *
 * Created: 5/22/2022 3:17:18 PM
 *  Author: ssrag
 */ 
//fast pwm mode
void PWM_init(){
	/*set fast PWM mode with non-inverted output*/
	TCCR0 = (1<<WGM00) | (1<<WGM01) | (1<<COM01) | (1<<CS00);
	DDRB|=(1<<PB3);  /*set OC0 pin as output*/
}
