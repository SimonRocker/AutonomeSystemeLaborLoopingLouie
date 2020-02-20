/**
    File: timer.cpp
    Authors: Oktavian Gniot
    Version: 0.4.2 beta 2017-05-05

    Description: Implementation file for the battery module.
                 This file contains the implementation of the functions for the
                 battery module.
                 It is currently only a placeholder.
*/



#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>

#include "MainClock.h"


MainClock * mainClock;

#define MAIN_TIMER_PERIOD 10

#include <arduino.h>


void MainClock::startTimer(void) 
{

	cli();

	mainClock = this;
	
	TCCR3A =   (1<<WGM31) | (0<<WGM30);
	TCCR3B = (2<<CS32) | (1<<CS31) | (0<<CS30)| ( 1<<WGM33 ) | (1<<WGM32 );
	
	TCNT3H = 0;
	TCNT3L = 0;

	ICR3 = MAIN_TIMER_PERIOD;

    TIMSK3 |= (1 << TOIE3);


	OCR3AH=0;
	OCR3AL=0;

	OCR3BH=0;
	OCR3BL=0;


    sei();                      // enable global interrupt


   
}


void MainClock::setTick(void)
{

	if (this->status.systemTick!=0)
	{
		this->status.systemTickOverflow=1;

	}
	this->status.systemTick=1;

}



bool MainClock::isTick(void)
{

	if (this->status.systemTick==1)
	{
		this->status.systemTick=0;
		return(true);
	}
	
	
	return(false);
}


bool MainClock::hasOverflow(void)
{
	if (this->status.systemTickOverflow==1)
	{
		return(true);
	}
	
	
	return(false);
}


void MainClock::clearOverflow(void)
{
	this->status.systemTickOverflow=0;
}




ISR(TIMER3_OVF_vect) 
{
static uint8_t preScaler;

	if (preScaler & 0x2)
	{
		mainClock->setTick();
		preScaler = 0;
	}	
	
	preScaler++;
	
	ADCSRA |=(1<<ADSC);		// start ADC conversion
	
	
}
