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

#define MAIN_TIMER_PERIOD 650

#include <arduino.h>


void MainClock::startTimer(void) 
{

	cli();

	mainClock = this;
	
	TCCR1A =   (1<<WGM11) | (0<<WGM10);
	TCCR1B = (2<<CS12) | (1<<CS11) | (0<<CS10)| ( 1<<WGM13 ) | (1<<WGM12 );
	
	TCNT1H = 0;
	TCNT1L = 0;

	ICR1 = MAIN_TIMER_PERIOD;

    TIMSK1 |= (1 << TOIE1);


	OCR1AH=0;
	OCR1AL=0;

	OCR1BH=0;
	OCR1BL=0;


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
