/*******************************************************
Project :	Smart Traffic Light
Version :	v1.0
Date    :	11/23/2018
Author  :	Zikora Agbapu
Company :	Kennesaw State University
Comments:


Chip type               : ATmega32
Program type            : Application
AVR Core Clock frequency: 10.000000 MHz
Memory model            : Small
External RAM size       : 0
Data Stack size         : 256
*******************************************************/

#define F_CPU 1000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

//Creating sleep function to control the delay for the lights
void sleep(uint8_t millisec)
{
while(millisec)
{
_delay_ms(1000);
millisec--;
}
}

int main(void)
{
//Connecting the Pins of the traffic lights to the pins of the MCU	
DDRA |=1<<PA5;		 //enable pin 35 red1
DDRA |=1<<PA4;		 //enable pin 36 yellow1
DDRA |=1<<PA3;		 //enable pin 37 green1
DDRB |=1<<PB3;		 //enable pin 4 red2
DDRB |=1<<PB2;		 //enable pin 3 yellow2
DDRB |=1<<PB1;		 //enable pin 2 green2

//Setting PORTC as input for infrared sensor buttons
DDRC = 0x00;	

//Defining variables
int time_ns;
int time_ew;
int time_yellow = 2;
int time_red = 1.5;
int time_normal = 5;
int time_extended = 7;


//Set up for the ultrasonic sensor button
DDRD = 0<<PD2;		// Set PD2 as input (Using for interrupt INT0)
PORTD = 1<<PD2;		// Enable PD2 pull-up resistor

//Setting up the interrupts	
GICR = 1<<INT0;					// Enable INT0
MCUCR = 1<<ISC01 | 0<<ISC00;	// Trigger INT0 on rising edge
	
sei();				//Enable Global Interrupt

//Initializing the traffic lights as all off
 PORTA = 0x00;
 PORTB = 0x00;
	
while(1)
{
//Stage 1: Keeping Green on EW and Red on NS
	PORTA |= (1<<PA5);			//on red1 
	PORTB |= (1<<PB1);			//on green2 
	PORTB &= ~(1<<PB3);			//off red2
	PORTA &= ~(1<<PA4);			//off yellow1
	PORTB &= ~(1<<PB2);			//off yellow2
	PORTA &= ~(1<<PA3);			//off green1
	
	//Polling to find the value 
	if(PINC & 0b00000011)
		time_ew = time_extended;
	else
		time_ew = time_normal;
	sleep(time_ew);
	
//Stage2: Turning EW to yellow & green
	PORTB |= (1<<PB2);			//on yellow2
	sleep(time_yellow);
	
//Stage 3: Turning EW to red, while NS is red	
	PORTB &= ~(1<<PB2);			//off yellow2
	PORTB &= ~(1<<PB1);			//off green2
	PORTB |= (1<<PB3);			//on red2
	sleep(time_red);
	
//Stage 4: Turning NS to green	
	PORTA &= ~(1<<PA5);			//off red1
	PORTA |= (1<<PA3);			//on green1
	
	//Polling to find the value
	if(PINC & 0b00001100)
	time_ns = time_extended;
	else
	time_ns = time_normal;
	sleep(time_ns);

//Stage 5: Turning NS to green & yellow
	PORTA |= (1<<PA4);			//on yellow1 
	sleep(time_yellow);
	
//Stage 6: Turning NS red while EW is red	
	PORTA &= ~(1<<PA3);			//off green1
	PORTA &= ~(1<<PA4);			//off yellow1
	PORTA |= (1<<PA5);			//on red1
	sleep(time_red);
	
//Stage 1: Keeping Green on NS and Red on EW
	PORTA |= (1<<PA5);			//on red1 
	PORTB |= (1<<PB1);			//on green2 
	PORTB &= ~(1<<PB3);			//off red2
	PORTA &= ~(1<<PA4);			//off yellow1
	PORTB &= ~(1<<PB2);			//off yellow2
	PORTA &= ~(1<<PA3);			//off green1

}
}

ISR(INT0_vect)
{
		int time_emgcy = 3;
		PORTA |= (1<<PA5); //on red1 pin28
		PORTB |= (1<<PB3); //on red2
		PORTB &= ~(1<<PB1); //off green2
		PORTA &= ~(1<<PA4); //off yellow1
		PORTB &= ~(1<<PB2); //off yellow2
		PORTA &= ~(1<<PA3); //off green1
		sleep(time_emgcy);
	}