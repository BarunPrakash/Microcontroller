#include<avr/io.h>
#include<ext_interrupt.h>

void enable_INT0(int i)
{
	DDRD&=~(1<<2);	
	PORTD|=(1<<2);
	GICR|=(1<<INT0);
	GIFR|=(1<<INTF0);
	switch(i)
	{
		case 0:				//low level
			MCUCR&=~(1<<ISC00);
			MCUCR&=~(1<<ISC01);
			break;
		case 1:				//any logical change
			MCUCR|=(1<<ISC00);
			MCUCR&=~(1<<ISC01);
			break;
		case 2:				//falling edge
			MCUCR&=~(1<<ISC00);
			MCUCR|=(1<<ISC01);
			break;
		case 3:				//rising edge
			MCUCR|=(1<<ISC00);
			MCUCR|=(1<<ISC01);
			break;
		default:
			break;
	}		
}

void disable_INT0()
{
	GICR&=~(1<<INT0);
	GIFR&=~(1<<INTF0);
}

void enable_INT1(int i)
{
	DDRD&=~(1<<3);	
	PORTD|=(1<<3);
	GICR|=(1<<INT1);
	GIFR|=(1<<INTF1);
	switch(i)
	{
		case 0:				//low level
			MCUCR&=~(1<<ISC10);
			MCUCR&=~(1<<ISC11);
			break;
		case 1:				//any logical change
			MCUCR|=(1<<ISC10);
			MCUCR&=~(1<<ISC11);
			break;
		case 2:				//falling edge
			MCUCR&=~(1<<ISC10);
			MCUCR|=(1<<ISC11);
			break;
		case 3:				//rising edge
			MCUCR|=(1<<ISC10);
			MCUCR|=(1<<ISC11);
			break;
		default:
			break;
	}		
}

void disable_INT1()
{
	GICR&=~(1<<INT1);
	GIFR&=~(1<<INTF1);
}


void enable_INT2(int i)
{
	DDRB&=~(1<<2);
	PORTB|=(1<<2);
	GICR|=(1<<INT2);
	GIFR|=(1<<INTF2);
	switch(i)
	{
		case 0:				//falling edge
			MCUCSR&=~(1<<ISC2);
			break;
		case 1:				//rising edge
			MCUCSR|=(1<<ISC2);
			break;
		default:
			break;
	}		
}


void disable_INT2()
{
	GICR&=~(1<<INT2);
	GIFR&=~(1<<INTF2);
}