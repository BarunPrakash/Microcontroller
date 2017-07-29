/*
 * gps_test.c
 *
 *  Created on: Jul 1, 2014
 *      Author: Barun
 */

#define F_CPU 12000000UL

#include<avr/io.h>
#include<util/delay.h>

#define USART_BAUDRATE 4800
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)


#define LCD_DATA PORTA //LCD data port

#define ctrl PORTB
#define en PB2 //enable signal
#define rw PB1 //read/write signal
#define rs PB0 //resister select signal

void LCD_cmd(unsigned char cmd);
void init_LCD(void);
void LCD_write(unsigned char data);
void LCD_write_string(unsigned char *str);

void usart_init();
unsigned int usart_getch();

unsigned char value,i,lati_value[15],lati_dir, longi_value[15], longi_dir, alti[5] ;

int main(void)
{
DDRA=0xff; //LCD_DATA port as out put port
DDRB=0x07; //ctrl as out put
init_LCD(); //initialization of LCD
_delay_ms(50); // delay of 50 mili seconds
LCD_write_string("we at");
LCD_cmd(0xC0);
usart_init(); // initialization of USART
while(1)
{
value=usart_getch();
if(value=='$')
{
value=usart_getch();
if(value=='G')
{
value=usart_getch();
if(value=='P')
{
value=usart_getch();
if(value=='G')
{
value=usart_getch();
if(value=='G')
{
value=usart_getch();
if(value=='A')
{
value=usart_getch();
if(value==',')
{
value=usart_getch();
while(value!=',')
{
value=usart_getch();
}
lati_value[0]=usart_getch();
value=lati_value[0];
for(i=1;value!=',';i++)
{
lati_value[i]=usart_getch();
value=lati_value[i];
}
lati_dir=usart_getch();
value=usart_getch();
while(value!=',')
{
value=usart_getch();
}
longi_value[0]=usart_getch();
value=longi_value[0];
for(i=1;value!=',';i++)
{
longi_value[i]=usart_getch();
value=longi_value[i];
}
longi_dir=usart_getch();
LCD_cmd(0x01);
_delay_ms(1);
LCD_cmd(0x80);
_delay_ms(1000);
i=0;
while(lati_value[i]!='\0')
{
LCD_write(lati_value[j]);
j++;
}
LCD_write(lati_dir);
LCD_cmd(0xC0);
_delay_ms(1000);
i=0;
while(longi_value[i]!='\0')
{
LCD_write(longi_value[i]);
i++;
}
LCD_write(longi_dir);
_delay_ms(1000);

}
}
}
}
}
}
}
}
}

void init_LCD(void)
{
LCD_cmd(0x38); //initialization of 16X2 LCD in 8bit mode
_delay_ms(1);

LCD_cmd(0x01); //clear LCD
_delay_ms(1);

LCD_cmd(0x0E); //cursor ON
_delay_ms(1);

LCD_cmd(0x80); // ---8 go to first line and --0 is for 0th position
_delay_ms(1);
return;
}


void LCD_cmd(unsigned char cmd)
{
LCD_DATA=cmd;
ctrl =(0<<rs)|(0<<rw)|(1<<en);
_delay_us(40);
ctrl =(0<<rs)|(0<<rw)|(0<<en);
//_delay_ms(50);
return;
}


void LCD_write(unsigned char data)
{
LCD_DATA= data;
ctrl = (1<<rs)|(0<<rw)|(1<<en);
_delay_us(40);
ctrl = (1<<rs)|(0<<rw)|(0<<en);
//_delay_ms(50);
return ;
}


void usart_init()
{

UCSRB |= (1<<RXCIE) | (1 << RXEN) | (1 << TXEN);   // Turn on the transmission and reception circuitry
UCSRC |= (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1); // Use 8-bit character sizes

UBRRL = BAUD_PRESCALE; // Load lower 8-bits of the baud rate value into the low byte of the UBRR register
UBRRH = (BAUD_PRESCALE >> 8); // Load upper 8-bits of the baud rate value into the high byte of the UBRR register
}


unsigned int usart_getch()
{

while ((UCSRA & (1 << RXC)) == 0); // Do nothing until data have been recieved and is ready to be read from UDR
return(UDR); // return the byte
}

void LCD_write_string(unsigned char *str) //take address vaue of the string in pionter *str
{
int i=0;
while(str[i]!='\0') // loop will go on till the NULL charaters is soon in string
{
LCD_write(str[i]); // sending data on CD byte by byte
i++;
}
return;
}
