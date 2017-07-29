#include<AT89X52.h>
#define lcdport P0
sbit rs=P2^0;		  
sbit rdwr=P2^1;
sbit lcde=P2^2;
sbit bombsensor=P1^0;
sbit forwardsensor=P1^1;
sbit leftsensor=P1^2;
sbit rightsensor=P1^3;
sbit backsensor=P1^4;
sbit buzzer=P3^5;
sbit relay1=P3^6;
sbit relay2=P3^7;
bit bombsenseflag;
bit fwdsenseflag;
bit leftsenseflag;
bit rightsenseflag;
bit backsenseflag;
unsigned char buzzercount;
void delay();
void lcdinit();
void clr_lcd();
void dispslogan(char*);
void senddata(unsigned char);
void send_command(unsigned char);
void next_line();
void delay1(); 
const unsigned char slogan1[]="PATH FINDER       ";
const unsigned char slogan2[]="     WELCOME      ";

void timer0() interrupt 1
{
buzzercount++;
TF0=0;
TH0=0x4b;
TL0=0x0fd;
if(buzzercount==20)
{
buzzercount=0;
buzzer=1;
ET0=0;
TR0=0;
}}

/****************FUNCTION FOR SWAPPING LSBYTE AND MSBYTE OF THE DATA***************/

  unsigned char xch(unsigned char data1)
{
unsigned char temp,temp1;
temp=data1;
data1=data1>>4;
temp1=data1;
data1=temp;
data1=data1<<4;
data1=data1|temp1;
return(data1);
}
/********************** INITIALIZATION   OF   LCD  ***********************************/
 void lcdinit()
 {
                clr_lcd();             /*FUNCTION SET */
                send_command(0x28);
                delay();
                send_command(0x28);
                delay();
                send_command(0x28);
                delay();
                send_command(0x06);			//ENTRY MODE
                delay();

                send_command(0x0e);			   //DISPLAY ON/OFF
                delay();

                clr_lcd();
				   
} 
                  
/* FUNCTION  FOR  DISPLAYING  DATA  ON  THE   LCD *************************************/

void dispslogan(unsigned char *p)
{
   unsigned char data1;
   while(*p)
	{ 
	data1=*p;   
	senddata(data1);
	p++;
	}
}

/***************** FUNCTION FOR SENDING LCD COMMANDS***********************************/
 
 void send_command(unsigned char data1)
   {		  
        unsigned char newdata;
        rs=0;
        delay();              
        lcde=1;
         delay();       
         lcdport=data1;
		 lcde=0;
         delay();
         lcde=1;
         newdata=xch(data1);
         lcdport=newdata;
         delay();
         lcde=0;
         delay();
         rs=1; 
                
 }

/************************** FUNCTION FOR WRITING DATA ON THE LCD***********************/

void senddata(unsigned char data1)
{
        unsigned char newdata;
        rs=1;
        delay();              
        lcde=1;
         delay();       
         lcdport=data1;
		  lcde=0;
          delay();
          lcde=1;
          newdata=xch(data1);
          lcdport=newdata;
          delay();
          lcde=0;
          delay();
          rs=0; 
}
                






/*********** COMMAND FOR BRINGING LCD CURSOR ON SECOND LINE ***************************/  

void next_line()
{
send_command(0xc0);
delay();
}

/**************COMMAND FOR CLEARING LCD AND BRINGING LCD CURSOR ON FIRST LINE********/

void clr_lcd()
{          
send_command(0x01);
delay();
send_command(0x02);
delay();
}	 



    



void delay()
{
unsigned char i,j;
for(i=0;i<80;i++)
{
for(j=0;j<120;j++)
{}
}}	

void bombcheck()
{
if(bombsensor==0)
bombsenseflag=1;
}
void fwdsensorchk()
{
if(forwardsensor==0)
fwdsenseflag=1;
}
void leftsensorchk()
{
if(leftsensor==0)
leftsenseflag=1;
}
void rightsensorchk()
{
if(rightsensor==0)
rightsenseflag=1;
}
void backsensorchk()
{
if(backsensor==0)
backsenseflag=1;
}


/**********STARTING OF THE PATHFINDER PROJECT********************************/

void main()
{ 
TMOD=0x01;
TF0=0;
TH0=0x4b;
TL0=0x0fd;
EA=1;
ET0=0;
TR0=0;
delay();
lcdinit();
clr_lcd();
dispslogan(slogan1);
next_line();
dispslogan(slogan2);
/************STARTING OF THE MAIN LOOP**********************************************/

while(1)
{
bombcheck();
delay();
fwdsensorchk();
delay();
leftsensorchk();
delay();
rightsensorchk();
delay();
backsensorchk();
delay();

//*****************************************************************************************

if(fwdsenseflag==1)
{
relay1=1;
relay2=1;
}

if(fwdsenseflag==1 && leftsenseflag==1 && rightsenseflag==1)
{
relay1=0;
relay2=0;
}

if(bombsenseflag)
{
clr_lcd();
dispslogan("bomb detected");
buzzer=0;
ET0=1;
TR0=1;
}

//**********************************************************************************

if(fwdsenseflag)
{
if(leftsenseflag==1)
{
clr_lcd();
dispslogan("fwd-obstacle");
next_line();
dispslogan("left-obstacle");
relay1=1;
relay2=0;
}
if(rightsenseflag==1)
{
clr_lcd();
dispslogan("fwd-obstacle");
next_line();
dispslogan("right-obstacle");
relay1=0;
relay2=1;
}}
if(backsenseflag==1)
{
clr_lcd();
dispslogan("back-obstacle");
}

}}









    