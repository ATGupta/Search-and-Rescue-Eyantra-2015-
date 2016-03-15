 /*
 Team Id :          eYRCPlus-SR#1068 
 Author List :      Bittu Chauhan, Kavish Bhatia, Arya tanmay gupta
 Filename:          Rescue.c
 Theme:             Search and Rescue
 Functions:         servo_1,servo_free,servo_rest,align,rightcount,leftcount,leftanglerotate,rightanglerotate,makeitright,travel,led_config,red_on,blue_on,green_on,led_off,servo1_pin_config,motion_pin_config,left_encoder_pin_config,right_encoder_pin_config,lcd_port_config,adc_pin_config,buzzer_pin_config,led_config
 Global Variables:  data,rec,current,via,dest,ShaftCountLeft,ShaftCountRight,Degrees,val[30],index,x,sharp,value,z,Left_white_line,Right_white_line,Center_white_line,ADC_Value
 */ 
#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h> //included to support power function
#include "lcd.h"
#include "dij.cpp"
int data,rec;
int current=90,via,dest;
volatile unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder
volatile unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
volatile unsigned int Degrees,val[30]={0},index=0,x=0; //to accept angle in degrees for turning

unsigned char ADC_Conversion(unsigned char);
unsigned char sharp;
unsigned int value,z=0;
unsigned char ADC_Value;
unsigned char flag = 0;
unsigned char Left_white_line = 0;
unsigned char Center_white_line = 0;
unsigned char Right_white_line = 0;

/*
Function Name: SIGNAL
Input :       SIG_USART0_RECV(interrupt)
Output :      received signal from other bot
Logic:       -
Example Call:   -
*/
SIGNAL(SIG_USART0_RECV) 		// ISR for receive complete interrupt
{
	rec = UDR0; 				//making copy of data from UDR0 in 'rec' variable
	if(rec==149)
	{
		z=149;
		rec=200;
	}
}


//Function To Initialize UART0
// desired baud rate:9600
// actual baud rate:9600 (error 0.0%)
// char size: 8 bit
// parity: Disabled
void uart0_init(void)
{
	UCSR0B = 0x00; //disable while setting baud rate
	UCSR0A = 0x00;
	UCSR0C = 0x06;
	UBRR0L = 0x5F; //set baud rate lo
	UBRR0H = 0x00; //set baud rate hi
	UCSR0B = 0x98;
}

//Function To Initialize UART1
// desired baud rate:9600
// actual baud rate:9600 (error 0.0%)
// char size: 8 bit
// parity: Disabled
void uart2_init(void)
{
	UCSR2B = 0x00; //disable while setting baud rate
	UCSR2A = 0x00;
	UCSR2C = 0x06;
	UBRR2L = 0x5F; //set baud rate lo
	UBRR2H = 0x00; //set baud rate hi
	UCSR2B = 0x98;
}
void led_config(void)  //configuration setting for led on port H 5,6,7
{
	
	DDRH=0b11100000;
	PORTH=0x00;
}

/*
Function Name: red_on
Input :       none
Output :      set bit high for red pin
Logic:       -
Example Call:   red_on();
*/
void red_on(void)    //for red led
{
	PORTH=0b10000000;
}
/*
Function Name: led_off
Input :       none
Output :      set bit low for all pin
Logic:       -
Example Call:   led_off();
*/
void led_off(void)			//for led off
{
	PORTH=0x00;
}
/*
Function Name: blue_on
Input :       none
Output :      set bit high for blue pin
Logic:       -
Example Call:   blue_on();
*/
void blue_on(void)		//for blue led
{
	PORTH=0b00100000;
}
/*
Function Name: green_on
Input :       none
Output :      set bit high for green pin
Logic:       -
Example Call:   green_on();
*/
void green_on(void)		//for green led
{
	PORTH=0b01000000;
}
//Function to configure ports to enable robot's motion
void motion_pin_config (void)
{
	DDRA = DDRA | 0x0F;
	PORTA = PORTA & 0xF0;
	DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
	PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}
//Function to configure LCD port
void lcd_port_config (void)
{
	DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
	PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

//ADC pin configuration
void adc_pin_config (void)
{
	DDRF = 0x00;
	PORTF = 0x00;
	DDRK = 0x00;
	PORTK = 0x00;
}

//Function to configure INT4 (PORTE 4) pin as input for the left position encoder
void left_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}

//Function to configure INT5 (PORTE 5) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}
void buzzer_pin_config (void)
{
	DDRC = DDRC | 0x08;		//Setting PORTC 3 as output
	PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}


//Configure PORTB 5 pin for servo motor 1 operation
void servo1_pin_config (void)
{
	DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
	PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}
//Function to initialize ports
void port_init()
{
	servo1_pin_config();
	motion_pin_config(); //robot motion pins config
	left_encoder_pin_config(); //left encoder pin config
	right_encoder_pin_config(); //right encoder pin config
	lcd_port_config();
	adc_pin_config();
	buzzer_pin_config();
	led_config();
	//color_sensor_pin_config();
}

// Timer 5 initialized in PWM mode for velocity control
// Prescale:256
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:225.000Hz
void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

//Function For ADC Conversion
unsigned char ADC_Conversion(unsigned char Ch)
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;
	ADMUX= 0x20| Ch;
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}

//Function To Print Sesor Values At Desired Row And Coloumn Location on LCD
void print_sensor(char row, char coloumn,unsigned char channel)
{
	
	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
}

void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
	EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
	sei();   // Enables the global interrupt
}

void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
	EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
	sei();   // Enables the global interrupt
}

//ISR for right position encoder
ISR(INT5_vect)
{
	ShaftCountRight++;  //increment right shaft position count
}

//ISR for left position encoder
ISR(INT4_vect)
{
	ShaftCountLeft++;  //increment left shaft position count
}
//Function for velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
	unsigned char PortARestore = 0;

	Direction &= 0x0F; 		// removing upper nibbel for the protection
	PortARestore = PORTA; 		// reading the PORTA original status
	PortARestore &= 0xF0; 		// making lower direction nibbel to 0
	PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
	PORTA = PortARestore; 		// executing the command
}
void buzzer_on (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore | 0x08;
	PORTC = port_restore;
}

void buzzer_off (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore & 0xF7;
	PORTC = port_restore;
}

void forward (void) //both wheels forward
{
	motion_set(0x06);
}

void back (void) //both wheels backward
{
	motion_set(0x09);
}

void left (void) //Left wheel backward, Right wheel forward
{
	motion_set(0x05);
}

void right (void) //Left wheel forward, Right wheel backward
{
	motion_set(0x0A);
}

void soft_left (void) //Left wheel stationary, Right wheel forward
{
	motion_set(0x04);
}

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
	motion_set(0x02);
}

void soft_left_2 (void) //Left wheel backward, right wheel stationary
{
	motion_set(0x01);
}

void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
	motion_set(0x08);
}

void stop (void)
{
	motion_set(0x00);
}


unsigned int Sharp_GP2D12_estimation(unsigned char adc_reading)
{
	float distance;
	unsigned int distanceInt;
	distance = (int)(10.00*(2799.6*(1.00/(pow(adc_reading,1.1546)))));
	distanceInt = (int)distance;
	if(distanceInt>800)
	{
		distanceInt=800;
	}
	return distanceInt;
}
//Function used for turning robot by specified degrees
void angle_rotate(unsigned int Degrees)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	ShaftCountRight = 0;
	ShaftCountLeft = 0;

	while (1)
	{
		if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
		break;
	}
	stop(); //Stop robot
}

//Function used for moving robot forward by specified distance

void linear_distance_mm(unsigned int DistanceInMM)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
	
	ShaftCountRight = 0;
	while(1)
	{
		if(ShaftCountRight > ReqdShaftCountInt)
		{
			break;
		}
	}
	stop(); //Stop robot
}

void forward_mm(unsigned int DistanceInMM)
{
	forward();
	linear_distance_mm(DistanceInMM);
}

void back_mm(unsigned int DistanceInMM)
{
	back();
	linear_distance_mm(DistanceInMM);
}

void left_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	left(); //Turn left
	angle_rotate(Degrees);
}



void right_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	right(); //Turn right
	angle_rotate(Degrees);
}


void soft_left_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_left(); //Turn soft left
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void soft_right_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_right();  //Turn soft right
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void soft_left_2_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_left_2(); //Turn reverse soft left
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void soft_right_2_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_right_2();  //Turn reverse soft right
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}
/*
Function Name: left_count
Input :       none
Output :      bot turn left until it reaches a black line
Logic:		  bot looks for black line while turning left and stops as soon as it indentifies the black line
Example Call: left_count();
*/
void left_count()
{
	while(1)
	{
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		left(); //Turn left
		velocity(180,180);
		if(Center_white_line>0x15)
		return;
	}
	
}
//TIMER1 initialization in 10 bit fast PWM mode  
//prescale:256
// WGM: 7) PWM 10bit fast, TOP=0x03FF
// actual value: 52.25Hz 
void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
 TCNT1L = 0x01;	//Counter low value to which OCR1xH value is to be compared with
 OCR1AH = 0x03;	//Output compare Register high value for servo 1
 OCR1AL = 0xFF;	//Output Compare Register low Value For servo 1
 OCR1BH = 0x03;	//Output compare Register high value for servo 2
 OCR1BL = 0xFF;	//Output Compare Register low Value For servo 2
 OCR1CH = 0x03;	//Output compare Register high value for servo 3
 OCR1CL = 0xFF;	//Output Compare Register low Value For servo 3
 ICR1H  = 0x03;	
 ICR1L  = 0xFF;
 TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 					For Overriding normal port functionality to OCRnA outputs.
				  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR1C = 0x00;
 TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}


//Function to rotate Servo 1 by a specified angle in the multiples of 1.86 degrees
void servo_1(unsigned char degrees)
{
	float PositionPanServo = 0;
	PositionPanServo = ((float)degrees / 1.86) + 35.0;
	OCR1AH = 0x00;
	OCR1AL = (unsigned char) PositionPanServo;
}
/*
Function Name: servo_1_rest
Input :       none
Output :      servo returns to its resting position
Logic:		  -
Example Call: servo_1_rest();
*/
void servo_1_rest (void) //makes servo 1 free rotating
{
	servo_1(0);
}
//servo_free functions unlocks the servo motors from the any angle
//and make them free by giving 100% duty cycle at the PWM. This function can be used to
//reduce the power consumption of the motor if it is holding load against the gravity.

void servo_1_free (void) //makes servo 1 free rotating
{
	OCR1AH = 0x03;
	OCR1AL = 0xFF; //Servo 1 off
}
/*
Function Name: right_count
Input :       none
Output :      bot turn right until it reaches a black line
Logic:		  bot looks for black line while turning right and stops as soon as it indentifies the black line
Example Call: right_count();
*/

void right_count()
{
	while(1)
	{
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		right(); //Turn right
		velocity(180,180);
		if(Center_white_line>0x15)
		return;
	}
	
}
/*
Function Name: align
Input :       none
Output :      bot aligns itself with black line
Logic:		  bot looks for black line while turning right and stops as soon as it identifies the black line and do this for other direction as well
Example Call: align();
*/
int align()
{
	velocity(180,180);
	right_degrees(15);
	left_count();
	velocity(180,180);
	left_degrees(15);
	right_count();
	
	
}
/*
Function Name: makeitright
Input :       x(the sum of directions bot took while traversing the last dijkstra's call )
Output :      bot faces in the direction it need to travel in the next dijkstra's run
Logic:		  since there are four direction bot can face so sum of all previous turn tells which direction bot is facing now and accordingly turns itself in the next deisred direction
Example Call: makeitright(sum);
*/
void makeitright(int x)
{
	int a=4-(x%4);
	if((a+path[0])%4==0)
	{
		_delay_ms(100);
		align();
	}
	else if((a+path[0])%4==2)//it is back
	{
		
		right_degrees(150); //Rotate robot right by 90 degrees
		right_count(); //Rotate robot right by 90 degrees
		_delay_ms(100);
		align();
	}
	else if((a+path[0])%4==3)//it is right
	{
		right_degrees(45); //Rotate robot right by 90 degrees
		right_count(); //Rotate robot right by 90 degrees
		_delay_ms(100);
		align();
	}
	if((a+path[0])%4==1)//it is left
	{
		left_degrees(45); //Rotate robot left by 90 degrees
		left_count(); //Rotate robot right by 90 degrees
		_delay_ms(100);
		align();
	}
	
}


//Function to initialize all the devices
void init_devices()
{
	cli(); //Clears the global interrupt
	port_init();  //Initializes all the ports
	adc_init();
	timer5_init();
	timer1_init();
	//color_sensor_pin_interrupt_init();
	uart0_init(); //Initailize UART0 for serial communiaction
	uart2_init(); //Initailize UART1 for serial communiaction
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	sei();   // Enables the global interrupt
}
/*
Function Name: travel
Input :       s(source),d(destination)
Output :      1(successful traversing), current node number(where the path was obstructed by debris)
Logic:		  bot finds the path to destination by dijkstra algo and decide the turns it need to take while traversing arena
Example Call: travel(49,85);
*/
int travel(int s,int d)
{
	int count=0,current,j=0,flag;
	shortest_path(s,d);
	makeitright(x);
	if(x!=0)
	{
		count++;
	}
	x=path[0];
	if(s==90)
	{
		path[0]=4;
		count=1;
	}
	
	while(1)
	{
		
		
		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
		sharp = ADC_Conversion(11);						//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
		value = Sharp_GP2D12_estimation(sharp);				//Stores Distance calsulated in a variable "value".
		if(value>80 && value<130)
		{
			stop();
			_delay_ms(1000);
			//lcd_print(1,5,original[count],2);
			rem(original[count]);
			data=110;
			//lcd_print(1,1,data,3);
			UDR0=data;
			_delay_ms(100);
			data=original[count];
			//lcd_print(1,1,data,3);
			UDR0=data;
			current=original[count-1];
			//lcd_print(1,8,original[count-1],2);
			return current;
		}
		if(Center_white_line<0x15 && Left_white_line<0x15 && Right_white_line<0x15)// all white
		{
			if (flag==1)
			{
				forward();
				velocity(200,200);
			}
			if (flag==2)
			{
				forward();
				velocity(140,200);
			}
			if (flag==3)
			{
				forward();
				velocity(200,140);
			}
			if(flag==4)
			{
				forward();
				velocity(200,200);
			}
			
		}
		else if(Center_white_line>0x15 && Left_white_line<0x15 && Right_white_line<0x15)//center sensed color is black
		{
			forward();
			velocity(250,250);
			flag=1;
		}
		else if(Center_white_line<0x15 && Left_white_line>0x15 && Right_white_line<0x15)//left sensed color is black
		{
			flag=2;
			forward();
			velocity(170,250);
		}
		else if(Center_white_line<0x15 && Left_white_line<0x15 && Right_white_line>0x15)//right sensed color is black
		{
			flag=3;
			forward();
			velocity(250,170);
		}
		else if((Center_white_line>0x15 && Left_white_line>0x15 && Right_white_line>0x15)||(Center_white_line>0x15 && Left_white_line>0x15 && Right_white_line<0x15)||(Center_white_line>0x15 && Left_white_line<0x15 && Right_white_line>0x15))
		{
			flag=4;
			
			lcd_print(2,9,original[count],2);
			if(path[count]==1)//it is left
			{
				x+=path[count];
				forward_mm(75); //Moves robot forward 100mm
				left_degrees(45); //Rotate robot left by 90 degrees
				left_count();
				stop();
				_delay_ms(100);
				
				//align();
				//j++;
			}
			else if(path[count]==4)//it is straight
			{
				x+=path[count];
				//j++;
				forward_mm(45); //Moves robot forward 100mm
				
			}
			else if(path[count]==3)//it is right
			{
				x+=path[count];
				forward_mm(75); //Moves robot forward 100mm
				right_degrees(45); //Rotate robot right by 90 degrees
				right_count();
				stop();
				_delay_ms(100);
				//_delay_ms(100);
				//align();
			}
			else if(path[count]==2)//it is back
			{
				x+=path[count];
				forward_mm(75); //Moves robot forward 100mm
				right_degrees(150); //Rotate robot right by 90 degrees
				right_count();
				_delay_ms(100);
				//align();
			}
			else if(count==tot-1)//it is right
			{
				return 1;
			}
			count++;
		}
		//return 1;
	}
}

/*
Function Name: main
Input :       none
Output :      none
Logic:		  driver function to provide necessary medical services and call helper function appropriately
Example Call: -
*/
int main(void)
{
	init_devices();
	lcd_set_4bit();
	lcd_init();
	servo_1_rest();			//resetting servo 1 to rest position
	//_delay_ms(1000);
	led_off();
	unsigned int x=0,j=1,i=0,a[2],cur=45;
	data=200;
	UDR0=data;				//sending flag 200 to search bot in start so that search can start it task while rescue waits for it find survivors
	_delay_ms(100);
	index=0;
	j=0;
	while(1)
	{
		if(rec==110)        //flag 110 is to make bot understands the next node it is going to get is blocked hence can be removed from arena
		{
			while(rec==110)
			{
				_delay_ms(10);
			}
			if(rec<=90)
			{
				rem(rec);
			}
		}
		if(rec==150)        //flag 150 to understand that search is about to send the info of the survivors it identified in its run of the arena
		{
			data=250;					//search must stop now as rescue is about to start
			UDR0=data;
			_delay_ms(1000);
			while(rec==150){
				_delay_ms(10);
			}
			via=rec;                    // via is the pre-destination to orient the rescue bot in the right direction to provide medical procedure to the survivors 
			while(rec==via){
				_delay_ms(10);
			}
			dest=rec;					// destination ( location )of the survivors
			while(rec==dest){
				_delay_ms(10);
			}
			val[1]=rec;					// turn bot need to take to provide medical procedure
			while(rec==val[1]){
				_delay_ms(10);
			}
			val[2]=rec;					// type of first survivor if any
			while(rec==val[2]){
				_delay_ms(10);
			}
			val[3]=rec;					// turn bot need to take to provide medical procedure
			while(rec==val[3]){
				_delay_ms(10);
			}
			val[4]=rec;					// type of second survivor
			while(rec==val[4]){
				_delay_ms(10);
			}
			cur=rec;					// place where search is standing for rescue to avoid that node
		}			
		if(rec==200) //start traversing
		{
			data=250;                 //search need to stop now
			UDR0=data;
			_delay_ms(1000);
			rem1(cur);					// temporary node to remove where search bot is resting
			for(int i=1;i<=4;i++)
			{
				if(val[i]==92)			//red procedure begins
				{
					red_on();
					//lcd_string("RED");
					while(current!=1)
					{
						current=travel(current,via);
					}
					current=via;
					forward_mm(100);
					_delay_ms(1000);
					
					while(current!=1)
					{
						
						current=travel(current,dest);
					}
					forward_mm(100);
					_delay_ms(1000);
					current=dest;
					if(val[i-1]==101)
					{
					right_degrees(90);
					_delay_ms(1000);
					buzzer_on();
					_delay_ms(1000);
					buzzer_off();
					
					left_count();
					stop();
					}						
					else {
					left_degrees(90);
					_delay_ms(1000);
					buzzer_on();
					_delay_ms(1000);
					buzzer_off();
					right_count();
					stop();
					}					
					while(current!=1)
					{
						current=travel(current,86);
					}
					current=86;
					forward_mm(75);
					_delay_ms(1000);
					while(current!=1)
					{
						current=travel(current,85);
					}
					current=85;
					forward_mm(75);
					_delay_ms(1000);
					right_degrees(90);
					_delay_ms(1000);
					stop();
					
					buzzer_on();
					_delay_ms(1000);
					buzzer_off();
					left_degrees(45);
					//_delay_ms(1000);
					left_count();
					stop();
					led_off();
					val[i]=0;
					val[i-1]=0;
					i=0;
				}
				
			}
			
			//red procedure ends
			for(int i=1;i<=4;i++)
			{
				//lcd_print(2,12,i,3);
				if(val[i]==93)  //green procedure starts
				{
					green_on();
					while(current!=1)
					{
					
						current=travel(current,via);
					}
					current=via;
					forward_mm(100);
					_delay_ms(1000);
				
					while(current!=1)
					{
					
						current=travel(current,dest);
					}
					current=dest;
					forward_mm(100);
					_delay_ms(1000);
				
					if(val[i-1]==101)
					{
						right_degrees(90);
						_delay_ms(1000);
						servo_1(75);
						_delay_ms(1000);
						servo_1_rest();
						buzzer_on();
						_delay_ms(1000);
						buzzer_off();
						left_count();
						stop();
					}
					else {
						left_degrees(90);
						_delay_ms(1000);
						
						servo_1(75);
						_delay_ms(1000);
						servo_1_rest();
						buzzer_on();
						_delay_ms(1000);
						buzzer_off();
						right_count();
						stop();
					}
					led_off();
					val[i]=0;
					val[i-1]=0;
					i=0;
				}
			}
			//green procedure ends
			add(cur);
			//rec=375;
			data=200;     //search can start now
			UDR0=data;
			_delay_ms(3000);
			
			if(z==149)			//every survivor is served 
			{
				buzzer_on();
				_delay_ms(10000);
				buzzer_off();
				return 0;
			}
			
		}
		
	}
	
}
