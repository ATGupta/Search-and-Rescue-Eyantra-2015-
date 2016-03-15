 /*
 Team Id :          eYRCPlus-SR#1068 
 Author List :      Bittu Chauhan, Kavish Bhatia, Arya tanmay gupta
 Filename:          search.cpp
 Theme:             Search and Rescue
 Functions:         servo_1,servo_free,servo_rest,align,rightcount,leftcount,leftanglerotate,rightanglerotate,makeitright,travel,led_config,red_on,blue_on,green_on,led_off,servo1_pin_config,motion_pin_config,left_encoder_pin_config,right_encoder_pin_config,lcd_port_config,adc_pin_config,buzzer_pin_config,led_config
 Global Variables:  val,search,angle1,angle2,colr1,color2,threshold,data,rec,current,via,dest,ShaftCountLeft,ShaftCountRight,Degrees,val[30],index,x,sharp,value,z,Left_white_line,Right_white_line,Center_white_line,ADC_Value
 */ 
 #define F_CPU 14745600						// Clock frequency for the bot operation
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>							//included to support power function
#include "lcd.h"							// lcd functions initialized here
#include "dij.cpp"							//dijkstra implementation
unsigned int threshold=2500;				//threshold for black color(color sensor)

int data,rec,j=0;								//to store received/send data from UDR0
#include "CPPFile1.cpp"						//color sensor code

volatile unsigned long int ShaftCountLeft = 0;			//to keep track of left position encoder
volatile unsigned long int ShaftCountRight = 0;			//to keep track of right position encoder
volatile unsigned int Degrees;							//to accept angle in degrees for turning

unsigned char ADC_Conversion(unsigned char);
unsigned char sharp;									//front sensor value
unsigned int value;
unsigned char ADC_Value;
unsigned int search[]={1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2};     // blocks yet to look for
unsigned int val[][4]={ {68,77,79,88},{68,57,47,59},{66,75,77,86},{66,55,57,45},
						{64,75,73,84},{64,55,53,43},{62,41,51,53},{62,71,73,82},
						{22,31,41,33},{22,11,13,2},{24,33,35,43},{24,13,15,4},
						{26,17,15,6},{26,37,35,45},{28,37,39,47},{28,17,19,8}}	;
unsigned int count=0,box=0,x=0,flag=0;                             
unsigned int via=0,dest=0,color1=0,color2=0,angle1=0,angle2=0,deb=0;
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
//Configure PORTB 6 pin for servo motor 2 operation
void servo2_pin_config (void)
{
	DDRB  = DDRB | 0x40;  //making PORTB 6 pin output
	PORTB = PORTB | 0x40; //setting PORTB 6 pin to logic 1
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

void color_sensor_pin_config(void)
{
	DDRD  = DDRD | 0xFE; //set PD0 as input for color sensor output
	PORTD = PORTD | 0x01;//Enable internal pull-up for PORTD 0 pin
}


void color_sensor_pin_interrupt_init(void) //Interrupt 0 enable
{
	cli(); //Clears the global interrupt
	EICRA = EICRA | 0x02; // INT0 is set to trigger with falling edge
	EIMSK = EIMSK | 0x01; // Enable Interrupt INT0 for color sensor
	sei(); // Enables the global interrupt
}
//ISR for color sensor
ISR(INT0_vect)
{
	pulse++; //increment on receiving pulse from the color sensor
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
	servo2_pin_config();
	motion_pin_config(); //robot motion pins config
	left_encoder_pin_config(); //left encoder pin config
	right_encoder_pin_config(); //right encoder pin config
	lcd_port_config();
	adc_pin_config();
	buzzer_pin_config();
	led_config();
	color_sensor_pin_config();
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

//Function to rotate Servo 2 by a specified angle in the multiples of 1.86 degrees
void servo_2(unsigned char degrees)
{
	float PositionTiltServo = 0;
	PositionTiltServo = ((float)degrees / 1.86) + 35.0;
	OCR1BH = 0x00;
	OCR1BL = (unsigned char) PositionTiltServo;
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
	servo_1(90);
}
/*
Function Name: servo_2_rest
Input :       none
Output :      servo returns to its resting position
Logic:		  -
Example Call: servo_2_rest();
*/
void servo_2_rest (void) //makes servo 1 free rotating
{
	servo_2(0);
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
Function Name: IDR
Input :       none
Output :      none
Logic:		  it moves the debris in clearing zone and idenfies the color of survivor beneath
Example Call: servo_1_rest();
*/
void IDR(void)				//identification routine
{
	stop();
	servo_1(40);
	int x=40;
	for(int j=x;j<=95;j++){
		servo_1(j);
		_delay_ms(50);
		
	}
	if(angle2==0)
	servo_1(85);
	else servo_1(90);
	_delay_ms(100);
	color();
	if(angle2==0)
	color1=data;
	else color2=data;
	if(data==92)			//color of patch is red
	{
		red_on();
		
	}
	else if(data==93)			//color of patch is green
	{
		green_on();
		
	}
	_delay_ms(2000);
	led_off();
	servo_1_rest();
	_delay_ms(2000);
}

//Function used for turning robot by specified degrees
void left_angle_rotate(unsigned int Degrees)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	ShaftCountRight = 0;
	ShaftCountLeft = 0;
	
	while (1)
	{
		//Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		//print_sensor(1,1,2);
		sharp = ADC_Conversion(11);						//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
		value = Sharp_GP2D12_estimation(sharp);				//Stores Distance calculated in a variable "value".
		//lcd_print(1,1,value,3); 						//Prints Value Of Distanc in MM measured by Sharp Sensor.
		if(value>90 && value<150)
		{
			stop();
			angle2=102;
			IDR();
			stop();
			break;
		}
		left(); //Turn right
		velocity(200,200);
		
		if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
		{
			stop();
			blue_on();
			buzzer_on();
			_delay_ms(2000);
			buzzer_off();
			led_off();
			j=j+1;
			break;
		}
		
	}
	stop(); //Stop robot
}
//Function used for turning robot by specified degrees
void right_angle_rotate(unsigned int Degrees)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	ShaftCountRight = 0;
	ShaftCountLeft = 0;
	
	while (1)
	{
		//Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		//print_sensor(1,1,2);
		sharp = ADC_Conversion(11);						//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
		value = Sharp_GP2D12_estimation(sharp);				//Stores Distance calsulated in a variable "value".
		//lcd_print(1,1,value,3); 						//Prints Value Of Distanc in MM measured by Sharp Sensor.
		if(value>90 && value<120)
		{
			angle1=101;
			
			//lcd_print(1,1,data,3);
			
			//UDR0=data;
			IDR();
			stop();
			break;
		}
		right(); //Turn right
		velocity(200,200);
		if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
		{
			stop();
			_delay_ms(1000);
			blue_on();
			buzzer_on();
			_delay_ms(2000);
			buzzer_off();
			led_off();
			j=j+1;
			break;	
		}
		
	}
	stop(); //Stop robot
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
	int flag=1;
	while(1)
	{
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		//print_sensor(1,1,2);
		
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
Function Name: nextdest
Input :       none
Output :      next node to be searched
Logic:		  it marks out the nodes currently searched and finds the next one to look for
Example: dest=nextdest();
*/
int nextdest()
{
	int pos=0;
	for(int i=0;i<17;i++)
	{
		if(search[i]==1)
		{
			pos=i;
			break;
		}	
		if(search[i]==2)
		{
			stop();
			return 49;
			break;
		}
	}
	for(int j=0;j<4;j++)
	{
		if(val[pos][j]!=0)
		{
			return val[pos][j];
		}
	}
	search[pos]=0;
	return 0;
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
	//lcd_print(1,7,a,2);
	//lcd_print(1,10,a+path[0],2);
	if((a+path[0])%4==0)
	{
	}
	else if((a+path[0])%4==2)//it is back
	{
		
		right_degrees(135); //Rotate robot right by 90 degrees
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
	color_sensor_pin_interrupt_init();
	uart0_init(); //Initailize UART0 for serial communiaction
	uart2_init(); //Initailize UART1 for serial communiaction
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	sei();   // Enables the global interrupt
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
	servo_1_rest();
	servo_2_rest();
	rem(2);
	data=250;
	UDR0=data;
	flag=4;
	x=0;
	int etc=0,sdest=0,t=0,destination=0;
	unsigned int current=49;
	for(int k=0;k<100;k++,etc++)
	{
		
		if(etc==0)
		count=0;
		else 
		count=1;
		//lcd_print(1,7,k,2);
		sdest=nextdest();
		destination=sdest;
		lcd_print(2,1,current,2);
		lcd_print(2,5,sdest,2);
		shortest_path(current,sdest);
		makeitright(x);
		x=path[0];
		while(1)
		{
			if(rec>0&& rec<=90)
			{
				
				rem(rec);
				
			}
			
			Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
			Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
			Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
			//print_sensor(1,1,2);
			sharp = ADC_Conversion(11);						//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
			value = Sharp_GP2D12_estimation(sharp);				//Stores Distance calsulated in a variable "value".
			if(value>80 && value<150)
			{
				stop();
				//_delay_ms(1000);
				if(count==tot-1)
				{
					for(int i=0;i<16;i++)
					{
						for(int j=0;j<4;j++)
						{
							if(val[i][j]==sdest)
							val[i][j]=0;
						}
					}
					//lcd_print(1,5,sdest,2);
					rem(sdest);
					data=110;
					lcd_print(1,1,data,3);
					
					UDR0=data;
					_delay_ms(100);
					data=sdest;
					lcd_print(1,1,data,3);
					
					UDR0=data;
					
					current=original[count-1];
					//lcd_print(1,8,original[count-1],2);
				}
				else
				{//lcd_print(1,5,original[count],2);
					rem(original[count]);
					data=110;
					lcd_print(1,1,data,3);
				
					UDR0=data;
					_delay_ms(100);
					data=original[count];
					lcd_print(1,1,data,3);
					UDR0=data;
				current=original[count-1];
				//lcd_print(1,8,original[count-1],2);
				}				
				break;
			}
			if(Center_white_line<0x15 && Left_white_line<0x15 && Right_white_line<0x15)
			{
				if (flag==1)
				{
					forward();
					velocity(250,250);
				}
				if (flag==2)
				{
					forward();
					velocity(170,250);
				}
				if (flag==3)
				{
					forward();
					velocity(250,170);
				}
				if(flag==4)
				{
					forward();
					velocity(250,250);
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
				
				lcd_print(2,9,original[count],3);
				if(path[count]==1)//it is left
				{
					x+=path[count];
					forward_mm(75); //Moves robot forward 100mm
					left_degrees(45); //Rotate robot left by 90 degrees
					left_count();
					stop();
					
					//j++;
				}
				else if(path[count]==4)//it is straight
				{
					x+=path[count];
					//j++;
					forward_mm(45); //Moves robot forward 100mm
					//align();
				}
				else if(path[count]==3)//it is right
				{
					x+=path[count];
					forward_mm(75); //Moves robot forward 100mm
					right_degrees(45); //Rotate robot right by 90 degrees
					right_count();
					stop();
					//_delay_ms(1000);
				}
				else if(path[count]==2)//it is back
				{
					x+=path[count];
					forward_mm(75); //Moves robot forward 100mm
					right_degrees(130); //Rotate robot right by 90 degrees
					right_count();
					//align();
				}
				else if(count==tot-1)//it is right
				{
					//lcd_print(1,11,x,2);
					for(int i=0;i<16;i++)
					{
						for(int j=0;j<4;j++)
						{
							if(val[i][j]==sdest)
							search[i]=0;
						}
					}
					
					
					stop();
					forward_mm(75); //Moves robot forward 100mm
					
					if(k>1 && j!=2)
					{
						data=150;
						lcd_print(1,1,data,3);
						UDR0=data;
						_delay_ms(1000);
						UDR0=via;
						
						//lcd_print(1,1,via,3);
						_delay_ms(100);
						UDR0=dest;
						//lcd_print(1,1,dest,3);
						_delay_ms(100);
						UDR0=angle1;
						
						_delay_ms(100);
						UDR0=color1;
						_delay_ms(100);
						UDR0=angle2;
						_delay_ms(100);
						UDR0=color2;
						_delay_ms(100);
						UDR0=sdest;
						_delay_ms(100);
						data=200;
						UDR0=data;
						_delay_ms(1000);
						while(rec!=200)
						{
							stop();
							_delay_ms(10);
							lcd_print(1,1,t,3);
							t=t+1;
						}
						data=250;
						UDR0=data;
						_delay_ms(100);
					}
					if(sdest==49)
					{
						stop();
						data=149;
						UDR0=data;
						_delay_ms(10000);
						return 0;
					}
					
					via=dest=color1=color2=angle1=angle2=deb=j=0;
					via=original[count-1];
					dest=sdest;
					
					servo_1(0);
					servo_2(90);
					//box=0;
					//_delay_ms(3000);
					right_angle_rotate(110);
					stop();
					servo_1(0);
					left_degrees(50);
					stop();
					left_angle_rotate(180);
					stop();
					
					servo_1(0);
					right_degrees(30);
					servo_2_rest();
					_delay_ms(1000);
					servo_1_rest();
					
					right_count();
					stop();
					current=destination;
					//makeitright(x);
					break;
					
				}
				count++;
			}
			//break;					
		}
		
		
	}
	

}