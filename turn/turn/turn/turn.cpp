#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#define threshold 1000
unsigned int len,j=0,i=0;
unsigned int current=50,count=0,redcount=0,greencount=0,bluecount=0;
int turncount=0;
unsigned int destred[]={33,13,22};
unsigned int destblue[]={73,93,82};
unsigned int destgreen[]={53,41,54,61};

unsigned int pickup[12]={30,20,10,70,80,90,99,86,79,39,26,19};
void travel();
void align_north();
#include <math.h> //included to support power function
#include "lcd.h"
#include "dij.cpp"
#include "arm.h"
#include "CPPFile1.cpp"

#define blackthreshold 0xA
void port_init();
void timer5_init();
void velocity(unsigned char, unsigned char);
void motors_delay();
volatile unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder
volatile unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
	
unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char flag = 0;
unsigned char Left_white_line = 0;
unsigned char Center_white_line = 0;
unsigned char Right_white_line = 0;
unsigned char front_distance = 0;

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

//Function to configure ports to enable robot's motion
void motion_pin_config (void) 
{
 DDRA = DDRA | 0x0F;
 PORTA = PORTA & 0xF0;
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
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

//Function to Initialize PORTS
void port_init()
{
	lcd_port_config();
	adc_pin_config();
	color_sensor_pin_config();
	motion_pin_config();
	servo1_pin_config(); //Configure PORTB 5 pin for servo motor 1 operation
	servo2_pin_config(); //Configure PORTB 6 pin for servo motor 2 operation
	servo3_pin_config(); //Configure PORTB 7 pin for servo motor 3 operation
	left_encoder_pin_config(); //left encoder pin config
	right_encoder_pin_config(); //right encoder pin config	
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

//Function To Print Sesor Values At Desired Row And Coloumn Location on LCD
void print_sensor(char row, char coloumn,unsigned char channel)
{
	
	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
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

void forward (void) 
{
  motion_set (0x06);
}

void stop (void)
{
  motion_set (0x00);
}

void left (void) //Left wheel backward, Right wheel forward
{
	motion_set(0x05);
}
void right (void) //Left wheel forward, Right wheel backward
{
	motion_set(0x0A);
}
void back (void) //both wheels backward
{
	motion_set(0x09);
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

void left_count()
{
	while(1)
	{
		
	
	
	Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
	//Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor

	left(); //Turn left
	velocity(180,180);
	if(Center_white_line>blackthreshold)
	return;
	}	
	
}
void right_count()
{
	while(1)
	{
		
		
		
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		//Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor

		right(); //Turn right
		velocity(180,180);
		if(Center_white_line>blackthreshold)
		return;
	}
	
}


void init_devices (void)
{
 	cli(); //Clears the global interrupts
	port_init();
	adc_init();
	timer5_init();
	timer1_init();
	servo_init();
	color_sensor_pin_interrupt_init();
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	sei();   //Enables the global interrupts
}
// This Function calculates the actual distance in millimeters(mm) from the input
// analog value of Sharp Sensor.
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
void travel()
{
	
	while(1)
	{
		
		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
		front_distance=ADC_Conversion(11);
		front_distance=Sharp_GP2D12_estimation(front_distance);		
		lcd_print(1,1,front_distance,3);
		lcd_print(2,5,j,3);
		lcd_print(2,1,Center_white_line,3);
		lcd_print(2,9,tot,3);
		if(j==tot)
		{
			forward_mm(40);
			reset();
			back_mm(40);
			return;
		}
		else if(j==tot-1)
		{if((front_distance>70)&&(front_distance<90))
		{
			forward_mm(40);
			return;
		}
		}		
		 if(((Center_white_line>blackthreshold)&&(Left_white_line>blackthreshold)) ||((Center_white_line>blackthreshold) && (Right_white_line>blackthreshold)))
		{

			//velocity(0,0);
			forward_mm(65); //Moves robot forward 100mm
			stop();
			_delay_ms(500);
			if(path[j]==1)//it is left
			{
				left_degrees(45); //Rotate robot left by 90 degrees
				stop();
				_delay_ms(500);
				left_count();
				stop();
				_delay_ms(500);
				//j++;
			}

			else if(path[j]==2)//it is straight
			{
				//j++;
			}

			else if(path[j]==3)//it is right
			{
				right_degrees(45); //Rotate robot right by 90 degrees
				stop();
				_delay_ms(500);
				right_count();
				stop();
				_delay_ms(500);
				
			}
			else if(path[j]==4)//it is 180 turn
			{
				right_degrees(145); //Rotate robot right by 90 degrees
				stop();
				_delay_ms(500);
				right_count();
				stop();
				_delay_ms(500);
				
			}
			j++;

		}

		if((Center_white_line>blackthreshold)&&(Left_white_line<blackthreshold)&&(Right_white_line<blackthreshold))//center line black
		{
			flag=2;
			forward();
			velocity(150,150);
		}

		if((Right_white_line>blackthreshold) && (Center_white_line<blackthreshold)&&(Left_white_line<blackthreshold))// right line black
		{
			flag=3;
			forward();
			velocity(150,100);
		}
		if((Right_white_line<blackthreshold) &&( Center_white_line<blackthreshold) && (Left_white_line>blackthreshold))//left line black
		{
			flag=4;
			forward();
			velocity(100,150);
		}

		if((Center_white_line<blackthreshold )&& (Left_white_line<blackthreshold) && (Right_white_line<blackthreshold))//all white
		{
			if(flag==1)
			{
				forward();
				velocity(0,0);
				break;
			}
			if(flag==2)//correct ittt
			{
				forward();
				velocity(150,150);
			}
			if(flag==4)
			{
				forward();
				velocity(100,150);
			}
			if(flag==3)
			{
				forward();
				velocity(150,100);
			}

		}

	}	
}
void align_north()
{
	turncount=0;	
	for(;i<tot-1;i++)
	{
		if (path[i]==4)
		{ 
			turncount+=2;
		}
		if(path[i]==3)
		{
			turncount++;	  
		}      
		if(path[i]==1) 	
		{
			turncount--;
		}
	}
	int angle=turncount*90;
	if(turncount>0)
	{
		left_degrees(angle);
	
	}
	else
	{
		right_degrees(-angle);
	}
	back_mm(120);
}
//Main Function
int main()
{
	
	init_devices();
	lcd_set_4bit();
	lcd_init();
	reset();
	_delay_ms(2000);
	servo_1_free();
	servo_2_free();	
	while(count<12)
	{
		i=j=0;
		if(count>0)
		align_north();
		shortest_path(current,pickup[count]);
		lcd_string("NEXT NODE");
		lcd_print(1,1,pickup[count],4);
		_delay_ms(1000);
		current=pickup[count++];
		travel();
		i=j=1;
		color();
		//reset();
		
		
	}
	
}
