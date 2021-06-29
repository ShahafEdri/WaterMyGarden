//
// Smpl_ADC_VR1 : use ADC7 to read Variable Resistor (on-board)
//
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "NUC1xx.h"
#include "DrvSYS.h"
#include "Seven_Segment.h"
#include "DrvGPIO.h"
#include "DrvADC.h"
#include "Driver\DrvUART.h"
#include "NUC1xx-LB_002\LCD_Driver.h"
#include "Driver_PWM_Servo.h"

#define SHADE_PWM_ON 240 // 2.4ms
#define SHADE_PWM_OFF 50  // 0.5ms

#define HOUR 10 // TODO - change back to 3600
#define SEC 1000000
#define mSEC 1000

#define  INIT_PUMP_STATUS DrvGPIO_Open(E_GPE, 15, E_IO_OPENDRAIN)
#define  INIT_HUMIDITY_INNER_STATUS DrvGPIO_Open(E_GPC, 13, E_IO_OUTPUT)
#define  INIT_HUMIDITY_OUTER_STATUS DrvGPIO_Open(E_GPC, 14, E_IO_OUTPUT)
#define  INIT_SHADE_STATUS DrvGPIO_Open(E_GPC, 15, E_IO_OUTPUT)
#define  PUMP_STATUS_ON   DrvGPIO_ClrBit(E_GPE, 15)
#define  PUMP_STATUS_OFF  DrvGPIO_SetBit(E_GPE, 15)
#define  HUMIDITY_INNER_STATUS_ON   DrvGPIO_ClrBit(E_GPC, 13)
#define  HUMIDITY_INNER_STATUS_OFF  DrvGPIO_SetBit(E_GPC, 13)
#define  HUMIDITY_OUTER_STATUS_ON   DrvGPIO_ClrBit(E_GPC, 14)
#define  HUMIDITY_OUTER_STATUS_OFF  DrvGPIO_SetBit(E_GPC, 14)
#define  SHADE_STATUS_ON   DrvGPIO_ClrBit(E_GPC, 15)
#define  SHADE_STATUS_OFF  DrvGPIO_SetBit(E_GPC, 15)

#define HMD1_PREFIX "*A"
#define HMD2_PREFIX "*B"
#define TEMP_PREFIX "*T"
#define SWITCHON_PREFIX "C"
#define SWITCHOFF_PREFIX "c"
#define TIMEBAR_PREFIX "*t"
#define SHADEON_PREFIX "B"
#define SHADEOFF_PREFIX "b"
#define LIGHT_TOTAL_PREFIX "*S"
#define LIGHT_PREFIX "*s"
#define SLIDER_START_PREFIX "*W"
#define SLIDER_END_PREFIX "W"
#define GRAPH_PREFIX "*G"
#define GRAPH_SUFFIX '*'

// Timer Variables
#define  ONESHOT  0   // counting and interrupt when reach TCMPR number, then stop
#define  PERIODIC 1   // counting and interrupt when reach TCMPR number, then counting from 0 again
#define  TOGGLE   2   // keep counting and interrupt when reach TCMPR number, tout toggled (between 0 and 1)
#define  CONTINUOUS 3 // keep counting and interrupt when reach TCMPR number

uint32_t Timing_Flag_1s=0;


// THRESHOLDS light
#define PEAK_FORBIDDEN_LIGHT 			85 
#define AVG_FORBIDDEN_LIGHT 			65 
#define NIGHT_LIGHT								5

// THRESHOLDS water
#define HUMIDITY_AVG_THRESHOLD 		24 // (48(max) +0(min))/2 
#define HUMIDITY_LOCAL_THRESHOLD 	3
#define SUFFICIENT_SUN_LIGHT 			20

typedef enum led{led0,led1,led2,led3} led;
typedef enum onOff{off, on} onOff;
typedef enum ADC_Channel{humidity_inner_adc_channel=0, humidity_outer_adc_channel=3, light_adc_channel=2} ADC_Channel;

// uart variables
// longest string possible will look like this:
// length of -> *G100,100,100,100* = 18 chars
char  dataout[20] = "";  
uint32_t len = 0;
//uint8_t dataout[8] = "";

// lcd variables
char TEXT1[16]="HMD1:           ";	
char TEXT2[16]="HMD2:           ";	
char TEXT3[16]="light:          ";	

char UART_INPUT_TEXT[16];
volatile uint8_t comRbuf[9];
volatile uint8_t comRbytes = 0;
volatile uint8_t uart_input;

// Sensors variables
int32_t test; // TODO - delete
uint32_t humidity_inner, humidity_outer, humidity_avg, light_val;
uint32_t cumulative_light_SUM, cumulative_light_counter, cumulative_light_AVG;

// FLAGS
volatile bool sample_gap_flag = true;
bool shade_flag = true;
bool pump_flag = true;
bool humidity_flag_inner = true;
bool humidity_flag_outer = true;

void activate_hour_timer(bool on_off)
{
	if(on_off)
	{
		//SYSCLK->APBCLK.TMR2_EN =1;	//Enable TIMER2 clock source
		//TIMER2->TCSR.CRST = 1;	//Reset up counter
		//TIMER2->TCSR.IE = 1; //Enable interrupt
		TIMER2->TCSR.CEN = 1;		//Enable TIMER2
	}
	else
	{
		TIMER2->TCSR.CEN = 0;		//Disable TIMER2	
		//TIMER2->TCSR.CRST = 1;	//Reset up counter
		//TIMER2->TCSR.IE = 0; //disable interrupt
		//SYSCLK->APBCLK.TMR2_EN =0;	//Disable TIMER2 clock source

	}
}

void activate_shade(bool on_off)
{
	if(on_off==true)
	{
		PWM_Servo(0, SHADE_PWM_ON);
		activate_hour_timer(true);
	}
	else
	{
		PWM_Servo(0, SHADE_PWM_OFF);
		activate_hour_timer(false);
	}
	// enable timer for 1 hour to deactivate shade
}

// TIMER2 initialize to tick every 1ms
void InitTIMER2(void)
{
	/* Step 1. Enable and Select Timer clock source */          
	SYSCLK->CLKSEL1.TMR2_S = 0;	//Select 12Mhz for TIMER2 clock source 
  SYSCLK->APBCLK.TMR2_EN = 1;	//Enable TIMER2 clock source

	/* Step 2. Select Operation mode */	
	TIMER2->TCSR.MODE=PERIODIC;		//Select once mode for operation mode

	/* Step 3. Select Time out period = (Period of timer clock input) * (8-bit Prescale + 1) * (24-bit TCMP)*/
	TIMER2->TCSR.PRESCALE = 11;	// Set Prescale [0~255]
	TIMER2->TCMPR = 1000000;		// Set TCMPR [0~16777215]
	//Timeout period = (1 / 12MHz) * ( 11 + 1 ) * 1,000,000 = 1 s

	/* Step 4. Enable interrupt */
	TIMER2->TCSR.IE = 1;
	TIMER2->TISR.TIF = 1;		//Write 1 to clear for safty		
	NVIC_EnableIRQ(TMR2_IRQn);	//Enable TIMER2 Interrupt

	/* Step 5. Enable Timer module */
	TIMER2->TCSR.CRST = 1;		//Reset up counter
	TIMER2->TCSR.CEN = 0;		//Enable TIMER2

//	TIMER2->TCSR.TDR_EN=1;		// Enable TDR function
}


void TMR2_IRQHandler(void) // Timer1 interrupt subroutine 
{
	static uint32_t Timing_Counter_0=0;
	Timing_Counter_0++;

	if(Timing_Counter_0==HOUR)	//period
	{
		Timing_Counter_0 = 0;
		activate_shade(false);
		return;
	}
	TIMER2->TISR.TIF =1; 	   // to clear the timer flag interrupt
	//TIMER1->TCSR.CRST = 1;	//Reset up counter
	//TIMER1->TCSR.CEN = 1;		//Enable TIMER2
}

void UART_INT_HANDLE(void)
{
	while(UART0->ISR.RDA_IF==1) 
	{
		uart_input=UART0->DATA;
		comRbuf[comRbytes] = uart_input;
		comRbytes++;
		if(comRbytes==1) {	
			//sprintf(UART_INPUT_TEXT,"%s",comRbuf);
			//print_lcd(1,UART_INPUT_TEXT);			
			sprintf(TEXT3+15,"%c",UART0->DATA);
			print_lcd(1,TEXT3);
		  comRbytes=0;
		}
		// extra conditions
		switch(uart_input)
		{
			case 'c':
				PUMP_STATUS_OFF;
				break;
			case 'C':
				PUMP_STATUS_ON;
				break;
			default:
				//PUMP_STATUS_OFF;
				break;
		}
	}
}

void send_to_GUI(void)
{
	// prepare humidity for sending
	len = sprintf(dataout,"%s%d ",HMD1_PREFIX,humidity_inner); // convert ADC7 value into text
	//send to BLUETOOTH GUI
	DrvUART_Write(UART_PORT0, (uint8_t*)dataout,len);
	DrvSYS_Delay(mSEC);

	//print to LCD
	sprintf(TEXT1+6,"%d ",humidity_inner ); // convert ADC7 value into text	
	print_lcd(1, TEXT1);	   // output TEXT to LCD

	// prepare humidity for sending
	len = sprintf(dataout,"%s%d ",HMD2_PREFIX,humidity_outer); // convert ADC7 value into text
	//send to BLUETOOTH GUI
	DrvUART_Write(UART_PORT0, (uint8_t*)dataout,len);
	DrvSYS_Delay(mSEC);
	
	//print to LCD
	sprintf(TEXT2+6,"%d ",humidity_outer); // convert ADC7 value into text	
	print_lcd(2, TEXT2);	   // output TEXT to LCD

	// prepare light reading for sending
	len = sprintf(dataout,"%s%d ",LIGHT_PREFIX,light_val); // convert ADC7 value into text
	//send to bluetooth gui
	DrvUART_Write(UART_PORT0, (uint8_t*)dataout,len);
	DrvSYS_Delay(mSEC);
	
	// print to lcd
	sprintf(TEXT3+6,"%d ",light_val); // convert ADC7 value into text	
	print_lcd(3, TEXT3);	   // output TEXT to LCD
	
	// send to gui graph for all 4 channels
	len = sprintf(dataout,"%s%d,%d,%d,%d%c ",GRAPH_PREFIX,humidity_inner, humidity_outer, light_val, 0, GRAPH_SUFFIX); // convert ADC7 value into text
	DrvUART_Write(UART_PORT0, (uint8_t*)dataout,len);
	DrvSYS_Delay(mSEC);

}

bool is_shade_needed()
{
	if(light_val>PEAK_FORBIDDEN_LIGHT)
		return true;
	else
		if(cumulative_light_AVG>AVG_FORBIDDEN_LIGHT)
			return true;
		else
			return false;	
}

//bool is_watering_time()
//{
//
//}

bool is_sun_down()
{
	return(light_val<NIGHT_LIGHT);
}

bool is_watering_needed()
{ 
	if(humidity_avg>HUMIDITY_AVG_THRESHOLD)
		return false;
	else
		if(humidity_inner>HUMIDITY_LOCAL_THRESHOLD)
			return false;
		else
			if(cumulative_light_AVG > SUFFICIENT_SUN_LIGHT)
				return true;
			else
				return false;
}


void initialization()
{ // start all the decices properly
		//uart variables
	STR_UART_T sParam;
	
	UNLOCKREG();
	SYSCLK->PWRCON.XTL12M_EN = 1; // enable external clock (12MHz)
	SYSCLK->CLKSEL0.HCLK_S = 0;	  // select external clock (12MHz)
	DrvSYS_Open(48000000);
	//LOCKREG();
	
	DrvGPIO_InitFunction(E_FUNC_UART0);	// Set UART pins
	/* UART Setting */
	sParam.u32BaudRate 		  = 9600;
	sParam.u8cDataBits 		  = DRVUART_DATABITS_8;
	sParam.u8cStopBits 		  = DRVUART_STOPBITS_1;
	sParam.u8cParity 		    = DRVUART_PARITY_NONE;
	sParam.u8cRxTriggerLevel= DRVUART_FIFO_1BYTES;
	/* Set UART Configuration */
 	if(DrvUART_Open(UART_PORT0,&sParam) != E_SUCCESS);
	DrvUART_EnableInt(UART_PORT0, DRVUART_RDAINT, UART_INT_HANDLE);

	Initial_panel();  // initialize LCD pannel
	clr_all_panel();  // clear LCD panel 
	print_lcd(0, "Water My Garden");

	DrvADC_Open(ADC_SINGLE_END, ADC_CONTINUOUS_OP, 0x8D, EXTERNAL_12MHZ, 255); 
	DrvADC_StartConvert();                   // start A/D conversion					 					 

	InitPWM(0);            // initialize PWM0 - GPA12
	PWM_Servo(0, SHADE_PWM_OFF);

	INIT_PUMP_STATUS; // gpio output for water pump\
	
	InitTIMER2();
}

void activate_pump()
{
	PUMP_STATUS_ON;
	DrvSYS_Delay(SEC);
	DrvSYS_Delay(SEC);
	DrvSYS_Delay(SEC);
	PUMP_STATUS_OFF;
}


void gather_data()
{
	while(ADC->ADSR.ADF==0); // wait till conversion flag = 1, conversion is done
	ADC->ADSR.ADF=1;		     // write 1 to clear the flag
	
	//get humidity
	humidity_inner = DrvADC_GetConversionData(humidity_inner_adc_channel); // HMD1
	humidity_inner =100-(humidity_inner *100)/4095;
	
	//get humidity
	humidity_outer = DrvADC_GetConversionData(humidity_outer_adc_channel); // HMD1
	humidity_outer =100-(humidity_outer*100)/4095;
	
	humidity_avg = (humidity_inner + humidity_outer)/2;
	
	//get light reading
	light_val = DrvADC_GetConversionData(light_adc_channel); // HMD1
	light_val=100-(light_val*100)/4095;
	
	cumulative_light_SUM += light_val;
	cumulative_light_counter++; 
	cumulative_light_AVG =  cumulative_light_SUM / cumulative_light_counter;
}

int32_t main (void)
{
	initialization();	

	while(true)
	{
		//while(sample_gap_flag==false); // wait here until timer interrupt occured		
		gather_data();
		send_to_GUI();
		
		if(is_shade_needed())
			activate_shade(true);			

		if(is_sun_down())
			if(is_watering_needed())
				activate_pump();

		DrvSYS_Delay(SEC); // TODO - repalce with timer interrupt
	}
}
