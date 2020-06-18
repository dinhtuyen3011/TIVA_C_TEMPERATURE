#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"

#include <driverlib/timer.h>
#include <driverlib/interrupt.h>
#include <driverlib/sysctl.h>
#include <driverlib/uart.h>
#include <driverlib/adc.h>
#include <driverlib/gpio.h>
#include <driverlib/pwm.h>
#include "utils/uartstdio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom_map.h"

char str[100];
float externalTemp, internalTemp;
void ConfigureUART(void);
void config_adc(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);//enable adc0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);//configure port e is adc peripheral
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);//pin pe3 <-> adc channel 0
	ADCSequenceConfigure(ADC0_BASE,0, ADC_TRIGGER_PROCESSOR, 0);//set up adc trigger : processor trigger
	ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH0);//get adc from adc channel 0
	ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_TS | ADC_CTL_IE |ADC_CTL_END);//get adc from internal sensor
	ADCSequenceEnable(ADC0_BASE, 0);//0 senquence 0 (doc adc sequence cua ti)
}
void config_timer(void)
{
	uint32_t ui32Period;
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);//enable timer0
	TimerConfigure(TIMER0_BASE, TIMER_CFG_A_PERIODIC);//configure timer 0


	ui32Period = SysCtlClockGet()/1; //set up timer period =1s;

	TimerLoadSet(TIMER0_BASE,TIMER_A,ui32Period-1);// ui32Period -1 because count down to 0 then timer handler will be enable;

	IntEnable(INT_TIMER0A);//enable handler adc
	TimerIntEnable(TIMER0_BASE,TIMER_TIMA_TIMEOUT);
	IntMasterEnable();

	TimerEnable(TIMER0_BASE, TIMER_A);//enable timer A
}

void ConfigureUART(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);//enable uart peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);//configure gpio port A is uart peripheral

    GPIOPinConfigure(GPIO_PA0_U0RX);//configure pin PA0 is UART RX
    GPIOPinConfigure(GPIO_PA1_U0TX);//configure pin PA1 is UART TX
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    UARTStdioConfig(0, 115200, 16000000); // baurate 115200
    IntEnable(INT_UART0);//enable uart handler
	UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
	IntMasterEnable();
}
//chuong trinh ngat uart
void UART0_IntHandler(void)
{
	uint32_t ui32Status;
	ui32Status = UARTIntStatus(UART0_BASE, true);
	UARTIntClear(UART0_BASE, ui32Status);
}
void Timer0IntHandler(void)
{
	TimerIntClear(TIMER0_BASE,TIMER_TIMA_TIMEOUT);
	uint32_t Data[2];
	ADCIntClear(ADC0_BASE,0);
	ADCProcessorTrigger(ADC0_BASE, 0);
	while (!ADCIntStatus(ADC0_BASE,0, false))
	{
	}
	ADCSequenceDataGet(ADC0_BASE, 0, Data);
	externalTemp =(Data[0])*330/4096; // Vref=3,3V,ADC 12 bit --> divided for 2^12 VOUT(mV) = 10mV/°C * T -->T = (ADC_RESULT * VREF)/4096 * 0.01 °C = ADC_RESULT*3,3*100/4096
	internalTemp = (1475 - (2475* Data[1]) / 4096)/10;//TEMP = 147.5 - ((75 * (Vre4f) * ADCCODE) / 4096)
	sprintf(str,"%.3f,%.3f",externalTemp,internalTemp);
	UARTprintf("%s \r",str);
	memset(str, 0, sizeof(str));
}
int main(void)
{
	SysCtlClockSet(SYSCTL_SYSDIV_5| SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);	//Cau hinh 3 LED la output
	GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,0xff);//turn on led red
	GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2,0xff);//turn on led blue

	config_adc();
	config_timer();
	ConfigureUART();
	while(1)
	{
	}

}

