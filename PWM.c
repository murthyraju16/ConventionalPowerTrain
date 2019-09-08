
/******************************************************************************
* File Name: PWM.c
* Description:  This file contains the set up for fast-PWM mode
******************************************************************************/
/******************************************************************************
*                      Includes
******************************************************************************/
#include"PWM.h"
#include<avr/io.h>
#include"avr/interrupt.h"
#include<util/delay.h>
/******************************************************************************
*                      Defines and data types
******************************************************************************/


/******************************************************************************
*                      Global variables
******************************************************************************/

/******************************************************************************
*                      Static variables
******************************************************************************/
/******************************************************************************
*                      Internal function prototypes
******************************************************************************/
/******************************************************************************
*                      Public functions
******************************************************************************/
void PWM_setup()
{
TCCR0A|= ((1<<WGM01) | (1<<WGM00));  /* fast pwm mode*/
TCCR0A|= (1<< COM0A1); /* non inverting mode  and clear OC0A on compare match*/
TCCR0A&= ~(1<<COM0A0);
TCCR0A|= (1<< COM0B1); /* non inverting mode  and clear OC0B on compare match*/
TCCR0A&= ~(1<<COM0B0);

TCNT0=0x00;   /*Timer counter register*/
OCR0A=0x00;     /* FOR INJECTION DURATION */
OCR0B=0x00;     /* FOR IGNITION ANGLE */
sei();       /* Global interrupt*/
}

void Injection_PWM(float time_val)
{
    OCR0A = (time_val/0.080)*(255.0);
    /* Time period of each pulse is 50 msec */
    /* for 30 msec the duty cycle will be adjusted accordingly by the above formula */
}

void PWM_enable()
{
    TCCR0B|= ((1<<CS00)); /*clock and WGM02*/
    TCCR0B&=~(1<<CS01);
    TCCR0B|=(1<<CS02);      /*101    // timer is on*/
}

void TIMER2PWM_init()
{
    SET_BIT(DDRB,PB3); // PD6 (OC0A)output
    TCNT2 =0x00;  // give access to timer and counter
    TCCR2A |= ((1<<WGM20)|(1<<WGM21)|(1<<COM2A1)); // enabled the mode and effect
    TCCR2B |= 1<<CS22; //clock and WGM02
}

void Ignition_PWM(uint8_t ignvalue)
{
    OCR0B=ignvalue;
    _delay_ms(500);
    OCR0B=ignvalue;
    _delay_ms(500);

}

void PWM_disable()
{

   TCCR0B&=0X00;        /* clock freq is 0 */

}



/******************************************************************************
*                      Internal functions
******************************************************************************/

/******************************************************************************
*                      End of File
******************************************************************************/

