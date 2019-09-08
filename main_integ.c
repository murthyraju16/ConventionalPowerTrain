
/*
PD0 -> seat belt
PD1- gear neutral
PD2- knock
PD3- ignition switch
PD4 -Set speed

Analog inputs
PC0- Pedal sensor
PC1-  Temperature
PC2- Engine load
PC3-  Engine speed
PC4-  EGO
PC5-  Battery voltage


Outputs:
PD5-ignition angle        -OC0B
PB3-Throttle valve        -OC2A
PD6-fuel injection PWM    -OC0A
PB5- Relay output
*/

#include <avr/io.h>
#include"ADC.h"
#include"PWM.h"
#include<avr/interrupt.h>
/******************************************************************************
*                      Defines and data types
******************************************************************************/
#define CRANKSPEED_THRE 13  //0.0625V
#define MIN_BATTERY_THRE 766 // 11V - 3.7V
#define ENGINE PD3
#define READ_BIT(PORT,PIN) (PORT&(1<<PIN))

static uint8_t rpm_index(float);
static uint8_t load_index(float);
static uint8_t temp_index(float);
static uint8_t oxy_index(float);

/*********************************  *********************************************
*                    Main Function
******************************************************************************/
typedef struct
{
    volatile uint8_t FLAG_BIT:1;

} FLAG;

FLAG IGNITION_FLAG;

int main(void)
{


    CLR_BIT(DDRD,PD0);  /* SEAT BELT */
    CLR_BIT(DDRD,PD1);   /* GEAR NEUTRAL */             /*All of them are in pull down configuration */
    CLR_BIT(DDRD,PD2);   /* KNOCK */
    CLR_BIT(DDRD,ENGINE);   /* IGNITION SWITCH */
    CLR_BIT(DDRD,PD4);   /* SET SPEED MODE */


    SET_BIT(DDRB,PB3);   // PB3 (OC2A)output
    SET_BIT(DDRD,PD5);  // PD5 (OC0B)
    SET_BIT(DDRB,PB5);  //RELAY
    SET_BIT(DDRD,PD6);  /* PD6(OC0A) OUTPUT = FUEL INJECTION DURATION IN MILLI SECONDS AS PWM PULSE WIDTH */


    EICRA|=(1<<ISC10);
    EICRA&=~(1<<ISC11); // Any logical change will raise interrupt

    EIMSK|=(1<<INT1);   //Local Interrupt enable

    CLR_BIT(DDRC,PC1);          /* PC1 --> COOLANT TEMPERATURE SENSOR */
    CLR_BIT(DDRC,PC2);          /* PC2 --> ENGINE LOAD SENSOR */
    CLR_BIT(DDRC,PC3);          /* PC3 --> ENGINE SPEED SENSOR */
    CLR_BIT(DDRC,PC4);          /* PC4 --> EXHAUST GAS OXYGEN SENSOR */


    // In Look up table of Fuel timing map the row is for RPM and column id for Load
    int Fuel_Map[16][16]= {{15.1, 14.9, 14.8, 14.6, 14.4, 13.9, 13.5, 12.5, 12.5, 12.5, 12.5, 12.5, 12.5,12.5, 12.5, 12.5},
                           {15.1, 14.9, 14.7, 14.3, 14.3, 13.7, 13.5, 12.7, 12.7, 12.7,12.7, 12.7, 12.7, 12.7, 12.7, 12.7},
                           {15.1, 14.9, 14.7, 14.0, 13.9, 13.6, 13.5, 12.9, 12.9, 12.9, 12.9, 12.9, 12.9, 12.9, 12.9, 12.9},
                           {15.1, 14.7, 14.3, 13.9, 13.8, 13.6, 13.5, 13.5, 13.5, 13.5, 13.5, 13.5, 13.5, 13.4, 13.4, 13.4},
                           {15.1, 14.7, 13.9, 13.9, 13.8, 13.5, 13.4, 13.4, 13.4, 13.4, 13.2, 13.2, 13.2, 13.2, 13.2, 13.2},
                           {15.1, 14.7, 13.9, 13.8, 13.8, 13.5, 13.3, 12.7, 12.5, 12.5, 12.5, 12.5, 12.5, 12.5, 12.5, 12.5},
                           {14.1, 14.1, 13.9, 13.8, 13.7, 13.4, 13.3, 12.7, 12.5, 12.5, 12.4, 12.3, 12.3, 12.2, 12.0, 12.0},
                           {13.9, 13.9, 13.7, 13.7, 13.5, 13.3, 13.2, 12.6, 12.2, 11.3, 11.2, 11.2, 11.2, 11.1, 11.1, 11.0},
                           {13.8, 13.6, 13.5, 13.5, 13.4, 13.2, 13.2, 12.6, 12.1, 11.3, 11.2, 11.2, 11.2, 11.1, 11.0, 10.9},
                           {13.5, 13.5, 13.3, 13.2, 13.2, 13.1, 13.1, 12.6, 12.0, 11.2, 11.1, 11.1, 11.1, 11.0, 10.9, 10.6},
                           {13.5, 13.4, 13.2, 13.1, 13.1, 13.0, 12.9, 12.5, 12.0, 11.2, 11.1, 11.1, 11.0, 10.9, 10.6, 10.5},
                           {13.3, 13.2, 13.2, 13.1, 13.0, 12.9, 12.8, 12.5, 11.9, 11.1, 11.0, 11.0, 11.0, 10.8, 10.4, 10.4},
                           {13.2, 13.1, 13.2, 13.1, 13.0, 12.8, 12.8, 12.2, 11.8, 11.1, 11.0, 10.9, 10.9, 10.7, 10.3, 10.2},
                           {13.1, 13.1, 13.0, 13.0, 12.9, 12.6, 12.3, 11.9, 11.7, 11.0, 10.9, 10.8, 10.7, 10.6, 10.2, 10.2},
                           {13.1, 13.1, 12.9, 12.9, 12.8, 12.4, 12.3, 11.9, 11.7, 10.9, 10.8, 10.7, 10.6, 10.5, 10.1, 10.0},
                           {12.9, 12.9, 12.9, 12.8, 12.6, 12.3, 12.0, 11.9, 11.7, 10.8, 10.7, 10.6, 10.5, 10.4, 10.0, 10.0}};
    /* Look up table for Engine temperature sensor */
    float temp_factor[5]={1.2,1.15,1.0,0.99,0.98};
    /* Look up table for EGO sensor */
    float oxy_factor[7] = { 1.0, 1.0, 0.97, 0.94, 0.9, 0.83, 0.75 };
    float Engine_load,Engine_speed,coolant_temp,EGO_level;
    uint8_t   load_ind,rpm_ind,temp_ind,oxy_ind;

    float base_PW,pulse_width,AFR_value;

    uint16_t  set_speed;
    uint16_t ADC_OUTPUT=0;

    uint16_t ADC_value1;
    uint16_t ADC_value2;
    uint8_t speed_inc=0,load_inc=0,out;


    ADC_INIT();
    ENABLE_ADC();
    PWM_setup();
    TIMER2PWM_init();


    sei();

    while(1)
    {
         if(READ_BIT(PIND,ENGINE)) /* Ignition switch for engine is on */
         {
    /*----....---- Staring System ----....---- */

            uint16_t crankSpeed_V = ReadADC(3);
            uint16_t battery_V = ReadADC(5);

            if(READ_BIT(PIND,PD0) && READ_BIT(PIND,PD1) && (battery_V>MIN_BATTERY_THRE)    && crankSpeed_V<64   && crankSpeed_V>CRANKSPEED_THRE )
            {
                 PORTB|=(1<<PB5);
            }
            else
            {
             PORTB&=~(1<<PB5);    /* OFF*/
            }
    /*----....---- iNTEGRATION CONDITION ----....---- */

            if((~READ_BIT(PORTB,PB5) && ReadADC(3)<64) || ReadADC(3) >=64 ) /* either if engine speed is less than 500rpm and relay should be on or enginespeed has to be greater than 500rpm */
            {
    /*----....---- Electronic Fuel Injection System ----....---- */

                PWM_enable();
                Engine_load=(ReadADC(2)*(230.0/1023.0))+20.0;                /* map = 20 to 250 kPa */
                load_ind=load_index(Engine_load);

                Engine_speed=ReadADC(3)*(8000.0/1023.0);                   /* rpm = 0-8000 */
                rpm_ind=rpm_index(Engine_speed);

                coolant_temp=(ReadADC(1)*(110.0/1023.0))+273.0;             /* temp = 273k to 383k*/ /*--considering the india temperature--*/
                temp_ind=temp_index(coolant_temp);

                EGO_level=(ReadADC(4)*(0.6/1023.0))+0.2;                    /* ego  = 0.2 to 0.8*/
                oxy_ind=oxy_index(EGO_level);

                AFR_value = Fuel_Map[rpm_ind][load_ind];                    /* depending on the speed and load on engine , the base pulse width is identified from look-up table of fuel-timing map */
                base_PW =  2.0 / (4.0 * AFR_value);
                pulse_width = base_PW*temp_factor[temp_ind]*oxy_factor[oxy_ind]; /* effect on injection timing by engine temperature and o2 level in exhaust gas. */

                Injection_PWM(pulse_width); /* generating pwm signal depends on the engine speed,temperature,o2 level and engine load */

     /*----....---- Electronic Throttle System ----....---- */
                if(READ_BIT(PIND,PD4))
                {
                    set_speed=ReadADC(3);
                    if(0<=set_speed && set_speed<=256)
                    {
                        SET_PWM_ON_VALUE(35);
                    }
                    else if (257<=set_speed && set_speed<=512)
                    {
                        SET_PWM_ON_VALUE(45);
                    }
                    else if(513<=set_speed && set_speed<=768)
                    {
                        SET_PWM_ON_VALUE(55);
                    }
                    else if(768<=set_speed && set_speed<=1023)
                    {
                        SET_PWM_ON_VALUE(65);
                    }
                    else SET_PWM_ON_VALUE(0);
                }
                else
                {  // manual mode
                    ADC_OUTPUT = ReadADC(0);
                    if(ADC_OUTPUT>850)
                        SET_PWM_ON_VALUE(85);                                //restricting Throttle position to 85% duty cycle
                    else if(ADC_OUTPUT>750&& ADC_OUTPUT<850)
                        SET_PWM_ON_VALUE(75);                               //restricting Throttle position to 75% duty cycle
                    else if(ADC_OUTPUT>650 && ADC_OUTPUT<750)
                        SET_PWM_ON_VALUE(65);                              //restricting Throttle position to 65% duty cycle
                    else if(ADC_OUTPUT>550 && ADC_OUTPUT<649)
                        SET_PWM_ON_VALUE(55);                              //restricting Throttle position to 55% duty cycle
                    else if(ADC_OUTPUT>450  && ADC_OUTPUT<549)
                        SET_PWM_ON_VALUE(45);                              //restricting Throttle position to 45% duty cycle
                    else if(ADC_OUTPUT>350  && ADC_OUTPUT<449)
                        SET_PWM_ON_VALUE(35);                              //restricting Throttle position to 35% duty cycle
                    else if(ADC_OUTPUT>250 &&  ADC_OUTPUT<349)
                        SET_PWM_ON_VALUE(25);                              //restricting Throttle position to 25% duty cycle
                    else if(ADC_OUTPUT>150 &&  ADC_OUTPUT<249)
                        SET_PWM_ON_VALUE(15);                              //restricting Throttle position to 15% duty cycle
                    else if(ADC_OUTPUT>50 && ADC_OUTPUT<149)
                        SET_PWM_ON_VALUE(10);                              //restricting Throttle position to 10% duty cycle
                    else if(ADC_OUTPUT>0 && ADC_OUTPUT<49)
                        SET_PWM_ON_VALUE(5);                               //restricting Throttle position to 5% duty cycle

                }
    /*----....---- Electronic Ignition System ----....---- */
                ADC_value1=ReadADC(3);
                ADC_value2=ReadADC(2);
                if(ADC_value1>=10&&ADC_value2>=10)
                {
                    speed_inc=ADC_value1/100;
                    switch(speed_inc)
                    {
                        case 1:
                            out=64;
                            break;
                        case 2:
                            out=77;
                            break;
                        case 3:
                            out=90;
                            break;
                        case 4:
                            out=102;
                            break;
                        case 5:
                            out=115;
                            break;
                        case 6:
                            out=140;
                            break;
                        case 7:
                            out=155;
                            break;
                        case 8:
                            out=170;
                            break;
                        case 9:
                            out=180;
                            break;
                        case 10:
                            out=195;
                            break;
                        default:
                            out=55;
                            break;
                    }

                    load_inc=ADC_value2/100;
                    switch(load_inc)
                    {
                        case 1:
                            out=out-2;
                            break;
                        case 2:
                            out=out-4;
                            break;
                        case 3:
                            out=out-6;
                            break;
                        case 4:
                            out=out-8;
                            break;
                        case 5:
                            out=out-10;
                            break;
                        case 6:
                            out=out-12;
                            break;
                        case 7:
                            out=out-14;
                            break;
                        case 8:
                            out=out-16;
                            break;
                        case 9:
                            out=out-18;
                            break;
                        case 10:
                            out=out-20;
                            break;
                        default:
                            out=out;
                            break;
                    }
                    if(READ_BIT(PIND,PD2))
                    {
                        out = out - 12.75;
                    }
                    else out = out;
                }
                Ignition_PWM(out);
            }
        }
        else        /*when engine is not working */
        {
            CLR_BIT(PORTB,PB5);
            Injection_PWM(0);
            SET_PWM_ON_VALUE(0);
            out = 0;
            Ignition_PWM(out);
        }
   }
    return 0;
}

ISR(INT1_vect)
{

    if(IGNITION_FLAG.FLAG_BIT==1)
    {
    IGNITION_FLAG.FLAG_BIT = 0;
    }
    else IGNITION_FLAG.FLAG_BIT = 1;


}
/*********************************************************************************************************************
*           Index's for look up tables of fuel - timing map,Engine temperature sensor,EGO sensor                     *
*********************************************************************************************************************/

static uint8_t rpm_index(float value)
{
    uint8_t index;
    if ((value>=0) && (value<350))
    {
        index=0;
    }
    else if ((value>=350) && (value < 750))
    {
        index=1;

    }
    else if ((value>= 750) &&(value < 1000))
    {
        index=2;

    }
    else if ((value>= 1000) && (value < 1500))
    {
        index=3;
    }
    else if ((value >= 1500) && (value < 2000))
    {
        index=4;
    }
    else if ((value >= 2000) && (value < 2500))
    {
        index=5;
    }
    else if ((value >= 2500) && (value < 3000))
    {
        index=6;
    }
    else if ((value >= 3000) && (value < 3500))
    {
        index=7;
    }
    else if ((value >= 3500) && (value < 4000))
    {
        index=8;
    }
    else if ((value >= 4000) && (value < 4500))
    {
        index=9;
    }
    else if ((value >= 4500) && (value < 5000))
    {
        index=10;
    }
    else if ((value >= 5000) && (value < 5500))
    {
        index=11;
    }
    else if ((value >= 5500) && (value < 6000))
    {
        index=12;
    }
    else if ((value >= 6000) && (value < 6750))
    {
        index=13;
    }
    else if ((value >= 6750) && (value < 7500))
    {
        index=14;
    }
    else if ((value >= 7500) && (value <= 8000))
    {
        index=15;
    }
    return index;
}

static uint8_t load_index(float value)
{
    uint8_t index;
    if ((value>=20) && (value<35))
    {
        index=0;
    }
    else if ((value>=35) && (value < 50))
    {
        index=1;

    }
    else if ((value>= 50) &&(value < 65))
    {
        index=2;

    }
    else if ((value>= 65) && (value < 70))
    {
        index=3;
    }
    else if ((value >= 70) && (value < 85))
    {
        index=4;
    }
    else if ((value >= 85) && (value < 100))
    {
        index=5;
    }
    else if ((value >= 115) && (value < 130))
    {
        index=6;
    }
    else if ((value >= 130) && (value < 145))
    {
        index=7;
    }
    else if ((value >= 145) && (value < 160))
    {
        index=8;
    }
    else if ((value >= 160) && (value < 175))
    {
        index=9;
    }
    else if ((value >= 175) && (value < 190))
    {
        index=10;
    }
    else if ((value >= 190) && (value < 205))
    {
        index=11;
    }
    else if ((value >= 205) && (value < 220))
    {
        index=12;
    }
    else if ((value >= 220) && (value < 230))
    {
        index=13;
    }
    else if ((value >= 230) && (value < 240))
    {
        index=14;
    }
    else if ((value >= 240) && (value <= 250))
    {
        index=15;
    }
    return index;
}

static uint8_t temp_index(float value)
{
    uint8_t index;
    if ((value>=273.0) && (value<295.0))
    {
        index=0;
    }
    else if ((value>=295.0) && (value < 317.0))
    {
        index=1;

    }
    else if ((value>= 317.0) &&(value < 339.0))
    {
        index=2;

    }
    else if ((value>= 339.0) && (value < 361.0))
    {
        index=3;
    }
    else if ((value >= 361.0) && (value <= 383.0))
    {
        index=4;
    }
    return index;
}

static uint8_t oxy_index(float value)
{
    uint8_t index;
    if ((value>=0.2) && (value<0.35))
    {
        index=0;
    }
    else if ((value>=0.35) && (value <= 0.45))
    {
        index=1;

    }
    else if ((value> 0.45) &&(value < 0.55))
    {
        index=2;

    }
    else if ((value>= 0.55) && (value < 0.65))
    {
        index=3;
    }
    else if ((value >= 0.65) && (value < 0.7))
    {
        index=4;
    }
    else if ((value >= 0.7) && (value < 0.75))
    {
        index=5;
    }
    else if ((value >= 0.75) && (value <= 0.8))
    {
        index=6;
    }
    return index;
}

/******************************************************************************
*                      End of File
******************************************************************************/




