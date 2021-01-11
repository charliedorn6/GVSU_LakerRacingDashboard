/*
 * Author: Nigel Armstrong
 * Contact: armstron@mail.gvsu.edu
 * Date of Last Update: 2020.09.11
 * Version #: v1.0.0
 * Use: For use with custom EGR436 Dashboard PCB designed by Nigel Armstrong and John Santose. This program only performs current needed functions as a dashboard.
 *      This program does not add functions for bluetooth, usb interfacing, shift logging, smart shifting, or numeric RPM readout.
 * Description: This program measures RPM from the tachometer signal off of the PE3 ECU when set up for 8 tach pulses per rev.
 *              This program also automates shifting for paddle shifters, ensuring when a paddle is pulled, there is feedback to ensure a new gear is selected.
 *              This program also reports RPM readout to an adjustable RGB lightstrip with adjustable shift lights.
 *              This program sets up all pins that are used on the EGR436 dashboard PCB, but not all are used for functions in this program.
 */

/* DriverLib Includes */
#include "driverlib_files/driverlib.h"

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include "msp.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "string.h"


//Lightstrip value settings
#define BRIGHTNESS 3
#define NUM_GREEN_LEDS 18
#define NUM_YELLOW_LEDS 6
#define NUM_RED_LEDS 6
#define NUM_LEDS (NUM_GREEN_LEDS + NUM_YELLOW_LEDS + NUM_RED_LEDS)
#define MAX_RPM 12000

void sysTickInit(void);
void msDelay(int delay);
void pinInit(void);
void spiInit(void);
void sendSPIbyte(uint8_t s);
void sendLS_START(void);
void sendLS(uint8_t red, uint8_t green, uint8_t blue);
void clearLS(void);
void rpmtoLS(void);

uint8_t lightstrip [NUM_LEDS] [3];
int i;
uint16_t rpm;
uint8_t ledsON;
uint16_t rpmCaptureValue;
uint8_t gearIndex = 1;
bool LSflash_flag = 0;
bool LSflag = 0;
bool up_flag = 0;
bool down_flag = 0;
uint8_t gearON  [10] = {0x77, 0x05, 0xB3, 0xA7, 0xC5, 0xE6, 0xF6, 0x07, 0xF7, 0xE7};
uint8_t gearOFF [10] = {0x88, 0xFA, 0x4C, 0x58, 0x3A, 0x19, 0x09, 0xF8, 0x08, 0x18};

void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer

    pinInit();
    sysTickInit();
    spiInit();

    for(i = 0; i < NUM_LEDS; i++){//FOR LOOP TO LOAD COLOR SETTINGS INTO RPM LIGHTSTRIP
        if(i < NUM_GREEN_LEDS){      //GREEN
            lightstrip [i] [0] = 0;
            lightstrip [i] [1] = 255;
            lightstrip [i] [2] = 0;
        }
        else if((i < (NUM_GREEN_LEDS + NUM_YELLOW_LEDS)) && (i >= NUM_GREEN_LEDS)){//YELLOW
            lightstrip [i] [0] = 255;
            lightstrip [i] [1] = 255;
            lightstrip [i] [2] = 0;
        }
        else{                        //RED
            lightstrip [i] [0] = 255;
            lightstrip [i] [1] = 0;
            lightstrip [i] [2] = 0;
        }
    }

    while (1)
    {
        if(LSflag){//If new RPM read in is available
            rpmtoLS();
            LSflag = 0;//reset flag
        }
        if(up_flag && gearIndex < 6){//If upshift paddle is pulled and current gear value is under 6
            gearIndex++;//increase gear index
            P4OUT |= gearON[gearIndex];//print increased gear to 7 segment
            P4OUT &= ~gearOFF[gearIndex];//turn off superfluous segments
            up_flag = 0;//reset flag
        }
        else if(down_flag && gearIndex > 1){//If downshift paddle is pulled and current gear value is above 1
            gearIndex--;//decrease gear index
            P4OUT |= gearON[gearIndex];//print decreased gear to 7 segment
            P4OUT &= ~gearOFF[gearIndex];//turn off superfluous segments
            down_flag = 0;//reset flag
        }
    }
}

void sysTickInit(void)//Initializes systick timer for use in delays
{

    SysTick->CTRL &= ~BIT0; //clears enable to stop the counter
    SysTick->LOAD = 0x00FFFFFF; //sets the period... note: (3006600/1000 - 1) = 1ms
    SysTick->VAL = 0; //clears the value
    SysTick->CTRL = 0x00000005; //enable SysTick, without interrupts
}

void msDelay(int delay){//input # of ms to delay (assuming 48 MHz sysclock)

    SysTick->LOAD = delay * 48000 - 1;           //counts up to delay
    SysTick->VAL = 0;                           //starts counting from 0
    while ((SysTick->CTRL & 0x00010000) == 0);  //wait until flag is set (delay number is reached)
}

void pinInit(void){//initializes all IO pins available on custom board
    //UART pins 1.2 and 1.3 for USB interfacing
    P1SEL0 |=  0x0C;//0b.0000.1100
    P1SEL1 &= ~0x0C;
    P1SEL0 &= ~0x01;
    P1SEL1 &= ~0x01;
    P1DIR |= 0x01;

    //SPI pins 2.1, 2.2, 2.3, and 7.4 for FRAM interfacing
    P2SEL0 |=  0x0E;//0b.0000.1110
    P2SEL1 &= ~0x0E;
    P7SEL0 &= ~0x10;//0b.0001.0000
    P7SEL1 &= ~0x10;
    P7DIR  |=  0x10;//Output
    P7OUT  &= ~0x10;

    //SPI pins 3.1 and 3.3 for Lightstrip
    //I2C pins 3.6 and 3.7 for 4-digit display
    P3SEL0 |=  0xCA;//0b.1100.1010
    P3SEL1 &= ~0xCA;

    //GPIO pins 4.0-4.7 for 7-segment gear display
    P4SEL0 &= ~0xFF;//0b.1111.1111
    P4SEL1 &= ~0xFF;
    P4DIR  |=  0xFF;//Outputs
    P4OUT  &= ~0xFF;//All off

    //ADC pin 5.5 for battery voltage
    P5SEL0 |=  0x20;//0b.0010.0000
    P5SEL1 |=  0x20;

    //Interrupt pins 6.0-6.2 and 6.4-6.6 for paddles/hall-effect sensors
    P6SEL0 &= ~0x77;//0b.0111.0111
    P6SEL1 &= ~0x77;
    P6DIR  &= ~0x77;//Inputs
    P6IES  |=  0x77;//Interrupt on falling edge
    P6IFG  &= ~0xFF;//Clear all interrupt flags
    P6IE   |=  0x77;//Interrupt enable

    //TimerA CC pin 7.3 for tachometer read from ECU
    P7SEL0 |=  0x08;//0b.0000.1000
    P7SEL1 &= ~0x08;

    //GPIO pins 8.4-8.6 for shifting relay outputs
    P8SEL0 &= ~0x70;//0b.0111.0000
    P8SEL1 &= ~0x70;
    P8DIR  |=  0x70;//Outputs
    P8OUT  &= ~0x70;//All off

    //UART pins 9.6 and 9.7 for Bluetooth interfacing
    P9SEL0 |=  0xC0;//0b.1100.0000
    P9SEL1 &= ~0xC0;
}

void spiInit(void){//initializes SPI modules and timers for various functions
    //Configure SPI for FRAM module
    EUSCI_A1->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Put eUSCI state machine in reset
    EUSCI_A1->CTLW0 = EUSCI_A_CTLW0_SWRST | // Remain eUSCI state machine in reset
            EUSCI_A_CTLW0_MST |             // Set as SPI master
            EUSCI_A_CTLW0_SYNC |            // Set as synchronous mode
            EUSCI_A_CTLW0_CKPL |            // Set clock polarity high
            EUSCI_A_CTLW0_MODE_2 |          // command select low
            EUSCI_A_CTLW0_MSB;              // MSB first

    EUSCI_A1->CTLW0 |= EUSCI_A_CTLW0_SSEL__SMCLK; // SMCLK
    EUSCI_A1->BRW = 0x01;                   // /2,fBitClock = fBRCLK/(UCBRx+1).
    EUSCI_A1->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;    // Initialize USCI state machine

    // Enable eUSCI_A1 interrupt in NVIC module
    //NVIC->ISER[0] = 1 << ((EUSCIA1_IRQn) & 31);

    //Configure SPI for Lightstrip
    EUSCI_A2->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Put eUSCI state machine in reset
    EUSCI_A2->CTLW0 = EUSCI_A_CTLW0_SWRST | // Remain eUSCI state machine in reset
            EUSCI_A_CTLW0_MST |             // Set as SPI master
            EUSCI_A_CTLW0_SYNC |            // Set as synchronous mode
            EUSCI_A_CTLW0_CKPL |            // Set clock polarity high
            EUSCI_A_CTLW0_MODE_0 |          // command select low
            EUSCI_A_CTLW0_MSB;              // MSB first

    EUSCI_A2->CTLW0 |= EUSCI_A_CTLW0_SSEL__SMCLK; // SMCLK
    EUSCI_A2->BRW = 0x02;                   // /3,fBitClock = fBRCLK/(UCBRx+1).
    EUSCI_A2->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;    // Initialize USCI state machine

    // Enable eUSCI_A1 interrupt in NVIC module
    NVIC->ISER[0] = 1 << ((EUSCIA2_IRQn) & 31);

    TIMER_A1->CCTL[0] = TIMER_A_CCTLN_CCIE; // TACCR0 interrupt enabled
    TIMER_A1->CCR[0] = 4000;//Flash speed is roughly (4000 * 2)/120000 ~0.07 seconds per flash
    TIMER_A1->CTL = TIMER_A_CTL_SSEL__ACLK | // ACLK, continuous mode
                TIMER_A_CTL_MC__CONTINUOUS;

    NVIC->ISER[0] = 1 << ((TA1_0_IRQn) & 31);

    TIMER_A0->CCTL[0] = TIMER_A_CCTLN_CM_1 | // Capture rising edge,
            TIMER_A_CCTLN_CCIS_0 |          // Use CCI2A=ACLK,
            TIMER_A_CCTLN_CCIE |            // Enable capture interrupt
            TIMER_A_CCTLN_CAP |             // Enable capture mode,
            TIMER_A_CCTLN_SCS;              // Synchronous capture

    TIMER_A0->CTL |= TIMER_A_CTL_TASSEL_2 | // Use SMCLK as clock source,
            0x00C0 |                        // 8 times divider (total 64 division)
            TIMER_A_CTL_MC__CONTINUOUS;   // Start timer in continuous mode

    TIMER_A0->EX0 |= 0x0007;      //8 times divider (total 64 division)

    NVIC->ISER[0] = 1 << ((TA0_0_IRQn) & 31);



    __enable_irq();//Enables all global interrupts on MSP
}

void sendSPIbyte(uint8_t s)//sends 8 bits (1 byte) to EUSCI_A2 (lightstrip)
{
    while ((EUSCI_A2->STATW & 0x0001));
    EUSCI_A2->TXBUF = (s);   // Load response data onto buffer and send
}

void sendLS_START(void)//Start frame for lightstrip as detailed in SK9822 datasheet
{
    sendSPIbyte(0);
    sendSPIbyte(0);
    sendSPIbyte(0);
    sendSPIbyte(0);
}

void sendLS(uint8_t red, uint8_t green, uint8_t blue)//led frame format for lightstrip as detailed in SK9822 datasheet
{
    sendSPIbyte(224+BRIGHTNESS);
    sendSPIbyte(blue);
    sendSPIbyte(green);
    sendSPIbyte(red);
}

void clearLS(void)//turns all leds on lightstrip off
{
    sendLS_START();

    for(i = 0; i <= NUM_LEDS; i++)//Loop to cover all leds
    {
        sendSPIbyte(224);
        sendSPIbyte(0);
        sendSPIbyte(0);
        sendSPIbyte(0);
    }
}

void rpmtoLS(void)//converts timerA count value to the correct display of leds on lightstrip
{
    if(!rpmCaptureValue){//Error value would not work in calculation
        rpm = 0;
    }
    else{
        rpm = (uint16_t)((750000.0 * 60.0) / ((float)rpmCaptureValue * 8.0));//(48MHz / 64 clock divider * 60 seconds per minute) / (timerA count value * 8 pulses per revolution)
    }                                                                        //8 pulses per revolution is set in the ECU settings on the PE3 (tach pulses per rev in Engine Setup)

    ledsON = (rpm * NUM_LEDS) / (MAX_RPM - 1500);//Calculates # of leds to turn on based on calculated RPM

    clearLS();//Clear lightstrip

    if(ledsON > NUM_LEDS){//If RPM is in shift zone, flash red to tell driver to shift
        if(LSflash_flag)//if flash flag on
        {
            sendLS_START();//start frame
            for(i = 0; i < NUM_LEDS; i++)//turn on all leds to red
            {
                sendLS(255, 0, 0);
            }
        }
        else//if flash flag off
        {
            clearLS();//clear lightstrip
        }
    }
    else//if RPM is below shift zone, display as a metered indicator
    {
        sendLS_START();//start frame
        for (i = 0; i < ledsON; i++)//turn on correct # of leds based on RPM
        {
            sendLS(lightstrip[i][0], lightstrip[i][1], lightstrip[i][2]);//References array for colors of each section
        }
    }
}

void TA1_0_IRQHandler(void)//Interrupt every 4000 / 1200000 to flash shift lights
{
    TIMER_A1->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;//clear interrupt flag
    LSflash_flag = !LSflash_flag;//toggle flash flag
    TIMER_A1->CCR[0] += 4000;              // Add Offset to TACCR0
}

void TA0_0_IRQHandler(void)//Interrupt based on input from ECU tach signal
{
    rpmCaptureValue = TIMER_A0->CCR[0];//timer count value into accessible variable
    LSflag = 1;//set flag

    TIMER_A0->CCTL[0] &= ~(TIMER_A_CCTLN_CCIFG);// Clear the interrupt flag
    TIMER_A0->R = 0;//reset timer count value
}

void PORT6_IRQHandler(void)
{
    if(P6IFG & BIT0)//up paddle
    {
        P8OUT |=  BIT4;//up relay on
        P8OUT &= ~BIT5;//down relay off
    }
    else if(P6IFG & BIT1)//down paddle
    {
        P8OUT |=  BIT5;//down relay on
        P8OUT &= ~BIT4;//up relay off
    }
    else if(P6IFG & BIT4)//up hall effect
    {
        P8OUT &= ~0x30;//all relays off
        up_flag = 1;//Up shift indicated, update 7 segment
    }
    else if(P6IFG & BIT5)//down hall effect
    {
        P8OUT &= ~0x30;//all relays off
        down_flag = 1;//down shift indicated, update 7 segment
    }

    P6IFG &= ~0xFF;//clear interrupt flag
}
