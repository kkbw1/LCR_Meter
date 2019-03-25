/* ----------------------------------------------------------- */
// LEDRGB =     PF123 OUT
// SW1 =        PF4   IN
//
// DUT1 =       PE0 -> AIN3 IN
// DUT2 =       PE1 -> AIN2 IN
//              PC7 -> AC0  IN
//
// MEAS_LR =    PA6 OUT
// HIGHSIDE_R = PA7 OUT
// MEAS_C =     PC4 OUT
// LOWSIDE_R =  PC5 OUT
// INTEGRATE =  PC6 OUT
//
// SPI0Clk =    PA2 OUT
// SPI0Tx =     PA5 OUT
// CS =         PB3 OUT
// A0 =         PB6 OUT
// RES =        PB2 OUT
//
// COL123 =     PD123 IN
// ROW1234 =    PB45,PE45 OUT
//
// PORTA, PORTB, PORTC, PORTD, PORTE, PORTF
/* ----------------------------------------------------------- */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "tm4c123gh6pm.h"

#include "wait.h"
#include "GraphicsOLED.h"
#include "uart.h"

#define __DEBUG__
//==================================================================================================//
//#define GLCD_CS     // PB3
//#define GLCD_DC     // PB6
//#define GLCD_RES    // PB2
#define GLCD_CLK    (1<<2)// PA2, SSI0CLK
#define GLCD_MOSI   (1<<5)// PA5, SSI0TX

// PORTF
//#define USER_SW2        (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 0*4)))
#define RED_LED         (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define BLUE_LED        (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define GREEN_LED       (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define USER_SW1        (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))

// PA[7:6], PC[6:4]
#define MEAS_LR         (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 6*4)))   // PA6 OUT
#define HIGHSIDE_R      (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 7*4)))   // PA7 OUT
#define MEAS_C          (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 4*4)))   // PC4 OUT
#define LOWSIDE_R       (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 5*4)))   // PC5 OUT
#define INTEGRATE       (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4)))   // PC6 OUT

// PD[3:1], PB[5:4], PE[5:4]
#define COL1            (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 1*4)))   // PD1
#define COL2            (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 2*4)))   // PD2
#define COL3            (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 3*4)))   // PD3
#define ROW1            (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 4*4)))   // PB4
#define ROW2            (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 5*4)))   // PB5
#define ROW3            (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 4*4)))   // PE4                                                               0x400243FC
#define ROW4            (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 5*4)))   // PE5                                                               0x400243FC

// Define ADC Channel numbers for DUT1, DUT2
#define CH_ADC_DUT1     3
#define CH_ADC_DUT2     2

// Step5. Add Instructions Set Pin_name On/Off -> ex.) SET_MEAS_LR_ON
#define SET_MEAS_LR_ON()        MEAS_LR = 1
#define SET_MEAS_LR_OFF()       MEAS_LR = 0
#define SET_HIGHSIDE_R_ON()     HIGHSIDE_R = 1
#define SET_HIGHSIDE_R_OFF()    HIGHSIDE_R = 0
#define SET_MEAS_C_ON()         MEAS_C = 1
#define SET_MEAS_C_OFF()        MEAS_C = 0
#define SET_LOWSIDE_R_ON()      LOWSIDE_R = 1
#define SET_LOWSIDE_R_OFF()     LOWSIDE_R = 0
#define SET_INTEGRATE_ON()      INTEGRATE = 1
#define SET_INTEGRATE_OFF()     INTEGRATE = 0

#define MAX_CHARS       20
#define WT5_PERIOD      0.025   //usec
//==================================================================================================//
uint8_t flag_null = 0;

// for step1,2,3
char str[MAX_CHARS + 1];
uint8_t cnt_str = 0;
uint8_t idx_strEnd;
uint8_t cnt_arg;
uint8_t pos[MAX_CHARS];
char type[MAX_CHARS];
uint8_t cnt_parse;
char type_now, type_old;

char str_parse[5][10];

double voltage_const = 0.8056640625;

float R33 = 32.86;
double R100k = 99000.0;
double C1u = 0.00000085;

double R = 0;
double C = 0;
double L = 0;
double ESR = 0;

// for step8
uint32_t cnt_wt5 = 0;
uint16_t adc_dut2 = 0;

uint8_t kb_loop = 0;
uint8_t chk_kb;
//==================================================================================================//
// Initialize Hardware
void setSystemClock()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    //50Mhz = 24nsec per clk => 20*1.2 => 41.666667MHz
    //66.67Mhz = 18nsec per clk => 14.999*1.2 => 55.555556MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);
//    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (3 << SYSCTL_RCC_SYSDIV_S);
//    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (2 << SYSCTL_RCC_SYSDIV_S);
}

void calcUARTBAUD(uint8_t HSE, uint32_t sysclk, uint32_t UARTBAUD)
{
    double UBRD;
    if(HSE == 0)
        UBRD = (double)sysclk/16.0/(double)UARTBAUD;
    else
        UBRD = (double)sysclk/8.0/(double)UARTBAUD;

    UART0_IBRD_R = (uint16_t)UBRD;                                  // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = (uint16_t)((UBRD-(uint16_t)(UBRD))*64+0.5);      // round(fract(r)*64)=45
}

void initHw()
{
    setSystemClock();

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port A, B, C, F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOC
            | SYSCTL_RCGC2_GPIOD | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOF;
    //==================================================================================================//
    // Configure UART0 pins PA[1:0]
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;            // turn-on UART0, leave other UARTs in same status

    GPIO_PORTA_DEN_R |= 0x03;                           // 0b0000 0011, default, added for clarity
    GPIO_PORTA_AFSEL_R |= 0x03;                         // 0b0000 0011, default, added for clarity
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
//    UART0_CTL_R = 0;                                    // turn-off UART0 to allow safe programming
//    UART0_CC_R = UART_CC_CS_SYSCLK;                     // use system clock (40 MHz)
//    calcUARTBAUD(0, 40000000, 115200);
//    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;    // configure for 8N1 w/ 16-level FIFO
//    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

    UART0_CTL_R = 0;                                    // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                     // use system clock (40 MHz)
    calcUARTBAUD(0, 40000000, 115200);
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;    // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module
    UART0_IM_R = UART_IM_RXIM | UART_IM_RTIM;
    NVIC_EN0_R |= 1 << (INT_UART0-16);                  // turn-on interrupt 21 (UART0)
    //==================================================================================================//
    // Configure SSI0 and GLCD pins PA[5:2], PB6, PB[3:2]
    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R0;

    GPIO_PORTA_DIR_R |= (GLCD_CLK | GLCD_MOSI);     // PA2, PA5
    GPIO_PORTA_DR2R_R |= (GLCD_CLK | GLCD_MOSI);    // PA2, PA5
    GPIO_PORTA_AFSEL_R |= (0x0F << 2);              // PA[5:2]
    GPIO_PORTA_PCTL_R &= ~(GPIO_PCTL_PA2_M | GPIO_PCTL_PA3_M | GPIO_PCTL_PA4_M | GPIO_PCTL_PA5_M);
    GPIO_PORTA_PCTL_R |= (GPIO_PCTL_PA2_SSI0CLK | GPIO_PCTL_PA3_SSI0FSS | GPIO_PCTL_PA4_SSI0RX | GPIO_PCTL_PA5_SSI0TX);
    GPIO_PORTA_PUR_R |= (1<<2);                     // bit2 SPI Clk -> pull-up enabled
    GPIO_PORTA_CR_R &= ~(0x0F << 2);                // PA[5:2] Int disable
    GPIO_PORTA_DEN_R |= (0x0F << 2);                // PA[5:2] Enable

    GPIO_PORTB_DIR_R |= ((1<<2) | (1<<3) | (1<<6)); // CS, DC, RES output
    GPIO_PORTB_DR2R_R |= ((1<<2) | (1<<3) | (1<<6));// CS, DC, RES 2ma
    GPIO_PORTB_DEN_R |= ((1<<2) | (1<<3) | (1<<6)); // CS, DC, RES Enable

    // Configure the SSI0 as a SPI master, mode 3, 8bit operation, 1 MHz bit rate
    SSI0_CR1_R &= ~SSI_CR1_SSE;                      // turn off SSI0 to allow re-configuration
    SSI0_CR1_R = 0;                                  // select master mode
    SSI0_CC_R = 0;                                   // select system clock as the clock source
    SSI0_CPSR_R = 40;                                // set bit rate to 1 MHz (if SR=0 in CR0)
    SSI0_CR0_R = SSI_CR0_SPH | SSI_CR0_SPO | SSI_CR0_FRF_MOTO | SSI_CR0_DSS_8; // set SR=0, mode 3 (SPH=1, SPO=1), 8-bit
    SSI0_CR1_R |= SSI_CR1_SSE;                       // turn on SSI0
    //==================================================================================================//
    // Configure LED and pushbutton pins PF[0:4]
    GPIO_PORTF_DIR_R = 0x0E;  // bits 1,2,3 are outputs, bits 0,4 are inputs
    GPIO_PORTF_DR2R_R = 0x0E; // bits 1,2,3 set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_PUR_R = 0x10;  // bits 4 enable internal pull-up for push button
    GPIO_PORTF_DEN_R = 0x1E;  // bits 1,2,3,4 // enable LEDs and pushbuttons

    // Configure Control Pins PA[7:6], PC[6:4] to control MEAS_LR, MEAS_C, HIGHSIDE_R, LOWSIDE_R, INTEGRATE_R
    GPIO_PORTA_DIR_R |= ((1<<7) | (1<<6));
    GPIO_PORTA_DR2R_R |= ((1<<7) | (1<<6));
    GPIO_PORTA_DEN_R |= ((1<<7) | (1<<6));
    GPIO_PORTC_DIR_R |= ((1<<6) | (1<<5) | (1<<4));
    GPIO_PORTC_DR2R_R |= ((1<<6) | (1<<5) | (1<<4));
    GPIO_PORTC_DEN_R |= ((1<<6) | (1<<5) | (1<<4));

    // PE45, PB45, PD123 => PB45,PE45 -> KB Row Out, PD123 -> KB Col In GPIO Int
    GPIO_PORTB_DIR_R |= ((1<<4) | (1<<5));
    GPIO_PORTB_DR2R_R |= ((1<<4) | (1<<5));
    GPIO_PORTB_DEN_R |= ((1<<4) | (1<<5));
    GPIO_PORTE_DIR_R |= ((1<<4) | (1<<5));
    GPIO_PORTE_DR2R_R |= ((1<<4) | (1<<5));
    GPIO_PORTE_DEN_R |= ((1<<4) | (1<<5));

    GPIO_PORTD_DIR_R &= ~((1<<1) | (1<<2) | (1<<3));
    GPIO_PORTD_PDR_R |= ((1<<1) | (1<<2) | (1<<3));
    GPIO_PORTD_DEN_R |= ((1<<1) | (1<<2) | (1<<3));

    GPIO_PORTD_IM_R = 0;
    GPIO_PORTD_IS_R &= ~((1<<1) | (1<<2) | (1<<3));         // Int Sensitive, Edge or Level
    GPIO_PORTD_IBE_R &= ~((1<<1) | (1<<2) | (1<<3));        // Both Edge
    GPIO_PORTD_IEV_R |= ((1<<1) | (1<<2) | (1<<3));     // Falling or Rising
    GPIO_PORTD_RIS_R = 0;
    GPIO_PORTD_ICR_R |= ((1<<1) | (1<<2) | (1<<3));     // Int Clear flag
    GPIO_PORTD_IM_R |= ((1<<1) | (1<<2) | (1<<3));      // Int Mask Enable

    NVIC_EN0_R |= 1 << (INT_GPIOD-16);
    //==================================================================================================//
    // Configure ADC
    SYSCTL_RCGCADC_R |= 1;                           // 1. turn on ADC module 0 clocking
    __asm(" NOP");                                   // 1-1. waiting 3clocks after setting ADC Clock Enable
    __asm(" NOP");
    __asm(" NOP");
                                                     // 2. GPIO Clock already enabled
    GPIO_PORTE_AFSEL_R |= 0x03;                      // 3. select alternative functions for AIN3, AIN2 (PE0, PE1)
    GPIO_PORTE_DEN_R &= ~0x03;                       // 4. turn off digital operation on pin (PE0, PE1)
    GPIO_PORTE_AMSEL_R |= 0x03;                      // 5. turn on analog operation on pin (PE0, PE1)
    ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // 6. select PLL as the time base (not needed, since default value)
    // ADC0 Sample Sequencer3 Configuration
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // 1. disable sample sequencer 3 (SS3) for programming
    ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // 2. single trigger, set EM3 bits in ADCPSSI as trigger(default)
    ADC0_TSSEL_R = 0;                                // 3. (not required)Choose which PWM module the generator is located.
    ADC0_SSMUX3_R = 0;                               // 4. set first sample to AN0 (Single Sampling)
//    ADC0_SSMUX3_R = 1;                               // 4. set first sample to AIN2-AIN3 (2*1, 2*1+1) (Differential Smapling)
    ADC0_SSCTL3_R = ADC_SSCTL3_END0;                 // 5. mark first sample as the end, sample single input select
//    ADC0_SSCTL3_R = ADC_SSCTL3_END0 | ADC_SSCTL3_D0; // 5. mark first sample as the end, sample differential input select
    ADC0_IM_R = 0;                                   // 6. Interrupt Disable
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                 // 7. enable sample sequencer 3 (SS3) to operate
    //==================================================================================================//
    // Configure AD Comparator
    SYSCTL_RCGCACMP_R |= 1;                                         // 1. AD Comparator0 Clock Enable
    __asm(" NOP");                                                  // 1-1. waiting 3clocks after setting ADComp Clock Enable before AD Comp registers are programmed
    __asm(" NOP");                                                  // not required in this initialization
    __asm(" NOP");
                                                                    // 2. already set GPIO Clock Enable
    GPIO_PORTC_AFSEL_R |= (1<<7);                                   // 3. C0- => PC7
    GPIO_PORTC_AMSEL_R |= (1<<7);                                   // 3. C0- => PC7
    GPIO_PORTC_PCTL_R &= ~(0xF0000000);                             // 4. PC7, PMC7 = 0
    COMP_ACREFCTL_R = COMP_ACREFCTL_EN | 0x0F;                      // 5. AC IRef Vol Enable, RNG=0, Viref=2.469V
    COMP_ACCTL0_R = COMP_ACCTL0_ASRCP_REF | COMP_ACCTL0_ISEN_FALL;  // 6. VIN+=Viref, Interrupt Sense=Falling Edge
//    COMP_ACINTEN_R = COMP_ACINTEN_IN0;                              // 6. Comparator 0 Int Enable
//    NVIC_EN0_R |= 1 << (INT_COMP0-16);                              // 6. turn-on interruptVector 41, Int Num = 25
    waitMicrosecond(20);                                            // 7. waiting 20us before reading ACSTATn
    //==================================================================================================//
    // Configure 32/64-bit Wide Timer 5
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R5;                    //1. turn-on timer5 clock

    WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;                               //2. turn-off counter before reconfiguring
    WTIMER5_CFG_R = 4;                                              //3. configure as 32-bit counter (A only)
    WTIMER5_TAMR_R = TIMER_TAMR_TAMR_1_SHOT | TIMER_TAMR_TACDIR;    //4. configure for one shot mode, count up
    WTIMER5_CTL_R = 0;                                              //5. turn-off Timer mode
    WTIMER5_IMR_R = 0;                                              //6. turn-off interrupts
    WTIMER5_TAV_R = 0;                                              //7. CNT value = 0, 1clk period = 1/40Mhz = 25ns
//    WTIMER5_CTL_R |= TIMER_CTL_TAEN;                                //8. turn-on counter

    NVIC_EN3_R &= ~(1 << (INT_WTIMER5A-16-96));                     //9. turn-off interrupt 120 (WTIMER5A)
    //==================================================================================================//
    // Configure Timer 1 as the time base
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER1_TAILR_R = 2000000;                         // set load value to period 50ms
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
}

uint16_t readADC0SS3_Single(uint8_t ch)
{
    // ADC0 Sample Sequencer3 Configuration
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // 1. disable sample sequencer 3 (SS3) for programming
    ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // 2. single trigger, set EM3 bits in ADCPSSI as trigger(default)
    ADC0_TSSEL_R = 0;                                // 3. (not required)Choose which PWM module the generator is located.
    ADC0_SSCTL3_R = ADC_SSCTL3_END0;                 // 5. mark first sample as the end, sample single input select
    ADC0_IM_R = 0;                                   // 6. Interrupt Disable
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                 // 7. enable sample sequencer 3 (SS3) to operate

    ADC0_SSMUX3_R = ch;
    __asm(" NOP");
    __asm(" NOP");
    __asm(" NOP");
    ADC0_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    return ADC0_SSFIFO3_R;                           // get single result from the FIFO
}

uint16_t readADC0SS3_Diff(uint8_t ch)
{
    // ADC0 Sample Sequencer3 Configuration
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // 1. disable sample sequencer 3 (SS3) for programming
    ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // 2. single trigger, set EM3 bits in ADCPSSI as trigger(default)
    ADC0_TSSEL_R = 0;                                // 3. (not required)Choose which PWM module the generator is located.
    ADC0_SSCTL3_R = ADC_SSCTL3_END0 | ADC_SSCTL3_D0; // 5. mark first sample as the end, sample differential input select
    ADC0_IM_R = 0;                                   // 6. Interrupt Disable
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                 // 7. enable sample sequencer 3 (SS3) to operate

    ADC0_SSMUX3_R = ch;
    __asm(" NOP");
    __asm(" NOP");
    __asm(" NOP");
    ADC0_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    return ADC0_SSFIFO3_R;                           // get single result from the FIFO
}

void pause()
{
    putsUart0("Press 'q' to clear\r\n");
    writeStrGraphicsLcd(0, 7, "Press 'q' to clear");

    chk_kb = 0;
    while(getcUart0() != 'q') // waiting until input 'q' is received
    {
        if((chk_kb&0x70) != 0)
            break;
    }

    NVIC_EN0_R |= 1 << (INT_UART0-16);                  // turn-on interrupt 21 (UART0)
}

void parsingString()
{
    //--------------------------------------------------------------//
    // Step 3.
    // Parse Strings
    //--------------------------------------------------------------//
    memset(pos, 0, MAX_CHARS*sizeof(pos[0]));           // Make all data in the array pos to NULL
    memset(type, 0, MAX_CHARS*sizeof(type[0]));         // Make all data in the array type to NULL
    memset(str_parse, 0, 50*sizeof(str_parse[0][0]));

    type_now = 0;
    type_old = 0;
    cnt_arg = 0;

    uint8_t i, j = 0;
    for(i = 0; i < idx_strEnd; i++)
    {
        type_old = type_now;
        if((str[i] >= 0x41 && str[i] <= 0x5A) || (str[i] >= 0x61 && str[i] <= 0x7A))    // Upper letters and lower letters
        {
            type_now = 'c';
            if((type_now != type_old) || (i == 0))
            {
                pos[cnt_arg] = i;
                type[cnt_arg] = 'c';
                cnt_arg++;
            }
        }
        else if(str[i] >= 0x30 && str[i] <= 0x39)   // Numbers
        {
            type_now = 'n';
            if((type_now != type_old) || (i == 0))
            {
                pos[cnt_arg] = i;
                type[cnt_arg] = 'n';
                cnt_arg++;
            }
        }
        else        // everything else -> delimiters
        {
            type_now = 'd';
            str[i] = 0;     // change it NULL
        }
    }

    for(i = 0; i < cnt_arg; i++)
    {
        uint8_t pos_now = pos[i];
        if(i == (cnt_arg - 1))     // if it's a last arg
        {
            while(str[pos_now] != 0)
            {
                str_parse[i][pos_now - pos[i]] = str[pos_now];
                pos_now++;
            }
        }
        else
        {
            while(pos_now < pos[i+1] && (str[pos_now] != 0))
            {
                str_parse[i][pos_now - pos[i]] = str[pos_now];
                pos_now++;
            }
        }
    }
    //----------------------- Display Step3 -----------------------//
    for(i = 0; i < cnt_arg; i++)
    {
        for(j = 0; j < 15; j++)
        {
            if(str_parse[i][j] == 0)
                break;
            writeCharGraphicsLcd(j, i+1, str_parse[i][j]);
        }
    }
    putsUart0(&str[pos[0]]);

    writeStrGraphicsLcd(0, 7, "Press Enter.");
    putsUart0("\r\nPress Enter to continue.\r\n");
    while(getcUart0() != 13){};    // waiting until input C/R is received

    for(i = 1; i < 8; i++)          // Clear display except page0
        writeStrGraphicsLcd(0, i, "                     ");
}

uint8_t isCommand(char* str, uint8_t min_args)
{
    uint8_t i;
    uint8_t cnt_pos;

    if(type[0] != 'c')              // if first str is not char type -> it's not valid command
        return 0;

    if(cnt_arg < min_args)          // if number of args < minimum number of args -> it's not valid command
        return 0;

    for(i = 0; i < cnt_arg; i++)    // changing every letters to lower letters
    {
        cnt_pos = 0;
        while(str_parse[i][cnt_pos]!=0)
        {
            if(str_parse[i][cnt_pos] >= 65 && str_parse[i][cnt_pos] <= 90)     // if letters are upper letters
                str_parse[i][cnt_pos] += 32;   // make it lower letters
            cnt_pos++;
        }
    }

    if(strcmp(&str_parse[0][0], "set") == 0)
        return 1;
    else if(strcmp(&str_parse[0][0], "reset") == 0)
        return 1;
    else if(strcmp(&str_parse[0][0], "voltage") == 0)
        return 1;
    else if(strcmp(&str_parse[0][0], "resistor") == 0)
        return 1;
    else if(strcmp(&str_parse[0][0], "capacitance") == 0)
        return 1;
    else if(strcmp(&str_parse[0][0], "inductance") == 0)
        return 1;
    else if(strcmp(&str_parse[0][0], "esr") == 0)
        return 1;
    else if(strcmp(&str_parse[0][0], "auto") == 0)
        return 1;
#ifdef __DEBUG__
    else if(strcmp(&str_parse[0][0], "dut") == 0)
        return 1;
#endif
    else
        return 0;
//    return true;
}

void software_reset()
{
    putsUart0("Reset MCU in 2sec\r\n");
    writeStrGraphicsLcd(0, 2, "Reset MCU in 2sec");
    waitMicrosecond(2000000);

    NVIC_APINT_R = (0x05FA << 16) | NVIC_APINT_SYSRESETREQ;   // Reset Microcontroller
}

void measure_voltage()
{
    uint8_t cnt_adc = 0;
    uint16_t sum_rawADC = 0;
    float avg_rawADC = 0;

    for(cnt_adc = 0; cnt_adc < 10; cnt_adc++)
    {
        sum_rawADC += readADC0SS3_Diff(1);
    }
    avg_rawADC = sum_rawADC / 10.0;

    int16_t mV = 1.60234 * avg_rawADC - 3281.58;
    if(mV < 0)
        mV = 0;

    char str_result[10];

#ifdef __DEBUG__
    sprintf(str_result, "%d", (int)avg_rawADC);
    writeStrGraphicsLcd(0, 2, str_result);
    putsUart0(str_result);
    putsUart0("\r\n");
#endif

    sprintf(str_result, "%d", mV);
    writeStrGraphicsLcd(0, 3, str_result);
    writeStrGraphicsLcd(strlen(str_result), 3, "mV");
    putsUart0(str_result);
    putsUart0("mV\r\n");
}

double getResFromRC(float t, float Vout, float Vin, float C)
{
    double num = -t;
    double den = log((double)(1.0) - ((double)(Vout)/(double)(Vin))) * (double)(C);

    return num/den;
}

uint8_t measure_resistor()
{
    // Rx-C1u
    uint8_t flag_timeout = 0;

    uint8_t cnt_measR = 0;

    uint32_t sum_cnt_wt5 = 0;

    uint16_t adc_dut1 = 0;
    uint16_t sum_adc_dut1 = 0;
    uint16_t sum_adc_dut2 = 0;

    float Vin = 3.27;
    float Vout = 2.469;

    // Initialize Pins
    SET_MEAS_LR_OFF();
    SET_MEAS_C_OFF();
    SET_HIGHSIDE_R_OFF();
    SET_LOWSIDE_R_OFF();
    SET_INTEGRATE_ON();

    for(cnt_measR = 0; cnt_measR < 5; cnt_measR++)  // measuring 5 times to get an average value
    {
        // Discharging C    (when V0 = 3.27v, time_discharging = 0.2336ms by Vc = 0.001v)
        SET_LOWSIDE_R_ON();
        waitMicrosecond(500000);    // waiting 500ms for discharging completely
        SET_LOWSIDE_R_OFF();        // End discharging

        // Charging C
        // Enable AC0 Int
        COMP_ACINTEN_R = COMP_ACINTEN_IN0;      // 6. Comparator 0 Int Enable
        NVIC_EN0_R |= 1 << (INT_COMP0-16);      // 6. turn-on interruptVector 41, Int Num = 25
        // Enable WT5 Counter
        WTIMER5_TAV_R = 0;                      // initializing counter value
        WTIMER5_CTL_R = 1;                      // turn-on the the counter
        // Start charging
        SET_MEAS_LR_ON();

        // AC0 Interrupt is finished before break this while loop
        while(1)                                // waiting for measuring time or time out
        {
            if(WTIMER5_CTL_R == 0)              // if measuring time finished
            {
                break;
            }
            else if(WTIMER5_TAV_R > 400000000)  // if Counter > 400M(=10sec)
            {
                flag_timeout = 1;
                break;
            }
        }

        // Disable AC0 Int, WT5 Counter
        COMP_ACINTEN_R = 0;                     // Comparator 0 Int Disable
        NVIC_EN0_R &= ~(1 << (INT_COMP0-16));   // turn off interruptVector 41, Int Num = 25
        WTIMER5_CTL_R = 0;

        if(flag_timeout == 1)
        {
            SET_MEAS_LR_OFF();                  // Stopping charging C

            break;
        }
        else
        {
            // Get V_dut1
            adc_dut1 = readADC0SS3_Single(CH_ADC_DUT1);

            sum_cnt_wt5 += cnt_wt5;
            sum_adc_dut1 += adc_dut1;
            sum_adc_dut2 += adc_dut2;

            SET_MEAS_LR_OFF();                  // Stopping charging C
        }
    }

    if(flag_timeout == 0)
    {
        // Calculating Resistance
//        double time_offset = (800*WT5_PERIOD) / 1000000.0;
//        double time_R = ((double)(sum_cnt_wt5/5.0) * WT5_PERIOD) / 1000000 - time_offset;
//        if(time_R < 0)
//            time_R = 0;

        double time_R = ((double)(sum_cnt_wt5/5.0) * WT5_PERIOD) / 1000000.0;

        Vin = 3.27;
//        Vin = ((sum_adc_dut1/5.0)*voltage_const)/1000.0;
//        Vout = ((sum_adc_dut2/5.0)*0.8018342)/1000.0;

        R = (getResFromRC(time_R, Vout, Vin, C1u) - 16.91) * 1.05;
        if(R < 0)
            R = 0;

        // Display Data
        char str_result[15];

#ifdef __DEBUG__
        sprintf(str_result, "%lu", cnt_wt5);
        putsUart0(str_result);
        putsUart0("cnt\r\n");
        writeStrGraphicsLcd(0, 2, str_result);
        writeStrGraphicsLcd(strlen(str_result), 2, "count");

        sprintf(str_result, "%.2f", (float)(cnt_wt5)/40.0);
        putsUart0(str_result);
        putsUart0("us\r\n");
        writeStrGraphicsLcd(0, 3, str_result);
        writeStrGraphicsLcd(strlen(str_result), 3, "us");
#endif

        sprintf(str_result, "%.2f", R);
        putsUart0(str_result);
        putsUart0("ohm\r\n");
        writeStrGraphicsLcd(0, 4, str_result);
        writeStrGraphicsLcd(strlen(str_result), 4, "ohm");

//        sprintf(str_result, "%1.3f", Vout);
//        writeStrGraphicsLcd(0, 4, str_result);

        return 1;
    }
    else
    {
        R = 0;

        writeStrGraphicsLcd(0, 2, "Time out failed.");
        putsUart0("Measuring resistance time out failed.\r\n");

        return 0;
    }
}

double getCapFromRC(double t, float Vout, float Vin, float Vrem, float R)
{
    double num = -t;
    double den = log((double)(1.0) - ((double)(Vout-Vrem)/(double)(Vin-Vrem))) * (double)(R);

    return num/den;
}

uint8_t measure_capacitance()
{
    // Cx-R100k
    uint8_t flag_timeout = 0;

    uint8_t cnt_measC = 0;
    uint32_t sum_cnt_wt5 = 0;

    uint16_t sum_adc_dut2 = 0;

    float Vin = 3.27;
    float Vout = 2.469;

    // Initialization
    SET_MEAS_LR_OFF();
    SET_MEAS_C_ON();
    SET_HIGHSIDE_R_OFF();
    SET_LOWSIDE_R_OFF();
    SET_INTEGRATE_OFF();

    for(cnt_measC = 0; cnt_measC < 3; cnt_measC++)  // measuring 5 times to get an average value
    {
        // Discharging C    (when V0 = 3.27v, C = 1mF -> time_discharging = 0.2336s by Vc = 0.001v)
        SET_LOWSIDE_R_ON();         // Strart discharging
        waitMicrosecond(500000);   // waiting 500ms for discharging completely
        SET_LOWSIDE_R_OFF();        // End discharging
        waitMicrosecond(100);

        // Charging C
        // Enable AC0 Int
        COMP_ACINTEN_R = COMP_ACINTEN_IN0;      // Comparator 0 Int Enable
        NVIC_EN0_R |= 1 << (INT_COMP0-16);      // turn-on interruptVector 41, Int Num = 25
        // Enable WT5 Counter
        WTIMER5_TAV_R = 0;                      // initializing counter value
        WTIMER5_CTL_R = 1;                      // turn-on the the counter
        // Start charging
        SET_HIGHSIDE_R_ON();

        // AC0 Interrupt is finished before break this while loop
        while(1)                                // waiting for measuring time or time out
        {
            if(WTIMER5_CTL_R == 0)              // if measuring time finished
            {
                break;
            }
            else if(WTIMER5_TAV_R > 700000000)  // if Counter > 300M(=7.5sec) (600M=>15sec)
            {
                flag_timeout = 1;
                break;
            }
        }

        // Disable AC0 Int, WT5 Counter
        COMP_ACINTEN_R = 0;                     // Comparator 0 Int Disable
        NVIC_EN0_R &= ~(1 << (INT_COMP0-16));   // turn off interruptVector 41, Int Num = 25
        WTIMER5_CTL_R = 0;

        if(flag_timeout == 1)
        {
            SET_HIGHSIDE_R_OFF();               // Stopping charging C

            break;
        }
        else
        {
            sum_cnt_wt5 += cnt_wt5;
            sum_adc_dut2 += adc_dut2;

            SET_HIGHSIDE_R_OFF();               // Stopping charging C
        }
    }

    if(flag_timeout == 0)
    {
        // Calculating Resistance
        double time_offset = 0.0 / 1000000;
        double time_C = ((double)(sum_cnt_wt5/3.0) * WT5_PERIOD) / 1000000.0 - time_offset;
        if(time_C < 0)
            time_C = 0;

        C = getCapFromRC(time_C, Vout, Vin, 0, R100k);

        // Display Data
        char str_result[15];

#ifdef __DEBUG__
        sprintf(str_result, "%lu", sum_cnt_wt5/3);
        putsUart0(str_result);
        putsUart0("cnt\r\n");
        writeStrGraphicsLcd(0, 2, str_result);
        writeStrGraphicsLcd(strlen(str_result), 2, "count");

        sprintf(str_result, "%.2f", (double)(cnt_wt5)/40.0);
        putsUart0(str_result);
        putsUart0("us\r\n");
        writeStrGraphicsLcd(0, 3, str_result);
        writeStrGraphicsLcd(strlen(str_result), 3, "us");
#endif

        sprintf(str_result, "%.6f", (C*1000000.0));
        putsUart0(str_result);
        putsUart0("uF\r\n");
        writeStrGraphicsLcd(0, 4, str_result);
        writeStrGraphicsLcd(strlen(str_result), 4, "uF");

#ifdef __DEBUG__
        sprintf(str_result, "%1.3fV", Vout);
        writeStrGraphicsLcd(0, 5, str_result);
#endif

        return 1;
    }
    else
    {
        C = 0;

        writeStrGraphicsLcd(0, 2, "Time out failed.");
        putsUart0("Measuring capacitance time out failed.\r\n");

        return 0;
    }
}

double calc_Vout_offset(double Vout)
{
    double a = 0.003136;
    double b = 1.906;
    double c = -6759.0;
    double d = -8.04;

    return a*exp(b*Vout) + c*exp(d*Vout);
}

double calc_Rdc_gain(double Rdc)
{
    double a = -15.68;
    double b = -1.398;
    double c = 1.057;

    return a*pow(Rdc, b)+c;
}

float getR1fromVoltageDivider(float Vin, float Vout, float R2)
{
    return R2*(Vin-Vout)/Vout;
}

float set_AC0_Viref(float Vthr)
{
    float Viref_gap = 0.1122;
    float Viref = 0;
    uint8_t bitsVREF = 0;

    if((Vthr-0.2-0.786) <= 0)
        return 0;

    COMP_ACREFCTL_R &= ~(COMP_ACREFCTL_EN | 0x000F);
    bitsVREF = (uint8_t)((Vthr-0.2-0.786) / Viref_gap);
    if(bitsVREF > 16)
        bitsVREF = 16;

    Viref = bitsVREF * Viref_gap + 0.786;
    COMP_ACREFCTL_R = COMP_ACREFCTL_EN | bitsVREF;      // AC IRef Vol Enable, RNG=0, Viref=2.469V

    return Viref;
}

double calc_time_gain(double t_usec)
{
    double a = -1.238;
    double b = -3.736;
    double c = 1.055;

    return (a*pow(t_usec, b)+c);
}

double getInductanceFromLR(float t, float Vout, float Vin, float R, float Rdc)
{
    // Rdc = DC Resistance of an inductor
    double num = -t*(R + Rdc);
    double den = log((double)(1.0) - (double)(Vout)*(double)(R+Rdc)/(double)(Vin)/(double)R);

    return num/den;
}

uint8_t measure_inductance()
{
    // Lx-R33
    uint8_t flag_timeout = 0;

    uint8_t cnt_measL = 0;
    uint32_t sum_cnt_wt5 = 0;

    uint16_t sum_adc2 = 0;
    float avg_adc2;

    float Vin = 3.27;
    float Vout = 2.469;

    float Vdut2 = 0;
    double Vdut2_real = 0;

    double Rdc = 0;
    double Rdc_cali = 0;

    // Initialization
    SET_MEAS_LR_OFF();
    SET_MEAS_C_OFF();
    SET_HIGHSIDE_R_OFF();
    SET_LOWSIDE_R_OFF();
    SET_INTEGRATE_OFF();

    // Getting R_dc for Inductor
    SET_LOWSIDE_R_ON();
    SET_MEAS_LR_ON();
    waitMicrosecond(1000000);                                                   // waiting enough L fully charged

    uint8_t i;
    for(i = 0; i < 5; i++)
    {
        sum_adc2 += readADC0SS3_Single(CH_ADC_DUT2);
    }
    avg_adc2 = sum_adc2 / 5.0;

    Vdut2 = (float)((avg_adc2*voltage_const) / 1000.0);
    Vdut2_real = Vdut2 + calc_Vout_offset(Vdut2);
    if(Vdut2_real > Vin)
        Vdut2_real = Vin;

    Rdc = getR1fromVoltageDivider(Vin, Vdut2, R33);                             // Get R1 from the voltage divider
    Rdc_cali = Rdc * calc_Rdc_gain(Rdc);
    if(Rdc_cali < 0)
        Rdc_cali = 0.0;

    Vout = set_AC0_Viref(Vdut2);                                                // Change Viref to be called AC0_EDGE_ISR

    // Trying to measuring Inductnace
    for(cnt_measL = 0; cnt_measL < 5; cnt_measL++)  // measuring 5 times to get an average value
    {
        // Discharging L            (when V0 = 3.27v, time_discharging = 26.6ms by Vc = 0.0005v)
        SET_MEAS_LR_OFF();          // Start discharging
        waitMicrosecond(300000);    // waiting 500ms for discharging completely

        // Charging L
        // Enable AC0 Int
        COMP_ACINTEN_R = COMP_ACINTEN_IN0;      // Comparator 0 Int Enable
        NVIC_EN0_R |= 1 << (INT_COMP0-16);      // turn-on interruptVector 41, Int Num = 25
        waitMicrosecond(1000);

        // Enable WT5 Counter
        WTIMER5_TAV_R = 0;                      // initializing counter value
        WTIMER5_CTL_R = 1;                      // turn-on the the counter
        SET_MEAS_LR_ON();                       // Start charging

        // AC0 Interrupt is finished before break this while loop
        while(1)                                // waiting for measuring time or time out
        {
            if(WTIMER5_CTL_R == 0)              // if measuring time finished
            {
                break;
            }
            else if(WTIMER5_TAV_R > 200000000)  // if Counter > 200M(=5sec)
            {
                flag_timeout = 1;
                break;
            }
        }

        // Disable AC0 Int, WT5 Counter
        COMP_ACINTEN_R = 0;                     // Comparator 0 Int Disable
        NVIC_EN0_R &= ~(1 << (INT_COMP0-16));   // turn off interruptVector 41, Int Num = 25
        WTIMER5_CTL_R = 0;

        if(flag_timeout == 1)   // when counter value is relatively too high
        {
            SET_MEAS_LR_OFF();                  // Stopping charging C

            break;
        }
        else                    // when AC0 Int occured and counter value is measured
        {
            sum_cnt_wt5 += cnt_wt5;

            SET_MEAS_LR_OFF();                  // Stopping charging C
        }
    }

    // Calculating Inductance
    if(flag_timeout == 0)
    {
        // Calcuating time
        double mean_cnt_wt5 = (double)(sum_cnt_wt5/5.0);
        double time_L = (mean_cnt_wt5 * WT5_PERIOD) / 1000000.0;
//        time_L += calc_time_offset(mean_cnt_wt5);
        double time_L_cali = time_L * calc_time_gain(time_L * 1000000.0);
        if(time_L_cali < 0)
            time_L_cali = 0;

        // Calc L
        L = getInductanceFromLR(time_L_cali, Vout, Vin, R33, Rdc_cali);
//        L = getInductanceFromLR(time_L_cali, Vout, Vin, R33, 0.0);

        // Reset Viref to 2.469V
        COMP_ACREFCTL_R = COMP_ACREFCTL_EN | 0x000F;      // AC IRef Vol Enable, RNG=0, Viref=2.469V

        // Display Data
        char str_result[15];

#ifdef __DEBUG__
//        sprintf(str_result, "%.3f", Rdc);
//        putsUart0(str_result);
//        putsUart0("ohm\r\n");
//        writeStrGraphicsLcd(0, 2, str_result);
//        writeStrGraphicsLcd(strlen(str_result), 2, "ohm");

        sprintf(str_result, "%.2f", Rdc_cali);
        putsUart0(str_result);
        putsUart0("ohm\r\n");
        writeStrGraphicsLcd(0, 2, str_result);
        writeStrGraphicsLcd(strlen(str_result), 2, "ohm");

        sprintf(str_result, "%lu", cnt_wt5);
        putsUart0(str_result);
        putsUart0("cnt\r\n");
        writeStrGraphicsLcd(0, 3, str_result);
        writeStrGraphicsLcd(strlen(str_result), 3, "cnt");

        sprintf(str_result, "%5.3f", time_L*1000000.0);
        putsUart0(str_result);
        putsUart0("us\r\n");
        writeStrGraphicsLcd(0, 4, str_result);
        writeStrGraphicsLcd(strlen(str_result), 4, "us");
#endif

        sprintf(str_result, "%.3f", L*1000000);
        putsUart0(str_result);
        putsUart0("uH\r\n");
        writeStrGraphicsLcd(0, 5, str_result);
        writeStrGraphicsLcd(strlen(str_result), 5, "uH");

//#ifdef __DEBUG__
//        sprintf(str_result, "%1.3f", Vdut2);
//        putsUart0(str_result);
//        putsUart0("V\r\n");
//        writeStrGraphicsLcd(0, 6, str_result);
//        writeStrGraphicsLcd(strlen(str_result), 6, "V");
//
//        sprintf(str_result, "%1.3f", Vout);
//        putsUart0(str_result);
//        putsUart0("V\r\n");
//        writeStrGraphicsLcd(8, 6, str_result);
//        writeStrGraphicsLcd(8+strlen(str_result), 6, "V");
//#endif

        return 1;
    }
    else
    {
        L = 0;

        writeStrGraphicsLcd(0, 2, "Time out failed.");
        putsUart0("Measuring Inductance time out failed.\r\n");

        return 0;
    }
}

void measure_esr()
{
    float Vin = 3.27;

    uint8_t cnt_measESR;
    uint16_t sum_adc2 = 0;
    float avg_adc2;

    float Vdut2;
    float Rdc;

    // Initialization
    SET_MEAS_LR_OFF();
    SET_MEAS_C_OFF();
    SET_HIGHSIDE_R_OFF();
    SET_LOWSIDE_R_OFF();
    SET_INTEGRATE_OFF();

    // Getting R_dc for Inductor
    SET_LOWSIDE_R_ON();
    SET_MEAS_LR_ON();
    waitMicrosecond(1000000);                                                   // waiting enough L fully charged

    for(cnt_measESR = 0; cnt_measESR < 5; cnt_measESR++)
    {
        sum_adc2 += readADC0SS3_Single(CH_ADC_DUT2);
    }

    avg_adc2 = sum_adc2 / 5.0;
    Vdut2 = (float)((avg_adc2*voltage_const) / 1000.0);

    Rdc = getR1fromVoltageDivider(Vin, Vdut2, R33);                             // Get R1 from the voltage divider
    Rdc = Rdc * calc_Rdc_gain(Rdc);
    if(Rdc < 0)
        Rdc = 0;

    ESR = Rdc;

    // Display Data
    char str_result[15];

#ifdef __DEBUG__
    sprintf(str_result, "%.2f", avg_adc2);
    putsUart0(str_result);
    putsUart0("\r\n");

    sprintf(str_result, "%.3f", Vdut2);
    putsUart0(str_result);
    putsUart0("V\r\n");
    writeStrGraphicsLcd(0, 2, str_result);
    writeStrGraphicsLcd(strlen(str_result), 2, "V");
#endif

    sprintf(str_result, "%.3f", Rdc);
    putsUart0(str_result);
    putsUart0("ohm\r\n");
    writeStrGraphicsLcd(0, 3, str_result);
    writeStrGraphicsLcd(strlen(str_result), 3, "ohm");
}

void mode_auto()
{
    uint8_t i;

    uint8_t check_auto[3];
    uint8_t flag_check = 0;     // 1 = R, 2 = C, 3 = L

    writeStrGraphicsLcd(0, 2, "Checking Res");
    check_auto[0] = measure_resistor();
    for(i = 2; i < 8; i++)          // Clear display except page0
        writeStrGraphicsLcd(0, i, "                     ");

    writeStrGraphicsLcd(0, 2, "Checking Cap");
    check_auto[1] = measure_capacitance();
    for(i = 2; i < 8; i++)          // Clear display except page0
        writeStrGraphicsLcd(0, i, "                     ");

    writeStrGraphicsLcd(0, 2, "Checking Ind");
    check_auto[2] = measure_inductance();
    for(i = 2; i < 8; i++)          // Clear display except page0
        writeStrGraphicsLcd(0, i, "                     ");

#ifdef __DEBUG__
    for(i = 0; i < 3; i++)
    {
        if(check_auto[i] == 1)
            putsUart0("T");
        else if(check_auto[i] == 0)
            putsUart0("F");
        else
            putsUart0("E");
    }
    putsUart0("\r\n");
#endif

    if(check_auto[0] == 1 && check_auto[1] == 0 && check_auto[2] == 0)
        flag_check = 1;
    if(check_auto[0] == 0 && check_auto[1] == 1 && check_auto[2] == 0)
        flag_check = 2;
    if(check_auto[0] == 0 && check_auto[1] == 0 && check_auto[2] == 1)
        flag_check = 3;

    if(check_auto[0] == 1 && check_auto[1] == 1 && check_auto[2] == 0)
    {
        if(R <= 0.1)
            flag_check = 2;

        if(R >= 100000.0)
            flag_check = 1;
    }
    if(check_auto[0] == 1 && check_auto[1] == 0 && check_auto[2] == 1)
    {
        if(L > (0.2/1000000.0))
        {
            measure_esr();
            for(i = 2; i < 8; i++)          // Clear display except page0
                writeStrGraphicsLcd(0, i, "                     ");

            flag_check = 3;
        }
        if((L < (0.2/1000000.0)) && (R > 8.0))
            flag_check = 1;
    }

    char str_result[15];
    if(flag_check == 1)
    {
        sprintf(str_result, "%.1fohm", R);
        writeStrGraphicsLcd(0, 2, "Resistor Detected");
        writeStrGraphicsLcd(0, 3, str_result);
        putsUart0("Resistor Detected\r\n");
        putsUart0(str_result);
        putsUart0("\r\n");
    }
    if(flag_check == 2)
    {
        sprintf(str_result, "%.6fuF", C*1000000.0);
        writeStrGraphicsLcd(0, 2, "Capacitor Detected");
        writeStrGraphicsLcd(0, 3, str_result);
        putsUart0("Capacitor Detected\r\n");
        putsUart0(str_result);
        putsUart0("\r\n");
    }
    if(flag_check == 3)
    {
        sprintf(str_result, "%.2fuH", L*1000000.0);
        writeStrGraphicsLcd(0, 2, "Inductor Detected");
        writeStrGraphicsLcd(0, 3, str_result);
        putsUart0("Inductor Detected\r\n");
        putsUart0(str_result);
        putsUart0("\r\n");

        sprintf(str_result, "%.1fohm", ESR);
        writeStrGraphicsLcd(0, 4, str_result);
        putsUart0("ESR: ");
        putsUart0(str_result);
        putsUart0("\r\n");
    }
    if(flag_check == 0)
    {
        putsUart0("Unable to detect the component");
        writeStrGraphicsLcd(0, 2, "Failed Auto mode.");
    }
}

void get_each_volt_dut12()
{
    uint32_t sum_adc1 = 0;
    uint32_t sum_adc2 = 0;

    double Vdut1;
    double Vdut2;

    uint8_t i;
    for(i = 0; i < 10; i++)
    {
        sum_adc1 += readADC0SS3_Single(CH_ADC_DUT1);
        waitMicrosecond(10000);
        sum_adc2 += readADC0SS3_Single(CH_ADC_DUT2);
        waitMicrosecond(10000);
    }

    Vdut1 = (sum_adc1 / 10.0) * voltage_const;
    Vdut2 = (sum_adc2 / 10.0) * voltage_const;

    double dVduts = Vdut1 - Vdut2;
    double RL = dVduts * 32.86 / Vdut2;

    char str_dut[15];
    sprintf(str_dut, "D1:%4.1fmV", Vdut1);
    writeStrGraphicsLcd(0, 2, str_dut);
    putsUart0(str_dut);
    putsUart0("\r\n");
    sprintf(str_dut, "D2:%4.1fmV", Vdut2);
    writeStrGraphicsLcd(0, 3, str_dut);
    putsUart0(str_dut);
    putsUart0("\r\n");
    sprintf(str_dut, "dV:%4.1fmV", dVduts);
    writeStrGraphicsLcd(0, 4, str_dut);
    putsUart0(str_dut);
    putsUart0("\r\n");
    sprintf(str_dut, "RL:%2.5fohm", RL);
    writeStrGraphicsLcd(0, 5, str_dut);
    putsUart0(str_dut);
    putsUart0("\r\n");
}

void Uart0Isr()
{
    char c = UART0_DR_R & 0xFF;
    putcUart0(c);
    if(c == 0x08)                   // if c == BackSpace
    {
        if(cnt_str > 0)
        {
            cnt_str--;
            str[cnt_str] = ' ';
        }
    }
    else if(c == 0x0D)              // if c == CarriageReturn
    {
        str[cnt_str] = 0;

        flag_null = 1;
        idx_strEnd = cnt_str;
        cnt_str = 0;
    }
    else if(c >= 32 && c <= 126)    // if c == printable data
    {
        str[cnt_str++] = c;
        if(cnt_str == MAX_CHARS)    // MAX chars = 20
        {
            str[cnt_str] = 0;

            flag_null = 1;
            idx_strEnd = cnt_str;
            cnt_str = 0;
        }
    }
}

void Timer1Isr()
{
    if(kb_loop == 0)
        ROW1 = 1;
    else if(kb_loop == 1)
        ROW1 = 0;
    else if(kb_loop == 2)
        ROW2 = 1;
    else if(kb_loop == 3)
        ROW2 = 0;
    else if(kb_loop == 4)
        ROW3 = 1;
    else if(kb_loop == 5)
        ROW3 = 0;
    else if(kb_loop == 6)
        ROW4 = 1;
    else if(kb_loop == 7)
        ROW4 = 0;

    kb_loop++;
    if(kb_loop == 8)
        kb_loop = 0;

    TIMER1_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag
}

void GPIODIsr()
{
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                    // turn-off timer before reconfiguring
    NVIC_EN0_R &= ~(1 << (INT_TIMER1A-16));             // turn-on interrupt 37 (TIMER1A)

    uint8_t t = GPIO_PORTD_MIS_R;
    uint8_t row = ((GPIO_PORTB_DATA_R & 0x30) >> 4 | (GPIO_PORTE_DATA_R & 0x30) >> 2);

    GREEN_LED = 1;
    if((t&0x02) != 0)
    {
//        while((GPIO_PORTD_DATA_R & 0x02) != 0);

        chk_kb = 0x10 | row;
    }
    else if((t&0x04) != 0)
    {
//        while((GPIO_PORTD_DATA_R & 0x04) != 0);

        chk_kb = 0x20 | row;
    }
    else if((t&0x08) != 0)
    {
//        while((GPIO_PORTD_DATA_R & 0x08) != 0);

        chk_kb = 0x40 | row;
    }
    GREEN_LED = 0;
    waitMicrosecond(10000);

    TIMER1_CTL_R |= TIMER_CTL_TAEN;                     // turn-on timer before reconfiguring
    NVIC_EN0_R |= 1 << (INT_TIMER1A-16);                // turn-on interrupt 37 (TIMER1A)

    GPIO_PORTD_ICR_R = 0x0E;
}

void ADComparator0Isr()
{
    // Get WT5 Counter Value
    cnt_wt5 = WTIMER5_TAV_R;
    // Get V_dut2
    adc_dut2 = readADC0SS3_Single(CH_ADC_DUT2);

    WTIMER5_CTL_R = 0;                  // turn off the couter
    COMP_ACMIS_R |= COMP_ACMIS_IN0;     // clear AC0 Int flag
}

int main(void)
{
    // Initialize hardware
    initHw();

    //----------------------------------------------------------//
    // Step 5.
    // Turn off every 5 control pins.
    //----------------------------------------------------------//
    SET_MEAS_LR_OFF();
    SET_MEAS_C_OFF();
    SET_HIGHSIDE_R_OFF();
    SET_LOWSIDE_R_OFF();     // De-Integrate
    SET_INTEGRATE_OFF();

    initGraphicsLcd();
    clearGraphicsLcd();
    clearGraphicsLcd();

    //----------------------------------------------------------//
    // Step 1.
    // Turn Red LED on for 500msec and Turn Red LED off for 500msec
    //----------------------------------------------------------//
    putsUart0("******* BOOTING MCU *******\r\n");
    writeStrGraphicsLcd(0, 3, "*******BOOTING*******");
    RED_LED = 1;
    waitMicrosecond(500000);
    RED_LED = 0;
    waitMicrosecond(500000);
    clearGraphicsLcd();

    while(1)
    {
        //----------------------------------------------------------------------------------------------------------------//
        //----------------------------------------------------------------------------------------------------------------//
        // Code for the project
        //----------------------------------------------------------------------------------------------------------------//
        //----------------------------------------------------------------------------------------------------------------//
        //--------------------------------------------------------------//
        // Step 2.
        // Getting Strings from the PC Terminal
        // Backspace => 8, Del => 127
        // CHAR => 32 ~ 126 including space
        // Upper letters => 65 ~ 90
        // Lower letters => 97 ~ 122
        // Numbers       => 48 ~ 57
        //--------------------------------------------------------------//
        writeStrGraphicsLcd(0, 0, str);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        if(flag_null == 1)          // received NULL char as final
        {
            UART0_IM_R &= ~(UART_IM_RTIM | UART_IM_RXIM);
            NVIC_EN0_R &= ~(1 << (INT_UART0-16));

            TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                    // turn-off timer before reconfiguring
            NVIC_EN0_R &= ~(1 << (INT_TIMER1A-16));             // turn-on interrupt 37 (TIMER1A)

            GPIO_PORTD_IM_R &= ~0x0E;
            NVIC_EN0_R &= ~(1 << (INT_GPIOD-16));

            parsingString();
            //--------------------------------------------------------------//
            // Step 4.
            // Check the string if it is a command or not
            //--------------------------------------------------------------//
            if(isCommand(str, 1) == 0)  // if command is not valid
            {
                writeStrGraphicsLcd(0, 1, "invalid CMD");
                writeStrGraphicsLcd(0, 7, "Press Enter.");
                putsUart0("It's invalid Command.\r\n");

                putsUart0("Press Enter to continue.\r\n");
                while(getcUart0() != 13){};    // waiting until input C/R is received
            }
            else
            {
                writeStrGraphicsLcd(0, 1, "CMD: ");
                writeStrGraphicsLcd(5, 1, &str_parse[0][0]);

                //--------------------------------------------------------------//
                // Step 5.
                // Controlling pins using terminal
                //--------------------------------------------------------------//
                if(strcmp(&str_parse[0][0], "set") == 0)
                {
                    if(strcmp(&str_parse[2][0], "on") == 0)
                    {
                        if(strcmp(&str_parse[1][0], "lr") == 0)
                            SET_MEAS_LR_ON();
                        else if(strcmp(&str_parse[1][0], "c") == 0)
                            SET_MEAS_C_ON();
                        else if(strcmp(&str_parse[1][0], "hs") == 0)
                            SET_HIGHSIDE_R_ON();
                        else if(strcmp(&str_parse[1][0], "ls") == 0)
                            SET_LOWSIDE_R_ON();
                        else if(strcmp(&str_parse[1][0], "in") == 0)
                            SET_INTEGRATE_ON();
                    }
                    else if(strcmp(&str_parse[2][0], "off") == 0)
                    {
                        if(strcmp(&str_parse[1][0], "lr") == 0)
                            SET_MEAS_LR_OFF();
                        else if(strcmp(&str_parse[1][0], "c") == 0)
                            SET_MEAS_C_OFF();
                        else if(strcmp(&str_parse[1][0], "hs") == 0)
                            SET_HIGHSIDE_R_OFF();
                        else if(strcmp(&str_parse[1][0], "ls") == 0)
                            SET_LOWSIDE_R_OFF();
                        else if(strcmp(&str_parse[1][0], "in") == 0)
                            SET_INTEGRATE_OFF();
                    }
                    pause();
                }
                //--------------------------------------------------------------//
                // Step 6.
                // Support "voltage" command
                //--------------------------------------------------------------//
                if(strcmp(&str_parse[0][0], "voltage") == 0)
                {
                    measure_voltage();
                    pause();
                }
                //--------------------------------------------------------------//
                // Step 7.
                // Support "reset" command
                //--------------------------------------------------------------//
                if(strcmp(&str_parse[0][0], "reset") == 0)
                {
                    software_reset();
                    pause();
                }
                //--------------------------------------------------------------//
                // Step 8.
                // Support "resistor" command
                //--------------------------------------------------------------//
                if(strcmp(&str_parse[0][0], "resistor") == 0)
                {
                    measure_resistor();
                    pause();
                }
                //--------------------------------------------------------------//
                // Step 9.
                // Support "capacitance" command
                //--------------------------------------------------------------//
                if(strcmp(&str_parse[0][0], "capacitance") == 0)
                {
                    measure_capacitance();
                    pause();
                }
                //--------------------------------------------------------------//
                // Step 10.
                // Support "inductance" command
                //--------------------------------------------------------------//
                if(strcmp(&str_parse[0][0], "inductance") == 0)
                {
                    measure_inductance();
                    pause();
                }
                //--------------------------------------------------------------//
                // Step 11.
                // Support "esr" command
                //--------------------------------------------------------------//
                if(strcmp(&str_parse[0][0], "esr") == 0)
                {
                    measure_esr();
                    pause();
                }
                //--------------------------------------------------------------//
                // Step 12.
                // Support "auto" command
                //--------------------------------------------------------------//
                if(strcmp(&str_parse[0][0], "auto") == 0)
                {
                    mode_auto();
                    pause();
                }

#ifdef __DEBUG__
                // Get Vdut1, Vdut2
                if(strcmp(&str_parse[0][0], "dut") == 0)
                {
//                    SET_MEAS_LR_ON();
//                    SET_LOWSIDE_R_ON();
                    waitMicrosecond(1000000);

                    get_each_volt_dut12();
                    pause();

//                    SET_MEAS_LR_OFF();
//                    SET_LOWSIDE_R_OFF();
                }
#endif
            }
            clearGraphicsLcd();

            memset(str, 0, MAX_CHARS);      // Make all data in the array str to NULL
            flag_null = 0;

            TIMER1_CTL_R |= TIMER_CTL_TAEN;                    // turn-off timer before reconfiguring
            NVIC_EN0_R |= (1 << (INT_TIMER1A-16));             // turn-on interrupt 37 (TIMER1A)

            GPIO_PORTD_ICR_R = 0x0E;
            GPIO_PORTD_IM_R |= 0x0E;
            NVIC_EN0_R |= (1 << (INT_GPIOD-16));

            UART0_IM_R |= (UART_IM_RTIM | UART_IM_RXIM);
            NVIC_EN0_R |= (1 << (INT_UART0-16));
        }
        else if((chk_kb & 0x70) != 0)
        {
            uint8_t col = chk_kb >> 4;
            uint8_t row = chk_kb & 0x0F;
            uint8_t flag_kb = 0;

            putsUart0("Key Pressed ");
            if(col == 1)
            {
                switch (row) {
                    case 1:
                        putsUart0("1\r\n");
                        flag_kb = 1;
                        break;
                    case 2:
                        putsUart0("4\r\n");
                        flag_kb = 4;
                        break;
                    case 4:
                        putsUart0("7\r\n");
                        flag_kb = 7;
                        break;
                }
            }
            else if(col == 2)
            {
                switch (row) {
                    case 1:
                        putsUart0("2\r\n");
                        flag_kb = 2;
                        break;
                    case 2:
                        putsUart0("5\r\n");
                        flag_kb = 5;
                        break;
                }
            }
            else if(col == 4)
            {
                switch (row) {
                    case 1:
                        putsUart0("3\r\n");
                        flag_kb = 3;
                        break;
                    case 2:
                        putsUart0("6\r\n");
                        flag_kb = 6;
                        break;
                }
            }

            chk_kb = 0;
            if(flag_kb != 0)
            {
                UART0_IM_R &= ~(UART_IM_RTIM | UART_IM_RXIM);
                NVIC_EN0_R &= ~(1 << (INT_UART0-16));

                TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                    // turn-off timer before reconfiguring
                NVIC_EN0_R &= ~(1 << (INT_TIMER1A-16));             // turn-on interrupt 37 (TIMER1A)

                GPIO_PORTD_IM_R &= ~0x0E;
                NVIC_EN0_R &= ~(1 << (INT_GPIOD-16));

                if(flag_kb == 1)
                {
                    writeStrGraphicsLcd(0, 0, "Key Pressed 1");
                    writeStrGraphicsLcd(0, 1, "Software Reset");
                    software_reset();
                }
                if(flag_kb == 2)
                {
                    writeStrGraphicsLcd(0, 0, "Key Pressed 2");
                    writeStrGraphicsLcd(0, 1, "Measuring Volt");
                    measure_voltage();
                }
                if(flag_kb == 3)
                {
                    writeStrGraphicsLcd(0, 0, "Key Pressed 3");
                    writeStrGraphicsLcd(0, 1, "Measuring R");
                    measure_resistor();
                }
                if(flag_kb == 4)
                {
                    writeStrGraphicsLcd(0, 0, "Key Pressed 4");
                    writeStrGraphicsLcd(0, 1, "Measuring C");
                    measure_capacitance();
                }
                if(flag_kb == 5)
                {
                    writeStrGraphicsLcd(0, 0, "Key Pressed 5");
                    writeStrGraphicsLcd(0, 1, "Measuring L");
                    measure_inductance();
                }
                if(flag_kb == 6)
                {
                    writeStrGraphicsLcd(0, 0, "Key Pressed 6");
                    writeStrGraphicsLcd(0, 1, "Measuring ESR");
                    measure_esr();
                }
                if(flag_kb == 7)
                {
                    writeStrGraphicsLcd(0, 0, "Key Pressed 7");
                    writeStrGraphicsLcd(0, 1, "Run Auto Mode");
                    mode_auto();
                }

                TIMER1_CTL_R |= TIMER_CTL_TAEN;                    // turn-off timer before reconfiguring
                NVIC_EN0_R |= (1 << (INT_TIMER1A-16));             // turn-on interrupt 37 (TIMER1A)

                GPIO_PORTD_ICR_R = 0x0E;
                GPIO_PORTD_IM_R |= 0x0E;
                NVIC_EN0_R |= (1 << (INT_GPIOD-16));

                writeStrGraphicsLcd(0, 7, "Press * Button");
                while(chk_kb != 0x18);
                chk_kb = 0;

                UART0_IM_R |= (UART_IM_RTIM | UART_IM_RXIM);
                NVIC_EN0_R |= (1 << (INT_UART0-16));

                clearGraphicsLcd();
            }
        }
    }
}
