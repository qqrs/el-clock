#include <stdint.h>

#include <msp430g2553.h>
#include "RTC.h"

// =============================================================================

#define MAX_UART_BUF 32
static char uart_tx_buf[MAX_UART_BUF];
static const char *uart_tx_buf_end = &uart_tx_buf[MAX_UART_BUF];
static char *uart_tx_buf_send = uart_tx_buf;
static char *uart_tx_buf_load = uart_tx_buf;

const char *string1 = "TTHello World ";
const char *string2 = "TTA123456789B123456789C123456789D123456789";

#define MCU_CLOCK           32768
#define PWM_FREQUENCY       1200     // In Hertz, ideally 50Hz.
unsigned int PWM_Period     = (MCU_CLOCK / PWM_FREQUENCY);  // PWM Period
unsigned int PWM_Duty       = (MCU_CLOCK / PWM_FREQUENCY) / 2;

// return codes
#define RC_FAIL -1
#define RC_OK 0

// =============================================================================

static void inline uart_begin_tx( void );
static int16_t uart_load_tx_str( const char *src );
static int16_t uart_load_tx_ch( const char ch );

static void lcd_write_time( void );

// =============================================================================

void main ( void )
{
    WDTCTL = WDTPW + WDTHOLD;                 // Stop watchdog timer

    //setTime( 0x12, 0, 0, 0);     // initialize time to 12:00:00 AM
    setTime( 0x01, 0x54, 0, 0);     // initialize time to 12:00:00 AM

    // TODO: set up unused pins to prevent floating inputs
    // TI datasheet recommends configuring as output (value does not matter) or
    // as input with internal pullup/pulldown

    // GPIO pin setup
    P1DIR = 0x41;               // set P1.0, 1.6 to output direction
                                // note: UART pin dir set by USCI
    P1SEL = 0x00;               // special functions turned on below as needed
    P1OUT = 0x08;
    P1REN = 0x08;               // set P1.3 pullup, interrupt enable, edge select
    P1IE = 0x08;
    P1IES = 0x08;
    P1IFG = 0x00;               // clear P1 interrupt flags

    //P2DIR = 0x00;               // set P1.0, P1.2 to output direction for speaker
    //P2SEL = 0x00;               // special functions turned on below as needed

    // TimerA setup for RTC
    TACTL = TASSEL_1 | MC_1;                    // ACLK, upmode
    TACCR0 = 32768-1;
    TACCTL0 |= CCIE;                          // enable TA0CCRO interrupt

    // TimerA1 setup for speaker
    //TA1CTL = TASSEL_1+MC_1;                    // ACLK, upmode
    //TA1CCR0 = 40;                              // about 800 Hz audio tone
    //TA1CCTL0 |= CCIE;                          // enable TA0CCRO interrupt
    
    // TimerA1 setup for speaker PWM
    //TA1CTL = MC_1 | TASSEL_1;           // Reset TA1R, set up mode, TA1 runs from 32768Hz ACLK
    //TA1CCR0 = 40;                               // Set PWM frequency
    //TA1CCTL0 = OUTMOD_4;                        // Enable IRQ, set output mode "toggle"
    //P2SEL |= BIT0;                              // PWM output on P2.0 for speaker
    //TA1CCTL0 |= CCIE;                           // enable TA0CCR1 interrupt
                                                // Activate Timer0_A3 periodic interrupts
    // TimerA1 setup for speaker PWM
    //TACCTL1 = OUTMOD_7; // TACCR1 reset/set
    //TACTL = TASSEL_2 + MC_1; // SMCLK, upmode
    //TACCR0 = PWM_Period-1; // PWM Period
    //TACCR1 = PWM_Duty; // TACCR1 PWM Duty Cycle
    //P1DIR |= BIT2; // P1.2 = output
    //P1SEL |= BIT2; // P1.2 = TA1 output

    // TimerA1 setup for speaker PWM
    TA1CCTL1 = OUTMOD_4;            // TA1CCR1 toggle
    TA1CTL   = TASSEL_1 | MC_1;     // ACLK, upmode
    TA1CCR0  = PWM_Period-1;        // TA1CCR1 PWM Period
    TA1CCR1  = PWM_Duty;            // TA1CCR1 PWM Duty Cycle
    P2DIR   |= BIT2;               // P2.2 = output
    P2SEL   |= BIT2;               // P2.2 = TA1.1 output

    // UART setup for LCD
    P1SEL |= BIT2;                            // select pin function: P1.2 = TXD
    P1SEL2 |= BIT2;                           // select pin function: P1.2 = TXD
    UCA0CTL1 |= UCSSEL_1;                     // CLK = ACLK
    UCA0BR0 = 0x03;                           // 32kHz/9600 = 3.41
    UCA0BR1 = 0x00;                           //
    UCA0MCTL = UCBRS1 + UCBRS0;               // Modulation UCBRSx = 3
    UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**

    // Disable LCD cursor
    uart_load_tx_ch('C');
    uart_load_tx_ch('S');
    uart_load_tx_ch('\0');
    uart_begin_tx();

    __bis_SR_register(GIE);                   // set global interrupt enable

    while(1)
    {
        __bis_SR_register(LPM3_bits);           // enter LPM3 and wait for an interrupt

        P1OUT ^= 0x01;                          // do any other needed items in loop
        if (TI_second != 0) {
            P1OUT ^= 0x40;
        }
        P2SEL ^= BIT2;               // toggle PWM for speaker beep
        //lcd_write_time();
        __no_operation();                       // set breakpoint here to see 1 second interrupt
    }
}

// =============================================================================

#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A (void)
{
    incrementSeconds();
    lcd_write_time();   // TODO: move this out of isr 
    __bic_SR_register_on_exit(LPM3_bits);
}

/*
#pragma vector=TIMER0_A1_VECTOR
__interrupt void Timer_A1 (void)
{
    P2OUT &= 0x01;
}
*/

// Port 1 GPIO pushbutton interrupt
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
    // P1OUT ^= 0x40;
    //TI_minute = 0x59;
    //TI_second = 0x58;

    //if (uart_load_tx_str(string1) == RC_OK) {
    //    uart_begin_tx();
    //}
  
    incrementMinutes();
    P1IFG &= ~0x08;                         // clear Port 1 interrupt
}

// TODO: configure UART ISR to clear LPM until done
// UART TXD interrupt
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void)
{
    UCA0TXBUF = *uart_tx_buf_send;              // send next character
    uart_tx_buf_send++;

    if (uart_tx_buf_send == uart_tx_buf_end) {  
        uart_tx_buf_send = uart_tx_buf;         // end of buffer; wrap to front
    }

    if (uart_tx_buf_send == uart_tx_buf_load) { // buffer is empty
        IE2 &= ~UCA0TXIE;                       // disable USCI_A0 TX interrupt
    }
}

/*
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
    if (UCA0RXBUF == 'u')                     // 'u' received?
    {
    }
}
*/

// =============================================================================


// return RC_FAIL if  not loaded due to full buffer; RC_OK otherwise
static int16_t uart_load_tx_ch( const char ch )
{
    // check for buffer full condition
    if (uart_tx_buf_load + 1 == uart_tx_buf_send
            || (uart_tx_buf_load == uart_tx_buf_end - 1 
                && uart_tx_buf_send == uart_tx_buf) )
    {
        return RC_FAIL;
    }

    *uart_tx_buf_load = ch;
    uart_tx_buf_load++;
    if (uart_tx_buf_load == uart_tx_buf_end) {
        uart_tx_buf_load = uart_tx_buf;     // end of buffer; wrap to front
    }
    return RC_OK;

}

// return RC_FAIL if no characters loaded due to full buffer; RC_OK otherwise
static int16_t uart_load_tx_str( const char *src )
{
    int16_t ret = RC_FAIL;
    while (1)
    {
        if (uart_load_tx_ch( *src ) != RC_OK) {
            break;
        }
        ret = RC_OK;

        if (!*src) {        // finished copying -- copied terminating null 
            break;
        }
        src++;
    }
    return ret;
}

static void inline uart_begin_tx( void ) 
{
    IE2 |= UCA0TXIE;                    // Enable USCI_A0 TX interrupt
    IFG2 |= UCA0TXIFG;                  // Trigger UART TXD interrupt to begin
}

// =============================================================================

#define itoc(val) ((val) + '0')         // convert int to char

static void lcd_write_time( void )
{
    uart_load_tx_ch('C');
    uart_load_tx_ch('L');
    uart_load_tx_ch('T');
    uart_load_tx_ch('T');
    uart_load_tx_ch( itoc(TI_hour >> 4) );
    uart_load_tx_ch( itoc(TI_hour & 0x0F) );
    uart_load_tx_ch(':');
    uart_load_tx_ch( itoc(TI_minute >> 4) );
    uart_load_tx_ch( itoc(TI_minute & 0x0F) );
    uart_load_tx_ch(':');
    uart_load_tx_ch( itoc(TI_second >> 4) );
    uart_load_tx_ch( itoc(TI_second & 0x0F) );
    uart_load_tx_ch(' ');

    if (TI_PM) {
        uart_load_tx_ch('P');
    } else {
        uart_load_tx_ch('A');
    }

    uart_load_tx_ch('\0');
    uart_begin_tx();
}





