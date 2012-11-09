#include <msp430g2553.h>
#include "RTC.h"

#define MAX_UART_BUF 32
static char uart_tx_buf[MAX_UART_BUF];// = {'T','T','a','b','c','d','e','f',0};
static const char *uart_tx_buf_end = &uart_tx_buf[MAX_UART_BUF];
static char *uart_tx_buf_send = uart_tx_buf;
static char *uart_tx_buf_load = uart_tx_buf;

char string1[] = { "TTHello World " };
unsigned int i;

void uart_tx_str( char *src );

void main ( void )
{
  WDTCTL = WDTPW + WDTHOLD;                 // Stop watchdog timer

  setTime( 0x12, 0, 0, 0);     // initialize time to 12:00:00 AM

  // TODO: set up unused pins to prevent floating inputs
  // TI datasheet recommends configuring as output (value does not matter) or
  // as input with internal pullup/pulldown

  // GPIO pin setup
  P1DIR = 0x41;                // set P1.0, 1.6 to output direction
                               // note: UART pin dir set by USCI
  P1OUT = 0x08;
  P1REN = 0x08;                // set P1.3 pullup, interrupt enable, edge select
  P1IE = 0x08;
  P1IES = 0x08;
  P1IFG = 0x00;                // clear P1 interrupt flags
  
  // TimerA setup for RTC
  TACCR0 = 32768-1;
  TACTL = TASSEL_1+MC_1;                    // ACLK, upmode
  TACCTL0 |= CCIE;                          // enable TA0CCRO interrupt

  // UART setup for LCD
  P1SEL = 0x04;                 // select pin function: P1.2 = TXD
  P1SEL2 = 0x04;                // select pin function: P1.2 = TXD
  //P1SEL = 0x06;                 // select pin function: P1.1 = RXD, P1.2 = TXD
  //P1SEL2 = 0x06;                // select pin function: P1.1 = RXD, P1.2 = TXD
  UCA0CTL1 |= UCSSEL_1;                     // CLK = ACLK
  UCA0BR0 = 0x03;                           // 32kHz/9600 = 3.41
  UCA0BR1 = 0x00;                           //
  UCA0MCTL = UCBRS1 + UCBRS0;               // Modulation UCBRSx = 3
  UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
  //IE2 |= UCA0RXIE;                          // Enable USCI_A0 RX interrupt

  __bis_SR_register(GIE);                   // set global interrupt enable

  while(1)
  {
    __bis_SR_register(LPM3_bits);           // enter LPM3 and wait for an interrupt

    P1OUT ^= 0x01;                          // do any other needed items in loop
    P1OUT ^= 0x40;
    if (TI_second == 0) {
      P1OUT ^= 0x40;
    }
    __no_operation();                       // set breakpoint here to see 1 second interrupt
  }
}


#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A (void)
{
    incrementSeconds();
    __bic_SR_register_on_exit(LPM3_bits);
}

// Port 1 GPIO pushbutton interrupt
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
    // P1OUT ^= 0x40;
    // TI_minute = 0x59;
    // TI_second = 0x58;

    //i = 0;
    uart_tx_str(string1);
    IE2 |= UCA0TXIE;                    // Enable USCI_A0 TX interrupt
    IFG2 |= UCA0TXIFG;                  // Trigger UART TXD interrupt to begin
    //UCA0TXBUF = string1[i++];

    P1IFG &= ~0x08;                         // clear Port 1 interrupt
}

// UART TXD interrupt
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void)
{
    UCA0TXBUF = *uart_tx_buf_send;              // send next character
    uart_tx_buf_send++;

    if (uart_tx_buf_send == uart_tx_buf_end) {  // end of buffer; wrap to front
        uart_tx_buf_send = uart_tx_buf;
    }

    if (uart_tx_buf_send == uart_tx_buf_load) { // buffer is empty
        IE2 &= ~UCA0TXIE;                       // disable USCI_A0 TX interrupt
    }

  //UCA0TXBUF = string1[i++];                 // TX next character

  //if (i == sizeof(string1))              // TX over?
    //IE2 &= ~UCA0TXIE;                       // Disable USCI_A0 TX interrupt
}

/*
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
  if (UCA0RXBUF == 'u')                     // 'u' received?
  {
    i = 0;
    IE2 |= UCA0TXIE;                        // Enable USCI_A0 TX interrupt
    UCA0TXBUF = string1[i++];
  }
}
*/

// =============================================================================

void uart_tx_str( char *src )
{
    while (1)
    {
        *uart_tx_buf_load = *src;

        // check for buffer full condition
        if (uart_tx_buf_load + 1 == uart_tx_buf_send
                || (uart_tx_buf_load == uart_tx_buf_end - 1 
                    && uart_tx_buf_send == uart_tx_buf) )
        {
            // TODO: wait here for buffer to empty?
            break;
        }

        uart_tx_buf_load++;
        if (uart_tx_buf_load == uart_tx_buf_end) {
            uart_tx_buf_load = uart_tx_buf;
        }

        if (!*src) {        // copied terminating null
            break;
        }
        src++;
    }
}





