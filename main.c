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

// return codes
#define RC_FAIL -1
#define RC_OK 0

static uint8_t rotary_history;
//static int16_t rotary_count;

//static uint16_t softpot_value;

#define ENCODER_PINS_MASK 0x18          // rotary encoder pins on Port 2
#define ENCODER_PINS_LSB 3              // assume MSB = LSB + 1

// config states for setting time, alarms, etc.
typedef enum {
    CFG_OFF = 0,        // not in config menu -- this is normal clock mode
    CFG_TIME_DAY,
    CFG_TIME_HOUR,
    CFG_TIME_MIN,
    CFG_TIME_SEC,
    CFG_ALM_HOUR,
    CFG_ALM_MIN,
} config_state_t;

static config_state_t cfg_state;

// alarm definition
typedef struct {
    uint8_t action_flags;
    uint8_t day_of_week_flags;
    char minute;
    char hour;
    char pm;
} alarm_def_t;

// day of week flags for alarms
#define DOWF_WEEKDAYS ( (1<<MONDAY) | (1<<TUESDAY) | (1<<WEDNESDAY) \
                                    | (1<<THURSDAY) | (1<<FRIDAY) )
#define DOWF_WEEKENDS ( (1<<SUNDAY) | (1<<SATURDAY) )

// action flags for alarms
#define ALM_ACT_BEEP        0x01
#define ALM_ACT_RINGER      0x02
#define ALM_ACT_LAMP        0x04

static alarm_def_t alarms[3];
static uint8_t num_alarms = sizeof(alarms)/sizeof(alarm_def_t);
static uint8_t active_alarms_flags;

// =============================================================================

static void inline uart_begin_tx( void );
static int16_t uart_load_tx_str( const char *src );
static int16_t uart_load_tx_ch( const char ch );

static void lcd_write_time( void );
static char *lcd_name_for_day( char day );

static void config_next_state();
static void config_s2_pressed();
static void config_rot_up();
static void config_rot_down();
static void config_rot_press();
static void config_hour_up( char *hour, char* pm );
static void config_hour_down( char *hour, char* pm );
static void config_minute_up( char *min );
static void config_minute_down( char *min );

static void alarm_reset_def(alarm_def_t *alm);
static void alarm_set_def(alarm_def_t *alm, uint8_t action_flags, 
                            uint8_t dow_flags, char hour, char min, char pm);
static void alarm_check_alarms();
static void alarm_activate_alarm(uint8_t action_flags);
static void alarm_deactivate_alarms();

// =============================================================================

void main ( void )
{
    WDTCTL = WDTPW + WDTHOLD;                 // Stop watchdog timer

    //setTime( 0x12, 0, 0, 0);     // initialize time to 12:00:00 AM
    //TI_dayOfWeek = 0;            // initialize day of week to Sunday
    setTime( 0x07, 0x44, 0x40, 0);     
    TI_dayOfWeek = MONDAY;
    alarm_set_def( &alarms[0], ALM_ACT_BEEP|ALM_ACT_RINGER, 
                                        DOWF_WEEKDAYS, 0x07, 0x45, 0x00 );
    alarm_reset_def( &alarms[1] );
    alarm_reset_def( &alarms[2] );
    active_alarms_flags = 0x00;

    // TODO: set up unused pins to prevent floating inputs
    // TI datasheet recommends configuring as output (value does not matter) or
    // as input with internal pullup/pulldown

    // GPIO pin setup
    P1DIR = 0x41;               // set P1.0, 1.6 to output direction
                                // note: UART pin dir set by USCI
    P1SEL = 0x00;               // special functions turned on below as needed
    P1REN = 0x18;               // set P1.3, 1.4 pullup
    P1OUT = 0x18;               // set P1.3, 1.4 pullup
    P1IE  = 0x18;               // set P1.3, 1.4 interrupt enable
    P1IES = 0x18;               // set P1.3, 1.4 edge select falling edge
    P1IFG = 0x00;               // clear P1 interrupt flags

    //P2DIR = 0x04;       // all inputs -- outputs turned on below as needed
    //P2SEL = 0x00;       // special functions off -- turned on below as needed
    P2REN = ENCODER_PINS_MASK;  // set encoder pins pullup
    P2OUT = ENCODER_PINS_MASK;  // set encoder pins pullup
    P2IE  = ENCODER_PINS_MASK;  // set encoder pins interrupt enable
    P2IES = ENCODER_PINS_MASK;  // set encoder pins edge select falling edge
    P2IFG = 0x00;               // clear P2 interrupt flags

    // TimerA0 setup for RTC
    TACTL = TASSEL_1 | MC_1;                    // ACLK, upmode
    TACCR0 = 32768-1;
    TACCTL0 |= CCIE;                            // enable TA0CCRO interrupt

    // TimerA1 setup for speaker PWM
    TA1CCTL1 = OUTMOD_4;            // TA1CCR1 toggle
    TA1CTL   = TASSEL_1 | MC_1;     // ACLK, upmode
    TA1CCR0  = (32768 / 1200) - 1;  // TA1CCR1 toggle period, for ~600 Hz tone
    P2DIR   |= BIT2;                // P2.2 = output
    //P2SEL   |= BIT2;              // P2.2 = PWM output controlled by TA1.1

    P2DIR |= BIT1;                  // P2.1 = bell ringer

    // UART setup for LCD
    P1SEL |= BIT2;                          // select pin function: P1.2 = TXD
    P1SEL2 |= BIT2;                         // select pin function: P1.2 = TXD
    UCA0CTL1 |= UCSSEL_1;                   // CLK = ACLK
    UCA0BR0 = 0x03;                         // 32kHz/9600 = 3.41
    UCA0BR1 = 0x00;                         //
    UCA0MCTL = UCBRS1 | UCBRS0;             // Modulation UCBRSx = 3
    UCA0CTL1 &= ~UCSWRST;                   // Initialize USCI state machine

    // Disable time/alarm set menu
    cfg_state = CFG_OFF;

    // Reset rotary encoder
    rotary_history = 0x00;
    //rotary_count = 0;

    // Disable LCD cursor
    uart_load_tx_ch('C');
    uart_load_tx_ch('S');
    uart_load_tx_ch('\0');
    uart_begin_tx();

    __bis_SR_register(GIE);                   // set global interrupt enable

    while(1)
    {
        __bis_SR_register(LPM3_bits);     // enter LPM3 and wait for interrupt

        if ( TI_second == 0 ) {        // seconds just rolled over, check alarms
            alarm_check_alarms();
        }

        //P1OUT ^= 0x01;                    // do any other needed items in loop
        //if (TI_second != 0) {
            //P1OUT ^= 0x40;
        //}

        if ( active_alarms_flags & ALM_ACT_BEEP ) {
            P2SEL ^= BIT2;               // toggle PWM for speaker beep
        }

        //lcd_write_time();
        __no_operation();      // set breakpoint here to see 1 second interrupt
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

// Port 1 GPIO pushbutton interrupt
// TODO: debounce pushbuttons
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
    if (P1IFG & 0x08) {                 // Launchpad S2 pressed
        config_s2_pressed();
        P1IFG &= ~0x08;                 // clear interrupt
    } else if (P1IFG & 0x10) {          // rotary encoder pushbutton pressed
        config_rot_press();
        P1IFG &= ~0x10;                 // clear interrupt
    } else {
        P1IFG = 0x00;                   // should never happen
    }

}


// Port 2 rotary encoder interrupt
// TODO: make this a state machine to handle glitches and direction changes
#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
{
    // shift history left and shift in two encoder bits
    rotary_history = (rotary_history << 2)|((P2IN >> ENCODER_PINS_LSB) & 0x03);

    // check history for a full CW or CCW cycle
    if ((rotary_history & 0x0F) == 0x0B) {
      //rotary_count++;
      config_rot_up();
    } else if ((rotary_history & 0x0F) == 0x07 ) {
      //rotary_count--;
      config_rot_down();
    }
    
    // no way to trigger on rise+fall so flip edge select to catch next change
    P2IES = (P2IN & ENCODER_PINS_MASK);      
    P2IFG &= ~ENCODER_PINS_MASK;        // clear interrupt
}

// TODO: need to configure UART ISR to clear LPM until done?
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
    char *day_name;

    // clear display
    uart_load_tx_ch('C');
    uart_load_tx_ch('L');

    // begin text command
    uart_load_tx_ch('T');
    uart_load_tx_ch('T');

    // print day of week
    day_name = lcd_name_for_day(TI_dayOfWeek);
    uart_load_tx_ch(day_name[0]);
    uart_load_tx_ch(day_name[1]);
    uart_load_tx_ch(' ');

    // print time
    uart_load_tx_ch( itoc(TI_hour >> 4) );
    uart_load_tx_ch( itoc(TI_hour & 0x0F) );
    uart_load_tx_ch(':');
    uart_load_tx_ch( itoc(TI_minute >> 4) );
    uart_load_tx_ch( itoc(TI_minute & 0x0F) );
    uart_load_tx_ch(':');
    uart_load_tx_ch( itoc(TI_second >> 4) );
    uart_load_tx_ch( itoc(TI_second & 0x0F) );
    uart_load_tx_ch(' ');

    // print AM/PM
    if (TI_PM) {
        uart_load_tx_ch('P');
    } else {
        uart_load_tx_ch('A');
    }

    // print config menu state for time set or alarm set
    if (cfg_state != CFG_OFF)
    {
        uart_load_tx_ch(' ');
        uart_load_tx_ch('*');
        if (cfg_state == CFG_TIME_DAY) {
            uart_load_tx_ch('D');
        } else if (cfg_state == CFG_TIME_HOUR) {
            uart_load_tx_ch('H');
        } else if (cfg_state == CFG_TIME_MIN) {
            uart_load_tx_ch('M');
        } else if (cfg_state == CFG_TIME_SEC) {
            uart_load_tx_ch('S');
        }
    }

    uart_load_tx_ch('\0');
    uart_begin_tx();
}

// two-character name for day of week
static char *lcd_name_for_day( char day )
{
    switch (day)
    {
        case SUNDAY:       return "SU";
        case MONDAY:       return "MO";
        case TUESDAY:      return "TU";
        case WEDNESDAY:    return "WE";
        case THURSDAY:     return "TH";
        case FRIDAY:       return "FR";
        case SATURDAY:     return "SA";
        default:           return "  ";
    }
}

// =============================================================================

static void config_next_state( void )
{
    switch (cfg_state)
    {
        case CFG_OFF:       cfg_state = CFG_TIME_DAY;  break;
        case CFG_TIME_DAY:  cfg_state = CFG_TIME_HOUR;  break;
        case CFG_TIME_HOUR: cfg_state = CFG_TIME_MIN;   break;
        case CFG_TIME_MIN:  cfg_state = CFG_TIME_SEC;   break;
        case CFG_TIME_SEC:  cfg_state = CFG_OFF;        break;
        //case CFG_ALM_HOUR:  cfg_state = CFG_ALM_MIN;    break;
        //case CFG_ALM_MIN:   cfg_state = CFG_OFF;        break;
        default:            cfg_state = CFG_OFF;        break;
    }

    lcd_write_time();
    // TODO: turn off rotary encoder interrupts when config mode not active
}

// handle s2 pushbutton press
static void config_s2_pressed( void )
{
    config_next_state();
}

// handle rotary encoder turned clockwise
static void config_rot_up( void )
{
    switch (cfg_state)
    {
        //case CFG_OFF:       break;
        //case CFG_ALM_HOUR:  cfg_state = CFG_ALM_MIN;    break;
        //case CFG_ALM_MIN:   cfg_state = CFG_OFF;        break;
        case CFG_TIME_DAY:
            TI_dayOfWeek = (TI_dayOfWeek==SATURDAY) ? SUNDAY : TI_dayOfWeek + 1;
            break;
        case CFG_TIME_HOUR: 
            config_hour_up( &TI_hour, &TI_PM );
            break;
        case CFG_TIME_MIN:  
            config_minute_up( &TI_minute );
            break;
        default:            
           return;      // return -- don't update lcd
    }
    lcd_write_time();
}

// handle rotary encoder turned counter-clockwise
static void config_rot_down( void )
{
    switch (cfg_state)
    {
        //case CFG_OFF:       break;
        //case CFG_ALM_HOUR:  cfg_state = CFG_ALM_MIN;    break;
        //case CFG_ALM_MIN:   cfg_state = CFG_OFF;        break;
        case CFG_TIME_DAY:
            TI_dayOfWeek = (TI_dayOfWeek==SUNDAY) ? SATURDAY : TI_dayOfWeek - 1;
            break;
        case CFG_TIME_HOUR: 
            config_hour_down( &TI_hour, &TI_PM );
            break;
        case CFG_TIME_MIN:  
            config_minute_down( &TI_minute );
            break;
        default:            
            return;     // return -- don't update lcd
    }
    lcd_write_time();
}


// handle rotary encoder button press
static void config_rot_press( void )
{
    switch (cfg_state)
    {
        case CFG_OFF:       
            alarm_deactivate_alarms();      // turn off any active alarms
            break;
        //case CFG_ALM_HOUR:  cfg_state = CFG_ALM_MIN;    break;
        //case CFG_ALM_MIN:   cfg_state = CFG_OFF;        break;
        case CFG_TIME_SEC:  
            TI_second = 0x00;
            break;
        default:            
            return;     // return -- don't update lcd
    }
    lcd_write_time();
}

// increment BCD-coded hour, flip AM/PM if needed
static void config_hour_up( char *hour, char* pm )
{
    if (*hour == 0x12) {
        *hour = 0x01;
        *pm ^= 1;
    } else if (*hour == 0x09) {
        *hour = 0x10;
    } else {
        (*hour)++;
    }
}

// decrement BCD-coded hour, flip AM/PM if needed
static void config_hour_down( char *hour, char* pm )
{
    if (*hour == 0x01) {
        *hour = 0x12;
        *pm ^= 1;
    } else if (*hour == 0x10) {
        *hour = 0x09;
    } else {
        (*hour)--;
    }
}

// increment BCD-coded minute
static void config_minute_up( char *min )
{
    if (*min == 0x59) {
        *min = 0x00;
    } else if (((*min) & 0x0F) == 0x09) {
        *min &= ~0x0F;
        *min += 0x10;
    } else {
        (*min)++;
    }
}

// decrement BCD-coded minute
static void config_minute_down( char *min )
{
    if (*min == 0x00) {
        *min = 0x59;
    } else if (((*min) & 0x0F) == 0x00) {
        *min &= ~0x0F;
        *min += 0x09;
        *min -= 0x10;
    } else {
        (*min)--;
    }
}

// =============================================================================

static void alarm_reset_def(alarm_def_t *alm)
{
    alm->action_flags = 0x00;
    alm->day_of_week_flags = 0x00;
    alm->hour = 0x12;
    alm->minute = 0x00;
    alm->pm = 0x00;
}

static void alarm_set_def(alarm_def_t *alm, uint8_t action_flags, 
                            uint8_t dow_flags, char hour, char min, char pm)
{
    alm->action_flags = action_flags;
    alm->day_of_week_flags = dow_flags;
    alm->hour = hour;
    alm->minute = min;
    alm->pm = pm;
}

// called every minute
static void alarm_check_alarms()
{
    // bell ringer timeout -- turn off after one minute
    active_alarms_flags &= ~ALM_ACT_RINGER;

    for ( uint8_t i = 0; i < num_alarms; i++ )
    {
        // check whether alarm is enabled and check day/time
        if ( alarms[i].action_flags == 0x00
            || !(alarms[i].day_of_week_flags & (1<<TI_dayOfWeek))
            || alarms[i].pm != TI_PM || alarms[i].hour != TI_hour 
            || alarms[i].minute != TI_minute ) {
                continue;
        }

        alarm_activate_alarm( alarms[i].action_flags );
    }

    // bell ringer timeout -- turn off unless another alarm activated it
    if ((P2OUT & BIT1) && !(active_alarms_flags & ALM_ACT_RINGER)) {
        P2OUT &= ~BIT1;                 // turn off bell ringer
    }
}

// activate this alarm
static void alarm_activate_alarm(uint8_t action_flags)
{
    active_alarms_flags |= action_flags;

    if ( action_flags & ALM_ACT_RINGER ) {
        P2OUT |= BIT1;                  // turn on bell ringer
    }
}

// deactivate all alarms
static void alarm_deactivate_alarms( void )
{
    active_alarms_flags = 0x00;
    P2SEL &= ~BIT2;                 // turn off PWM to speaker
    P2OUT &= ~BIT1;                 // turn off bell ringer
}


