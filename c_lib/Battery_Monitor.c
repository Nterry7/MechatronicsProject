#include "Battery_Monitor.h"

static const float BITS_TO_BATTERY_VOLTS = 2.0f * ( 5.0f / 1024.0f );

/**
 * Function Initialize_Battery_Monitor initializes the Battery Monitor to record the current battery voltages.
 */
void Initialize_Battery_Monitor()
{
    // Set conversion frequency to 125 kHz
    ADCSRA |= ( 1 << ADPS0 ) | ( 1 << ADPS1 ) | ( 1 << ADPS2 );

    // Enable A/D
    ADCSRA |= ( 1 << ADEN );

    // Enable A/D complete interrupt
    // ADCSRA |= (1 << ADIE);

    // Set 0 gain
    ADMUX &= ~( 1 << MUX0 ) | ~( 1 << MUX3 ) | ~( 1 << MUX4 );
    ADMUX |= ( 1 << MUX2 ) | ( 1 << MUX1 );

    // Enable external reference and capacitor
    ADMUX |= ( 1 << REFS0 );

    // Disable digital inputs to enable A/D on ADC6 for the battery
    DIDR0 |= ( 1 << ADC6D );

    // Initialize the filter
    float numerator[5]   = { 0.00006238698354840f, 0.000249547934193921f, 0.000374321901290881f, 0.000249547934193921f, 0.0000623869835484f };
    float denominator[5] = { 1.0f, -3.507786207390780f, 4.640902412686700f, -2.742652821120367f, 0.610534807561222 };
    uint8_t order        = 4;
    Filter_Init( &bat_filt, numerator, denominator, order );
}

/**
 * Function Battery_Voltage initiates the A/D measurement and returns the result for the battery voltage.
 */
float Battery_Voltage()
{
    static int Fill_Filter = 1;

    // A Union to assist with reading the LSB and MSB in the  16 bit register
    union {
        struct {
            uint8_t LSB;
            uint8_t MSB;
        } split;
        uint16_t value;
    } data = { .value = 0 };

    // Start A/D conversion
    ADCSRA |= ( 1 << ADSC );

    // Wait for the conversion to finish
    while( ( bit_is_set( ADCSRA, ADSC ) ) ) {}

    data.split.LSB = ADCL;
    data.split.MSB = ADCH;

    float bat_volt = data.value * BITS_TO_BATTERY_VOLTS;

    if( Fill_Filter ) {
        Fill_Filter = 0;
        Filter_SetTo( &bat_filt, bat_volt );
    }

    return Filter_Value( &bat_filt, bat_volt );
}

float Curr_Bat_Volt()
{
    return Filter_Last_Output( &bat_filt );
}