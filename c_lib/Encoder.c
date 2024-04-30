#include "Encoder.h"

/**
 * Internal counters for the Interrupts to increment or decrement as necessary.
 */
static volatile bool _last_right_A = 0;  // Static limits it's use to this file
static volatile bool _last_right_B = 0;  // Static limits it's use to this file

static volatile bool _last_left_A   = 0;  // Static limits it's use to this file
static volatile bool _last_left_B   = 0;  // Static limits it's use to this file
static volatile bool _last_left_XOR = 0;  // Necessary to check if PB4 triggered the ISR or not

static volatile int32_t _left_counts  = 0;  // Static limits it's use to this file
static volatile int32_t _right_counts = 0;  // Static limits it's use to this file

const float _counts_to_rad = 2.0f * M_PI / 909.7;

/** Helper Funcions for Accessing Bit Information */

// Look at zumo-pinout to see which pins are connected to what
static inline bool Right_XOR()
{
    return bit_is_set( PINE, 6 );
}
static inline bool Right_B()
{
    return bit_is_set( PINF, 0 );
}
static inline bool Right_A()
{
    return Right_XOR() ^ Right_B();
}  // No external pin directly for this

static inline bool Left_XOR()
{
    return bit_is_set( PINB, 4 );
}
static inline bool Left_B()
{
    return bit_is_set( PINE, 2 );
}
static inline bool Left_A()
{
    return Left_XOR() ^ Left_B();
}  // No external pin directly for this

/**
 * Function Encoders_Init initializes the encoders, sets up the pin change interrupts, and zeros the initial encoder
 * counts.
 */
void Initialize_Encoders()
{
    // Left encoder uses PB4 and PE2 pins as digital inputs. External interrupt PCINT4 is necessary to detect
    // the change in XOR flag. You'll need to see Section 11.1.5 - 11.1.7 for setup and use.
    // Note that the PCINT interrupt is trigered by any PCINT pin. In the ISR you should check to make sure
    // the interrupt triggered is the one you intend on processing.

    // Right encoder uses PE6 adn PF0 as digital inputs. External interrupt INT6 is necessary to detect
    // the changes in XOR flag. You'll need to see Sections 11.1.2-11.1.4 for setup and use.
    // You'll use the INT6_vect ISR flag.

    // start by enabling global interupts
    sei();

    // Enable interrupts on the INT6 pin (Right-side encoder)
    EIMSK &= ~( 1 << INT6 );   // Needs to be cleared before setting EICRB
    EICRB &= ~( 1 << ISC61 );  // Want to ensure this bit is off
    EICRB |= ( 1 << ISC60 );   // Enable interrupts on both edges (so quadrature is possible)

    // Clear the flag register for the Right-side encoder
    EIFR |= ( 1 << INTF6 );  // Set to one, because that clears
    EIMSK |= ( 1 << INT6 );  // Now that EICRB is set, can re-enable

    // Enable interrupts on the PCINT4 pin (Left-side encoder)
    PCICR |= ( 1 << PCIE0 );
    PCMSK0 |= ( 1 << PCINT4 );

    // Clear the flag register for the Left-side endcoder
    PCIFR |= ( 1 << PCIF0 );  // Set to one, because that clears

    // Need to set encoder pins as digital inputs; steps for each pin:
    // (1) Disable "Data Direction Registry" / set the pin to zero
    // (2) Enable the correct "Port" / set the pin to one

    // Right encoder: PE6
    DDRE &= ~( 1 << DDE6 );
    PORTE |= ( 1 << PORTE6 );

    // Right encoder: PF0
    DDRF &= ~( 1 << DDF0 );
    PORTF |= ( 1 << PORTF0 );

    // Left encoder: PB4
    DDRB &= ~( 1 << DDB4 );
    PORTB |= ( 1 << PORTB4 );

    // Left encoder: PE2
    DDRE &= ~( 1 << DDE2 );
    PORTE |= ( 1 << PORTE2 );

    // Initialize static file variables.
    _last_right_A = Right_A();
    _last_right_B = Right_B();

    _last_left_A   = Left_A();
    _last_left_B   = Left_B();
    _last_left_XOR = Left_XOR();

    _left_counts  = 0;
    _right_counts = 0;
}

/**
 * Function Encoder_Counts_Left returns the number of counts from the left encoder.
 * @return [int32_t] The count number.
 */
int32_t Encoder_Counts_Left()
{
    cli();  // Temporarily pause interupts while trying to copy the int32_t value
    int32_t temp_count_num = _left_counts;
    sei();

    return temp_count_num;
}

/**
 * Function Encoder_Counts_Right returns the number of counts from the right encoder.
 * @return [int32_t] The count number.
 */
int32_t Encoder_Counts_Right()
{
    cli();  // Temporarily pause interupts while trying to copy the int32_t value
    int32_t temp_count_num = _right_counts;
    sei();

    return temp_count_num;
}

/**
 * Function Encoder_Rad_Left returns the number of radians for the left encoder.
 * @return [float] Encoder angle in radians
 */
float Encoder_Rad_Left()
{
    // Documentation says that Counts Per Revolution of wheels is 909.7!
    // So.. Counts Per Radian is: Counts * 2*pi / 909.7
    return Encoder_Counts_Left() * _counts_to_rad;
}

/**
 * Function Encoder_Rad_Right returns the number of radians for the left encoder.
 * @return [float] Encoder angle in radians
 */
float Encoder_Rad_Right()
{
    // Documentation says that Counts Per Revolution of wheels is 909.7!
    // So.. Counts Per Radian is: Counts * 2*pi / 909.7
    return 2 * Encoder_Counts_Right() * _counts_to_rad;
}


/**
 * Interrupt Service Routine for the left Encoder. Note: May need to check that it is actually PCINT4 that triggered, as
 * the Pin Change Interrupts can trigger for multiple pins.
 * @return
 */
ISR( PCINT0_vect )
{
    if( Left_XOR() ^ _last_left_XOR ) {  // Check to make sure that this is a valid call to this service routine
        // Equation to update the encoder count:
        _left_counts += ( Left_A() ^ _last_left_B ) - ( _last_left_A ^ Left_B() );

        // Update the stored encoder states:
        _last_left_XOR = Left_XOR();
        _last_left_A   = Left_A();
        _last_left_B   = Left_B();
    }
}

/**
 * Interrupt Service Routine for the right Encoder.
 * @return
 */
ISR( INT6_vect )
{
    // Equation to update the encoder count:
    _right_counts += ( Right_A() ^ _last_right_B ) - ( _last_right_A ^ Right_A() );

    // Update the stored encoder states:
    _last_right_A = Right_A();
    _last_right_B = Right_B();
}
