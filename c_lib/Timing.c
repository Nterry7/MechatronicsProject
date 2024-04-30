/*
         MEGN540 Mechatronics Lab
    Copyright (C) Andrew Petruska, 2021.
       apetruska [at] mines [dot] edu
          www.mechanical.mines.edu
*/

/*
    Copyright (c) 2021 Andrew Petruska at Colorado School of Mines

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.

*/

#include "../c_lib/Timing.h"

/** These define the internal counters that will be updated in the ISR to keep track of the time
 *  The volatile keyword is because they are changing in an ISR, the static means they are not
 *  visible (not global) outside of this file.
 */
static volatile uint32_t _count_ms = 0;

/**
 * Function Initialize_Timing initializes Timer0 to have a prescalar of XX and initializes the compare
 * feature for use in an ISR.  It also enables ISR's.
 *
 *  FCPU = 16 000 000 Hz,  1 us => 1 000 000 Hz,  A prescalar of 16 would give you millisecond resolution, but you dont
 *  have enough time to do those additions every microsecond, in addition the 8-bit timer will roll over every 256 microseconds.
 *  So, you need to be counting in (effectively) 4us increments.  So use a prescalar of 64. This will give you a resolution of 4us per
 *  Timer0 Bit and enough head-room to compare at with 249 for a 1kHz update.
 *
 *  Since Timer 0 will be triggering at a kHz, we may want other things to be updated at 1kHz too.
 *
 */
void Initialize_Timing()
{
    // Set up timer with prescaler of 64
    TCCR0A &= ~(1 << COM0A0) | ~(1 << COM0A1);
    TCCR0B |= (1 << CS01) | (1 << CS00);
    
    // Compare value
    OCR0A = 249;

    // Initialize counter
    TCNT0 = 0;

    // Enable a compare match a interrupt
    TIMSK0 |= (1 << OCIE0A);

    // Initialize global interrupts
    sei();

    // Initialize the millisecond counter
    _count_ms = 0;
}

/**
 * This function gets the current time and returns it in seconds.
 * @return
 */
float Timing_Get_Time_Sec()
{
    // Calculate the time based on the milliseconds and microseconds
    return _count_ms * 0.001 + TCNT0*4 * 0.000001;
}
/**
 * This function gets the current time and returns it in a Time_t structure
 * @return
 */
Time_t Timing_Get_Time()
{
    // Set the time with the current millisecond count and the microseconds based on the value of the timer
    Time_t time = {
        .millisec = _count_ms,
        .microsec = TCNT0*4
    };

    return time;
}

/**
 * These functions return the individual parts of the Time_t struct, useful if you only care about
 * things on second or millisecond resolution.
 * @return
 */
uint32_t Timing_Get_Milli()
{
    return _count_ms;
}
uint16_t Timing_Get_Micro()
{
    // returns microseconds based on the value of the timer
    return TCNT0*4;
}

/**
 * This function takes a start time and calculates the time since that time, it returns it in the Time struct.
 * @param p_time_start a pointer to a start time struct
 * @return (Time_t) Time since the other time.
 */
float Timing_Seconds_Since( const Time_t* time_start_p )
{
    // Calculate the time between the start time and current time
    float delta_time = _count_ms - time_start_p->millisec;
    return delta_time;
}

/** This is the Interrupt Service Routine for the Timer0 Compare A feature.
 * You'll need to set the compare flags properly for it to work.
 */
ISR(TIMER0_COMPA_vect)
{
    // Reset timer to 0
    TCNT0 = 0;
    // take care of upticks of both our internal and external variables.
    _count_ms ++;
}
