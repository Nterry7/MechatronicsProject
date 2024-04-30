#include "MotorPWM.h"

static uint16_t CURRENT_MAX_PWM;

void Initialize_MotorPWM( uint16_t MAX_PWM )
{
    CURRENT_MAX_PWM = MAX_PWM;

    // Setup the 4 control pin outputs for the H-bridge.
    PORTB &= ~((1 << PORTB6) | (1 << PORTB5)); // Left | Right motor PWM outputs
    DDRB |= (1 << PIN6) | (1 << PIN5); // Left | Right PWM DDR

    PORTB |= (1 << PORTB2) | (1 << PORTB1); // Left | Right direction pins
    DDRB |= (1 << DDB2) | (1 << DDB1); // Left | Right direction DDR

    // Disable Timer 1 at startup
    TCCR1B &= ~(1 << CS10);

    // Set the timer prescaler to 1
    TCCR1B &= ~(1 << CS11);
    TCCR1B &= ~(1 << CS12);
    
    // Set the motor speed pins to be PWM outputs
    TCCR1A &= ~(1 << COM1A0);
    TCCR1A |=  (1 << COM1A1);
    TCCR1A &= ~(1 << COM1B0);
    TCCR1A |=  (1 << COM1B1);
    TCCR1A &= ~(1 << COM1C0);
    TCCR1A &= ~(1 << COM1C1);

    // Define the waveform generation mode
    TCCR1B |=  (1 << WGM13);
    TCCR1B &= ~(1 << WGM12);
    TCCR1B &= ~(1 << WGM11);
    TCCR1B &= ~(1 << WGM10);

    // Set the max pwm values and reset the counts to start at zero.
    MotorPWM_Set_Max(MAX_PWM);
    OCR1B = 0;
    OCR1A = 0;
}

void MotorPWM_Enable( bool enable )
{
    if (enable) {
        TCCR1B |= (1 << CS10);
    } else {
        TCCR1B &= ~(1 << CS10);
    }
}

bool MotorPWM_Is_Enabled()
{
    return TCCR1B & (1 << CS10);
}

void MotorPWM_Set_Left( int16_t pwm )
{
    // Enable PWM on the motors if required, else disable.
    if (pwm != 0 || !MotorPWM_Is_Enabled()) {
        MotorPWM_Enable(true);
    }

    // Set the direction of the left motor.
    if (pwm > 0) {
        PORTB &= ~(1 << PORTB2);
    } else {
        PORTB |= (1 << PORTB2);
    }

    if (pwm < 0) {
        pwm = -pwm;
    }

    // Set the speed of the left motor.
    OCR1B = pwm;
}

void MotorPWM_Set_Right( int16_t pwm )
{
    // Enable PWM on the motors if required, else disable.
    if (pwm != 0) {
        MotorPWM_Enable(true);
    }

    // Set the direction of the right motor.
    if (pwm > 0) {
        PORTB &= ~(1 << PORTB1);
    } else {
        PORTB |= (1 << PORTB1);
    }

    if (pwm < 0) {
        pwm = -pwm;
    }

    // Set the speed of the right motor.
    OCR1A = pwm;
}

int16_t MotorPWM_Get_Left()
{
    if (PORTB & (PORTB2 << 1)) {
        return -OCR1B;
    }
    return OCR1B;
}

int16_t MotorPWM_Get_Right()
{
    if (PORTB & (PORTB1 << 1)) {
        return -OCR1A;
    }
    return OCR1A;
}

uint16_t MotorPWM_Get_Max()
{
    return CURRENT_MAX_PWM;
}

void MotorPWM_Set_Max( uint16_t MAX_PWM )
{

    TCCR1B &= ~(1 << CS10);
    cli();
    uint8_t tempSREG = SREG;
    SREG = 0;

    CURRENT_MAX_PWM = MAX_PWM;
    TCNT1 = 0;
    ICR1 = MAX_PWM;

    SREG = tempSREG;
    sei();
    TCCR1B |= (1 << CS10);
}