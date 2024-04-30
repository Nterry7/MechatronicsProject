#include "Lab3_Tasks.h"

void Send_Encoder_Count( float _unused )
{
    struct {
        float v1;
        float v2;
    } data;
    data.v1 = Encoder_Rad_Left();
    data.v2 = ( 1 * Encoder_Rad_Right() );  // UUHHHHH..?
    USB_Send_Msg( "cff", 'E', &data, sizeof( data ) );
    return;
};

void Send_Battery_Voltage( float _unused )
{
    struct {
        float v1;
    } data;
    data.v1 = Curr_Bat_Volt();
    USB_Send_Msg( "cf", 'B', &data, sizeof( data ) );
    return;
};

void Battery_Monitoring( float _unused )
{
    static int warning_freq = 0;  // Send every second, function is called every 2ms
    float curr_volt         = Battery_Voltage();

    if( ( curr_volt <= 4 ) && ( warning_freq >= 500 ) ) {
        warning_freq = 0;
        struct {
            char let[7];
            float volts;
        } data = { .let = { 'B', 'A', 'T', ' ', 'L', 'O', 'W' }, .volts = curr_volt };
        USB_Send_Msg( "c7cf", '!', &data, sizeof( data ) );
    }

    if( warning_freq < 500 ) {
        warning_freq++;
    }
    return;
};