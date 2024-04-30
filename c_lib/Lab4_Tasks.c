#include "Lab4_Tasks.h"

void Task_Stop_Motors(float __not_used__)
{
    MotorPWM_Set_Left(0);
    MotorPWM_Set_Right(0);
}

void Task_Send_System_Data(float _time_since_last)
{
    struct {
        float time;
        int16_t pwm_left;
        int16_t pwm_right;
        int16_t encoder_left;
        int16_t encoder_right; 
    } data;

    data.time = Timing_Get_Time_Sec();
    data.pwm_left = MotorPWM_Get_Left();
    data.pwm_right = MotorPWM_Get_Right();
    data.encoder_left = Encoder_Counts_Left();
    data.encoder_right = Encoder_Counts_Right();

    USB_Send_Msg( "cfhhhh", 'q', &data, sizeof( data ) );
}