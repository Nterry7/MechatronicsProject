#include "Lab2_Tasks.h"

// Description: Sends the time of a loop iteration in main at the desired rate 
// Input: Time since the task was last run
// Output: NONE
void Send_Loop_Time( float _time_since_last ) {
    static float local_run_period = 0.0;
    
    // If the run period of the task is not 0, make it 0 so the 
    // loop iterations will still be counted even when the task
    // is not being run
    if (task_time_loop.run_period != 0.0) {
        local_run_period = task_time_loop.run_period;
        task_time_loop.run_period = 0.0;
    }

    // Reset the task run period to its origional run period and send the message
    else {
        task_time_loop.run_period = local_run_period;
        Time_Mgs_t time_msg = {
            .c = '1',
            .time = Timing_Get_Time_Sec() - (_time_since_last * 0.001)
        };
        USB_Send_Msg( "ccf", 'T', &time_msg, sizeof( time_msg ) );
    }
}

// Description: Sends the current time at the desired rate 
// Input: Time since the task was last run
// Output: NONE
void Send_Time_Now( float _time_since_last ) {
    Time_Mgs_t time_msg = {
        .c = '0',
        .time = Timing_Get_Time_Sec()
    };
    
    USB_Send_Msg( "ccf", 'T', &time_msg, sizeof( time_msg ) );
}