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

#include "Battery_Monitor.h"  // for Evaluating Battery levels
#include "Encoder.h"          // for Tracking encoder vals
#include "Lab1_Tasks.h"
#include "Lab2_Tasks.h"
#include "Lab3_Tasks.h"
#include "Lab5_Tasks.h"
#include "Message_Handling.h"           // for translating USB messages to microcontroller tasks
#include "SerialIO.h"                   // for USB communication
#include "Task_Management.h"            // for clean task management with functors
#include "Timing.h"                     // for Time understanding
#include "MotorPWM.h"

// put your global variables (only if absolutely required) here.
// Best to identify them as "static" to make them indentified as internal and start with a "_" to identify as internal.
// Try to initialize them if possible, so their values are never arbitrary.

void Initialize_Modules( float _time_not_used_ )
{
    // Initialize (reinitialize) all global variables

    // Initialize controller
    static float z_transform_numerator[2] = {1.016168314802673, -0.968549492978889};
    static float z_transform_denominator[2] = {1.0,-0.952381178176217};
    static uint8_t z_transform_order = 1;
    static float update_period = 0.002;
    float wheel_base_width = 0.06;
    float wheel_diameter = 0.038;
    float kp = 255.09911;
    int16_t MAX_PWM = 400;

    Initialize_Skid_Steer( &Skid_Controller, z_transform_numerator, z_transform_denominator, z_transform_order, 
                        wheel_base_width, wheel_diameter, kp, MAX_PWM, update_period, &Encoder_Rad_Left, &Encoder_Rad_Right, 
                        &MotorPWM_Set_Left, &MotorPWM_Set_Right );

    // Initialize all modules
    Initialize_USB();
    Initialize_Timing();
    Initialize_Encoders();
    Initialize_Battery_Monitor();
    Initialize_MotorPWM(MAX_PWM);

    // Setup task handling
    Initialize_Task( &task_restart, -1 /*do only once*/, Initialize_Modules /*function pointer to call*/ );

    // Setup message handling to get processed at some desired rate.
    Initialize_Task( &task_message_handling, 0.03, Task_Message_Handling );
    
    // Setup timer tasks
    Initialize_Task( &task_send_time, -1, Send_Time_Now);
    Initialize_Task( &task_time_loop, -1, Send_Loop_Time);
    Initialize_Task( &task_message_handling_watchdog, 100/*watchdog timout period*/,  Task_Message_Handling_Watchdog );

    // Setup encoder and battery tasks
    Initialize_Task( &task_send_encoder_count, -1.0f, Send_Encoder_Count );
    Initialize_Task( &task_send_battery_voltage, -1.0f, Send_Battery_Voltage );
    Initialize_Task( &task_battery_monitoring, 0.002f, Battery_Monitoring );

    // Setup the tasks for motors
    Initialize_Task(&task_schedule_stop, -1, Task_Stop_Motors);
    Initialize_Task(&task_send_system_data, -1, Task_Send_System_Data);

    // Setup tasks for controller
    Initialize_Task(&task_ctrl, update_period, Task_Ctrl);
    Initialize_Task(&task_ctrl_stop, -1, Task_Ctrl_Stop);

    // Activate tasks that should always run:
    Task_Activate( &task_message_handling ); 
    Task_Activate( &task_battery_monitoring );

}

/** Main program entry point. This routine configures the hardware required by the application, then
 *  enters a loop to run the application tasks in sequence.
 */
int main( void )
{    
    Initialize_Modules( 0.0 );

    for( ;; ) {

        Task_USB_Upkeep();

        // Lab 5 Tasks
        Task_Run_If_Ready(&task_ctrl);
        Task_Run_If_Ready(&task_ctrl_stop);

        // Lab 4 Tasks
        Task_Run_If_Ready(&task_schedule_stop);
        Task_Run_If_Ready(&task_send_system_data);

        // Lab 3 Tasks
        Task_Run_If_Ready( &task_send_encoder_count );
        Task_Run_If_Ready( &task_send_battery_voltage );
        Task_Run_If_Ready( &task_battery_monitoring );

        // Lab 2 Tasks
        Task_Run_If_Ready( &task_send_time );
        Task_Run_If_Ready( &task_time_loop );

        // Lab 1 Tasks
        Task_Run_If_Ready( &task_restart );
        Task_Run_If_Ready( &task_message_handling );
        Task_Run_If_Ready( &task_message_handling_watchdog );
    }

    return 0;
}