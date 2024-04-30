/*
         MEGN540 Mechatronics Lab
    Copyright (C) Andrew Petruska, 2021.
       apetruska [at] mines [dot] edu
          www.mechanical.mines.edu
*/

/*
    Copyright (c) 2021 Andrew Petruska at Colorado School of Mines

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to
    deal in the Software without restriction, including without limitation the
    rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
    sell copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
    IN THE SOFTWARE.

*/

#include "Message_Handling.h"

/**
 * Function _Message_Length returns the number of bytes associated with a command string per the
 * class documentation; This is only accessable from this file.
 * @param cmd
 * @return Size of expected string. Returns 0 if unrecognized.
 */
static uint8_t _Message_Length( char cmd );

/**
 * Function Task_Message_Handling processes USB messages as necessary and sets
 * status flags to control the flow of the program.
 */
void Task_Message_Handling( float _time_since_last )
{
    // *** MEGN540  ***
    // YOUR CODE HERE. I suggest you use your peak function and a switch interface
    // Either do the simple stuff strait up, set flags to have it done later.
    // If it just is a USB thing, do it here, if it requires other hardware, do it
    // in the main and set a flag to have it done here.

    // Check to see if their is data in waiting
    if( !USB_Msg_Length() )
        return;  // nothing to process...

    // Get Your command designator without removal so if their are not enough
    // bytes yet, the command persists
    char command = USB_Msg_Peek();

    // /* MEGN540 -- LAB 2 */ 
    bool command_processed = false;

    // Character to indicate undefined behavior
    char q_mark = '?';

    // process command
    switch( command ) {
        case '*':
            if( USB_Msg_Length() >= _Message_Length( '*' ) ) {
                // then process your multiplication...

                // remove the command from the usb recieved buffer using the
                // usb_msg_get() function
                USB_Msg_Get();  // removes the first character from the received buffer,
                                // we already know it was a * so no need to save it as a
                                // variable

                // Build a meaningful structure to put your data in. Here we want two
                // floats.
                struct __attribute__( ( __packed__ ) ) {
                    float v1;
                    float v2;
                } data;

                // Copy the bytes from the usb receive buffer into our structure so we
                // can use the information
                USB_Msg_Read_Into( &data, sizeof( data ) );
                
                // Call MEGN540_Lab_Task Function
                Multiply_And_Send( data.v1, data.v2 );

                // /* MEGN540 -- LAB 2 */ 
                command_processed = true;
            }
            break;
        case '/':
            if( USB_Msg_Length() >= _Message_Length( '/' ) ) {
                // then process your divide...
                
                // remove the command from the usb recieved buffer using the
                // usb_msg_get() function
                USB_Msg_Get();  // removes the first character from the received buffer,
                                // we already know it was a * so no need to save it as a
                                // variable

                // Build a meaningful structure to put your data in. Here we want two
                // floats.
                struct __attribute__( ( __packed__ ) ) {
                    float v1;
                    float v2;
                } data;

                // Copy the bytes from the usb receive buffer into our structure so we
                // can use the information
                USB_Msg_Read_Into( &data, sizeof( data ) );

                // Call MEGN540_Lab_Task Function
                Divide_And_Send( data.v1, data.v2 );
                // /* MEGN540 -- LAB 2 */ 
                command_processed = true;
            }
            break;
        case '+':
            if( USB_Msg_Length() >= _Message_Length( '+' ) ) {
                // then process your plus...
                
                // remove the command from the usb recieved buffer using the
                // usb_msg_get() function
                USB_Msg_Get();  // removes the first character from the received buffer,
                                // we already know it was a * so no need to save it as a
                                // variable

                // Build a meaningful structure to put your data in. Here we want two
                // floats.
                struct __attribute__( ( __packed__ ) ) {
                    float v1;
                    float v2;
                } data;

                // Copy the bytes from the usb receive buffer into our structure so we
                // can use the information
                USB_Msg_Read_Into( &data, sizeof( data ) );

                // Call MEGN540_Lab_Task Function
                Add_And_Send( data.v1, data.v2 );
                // /* MEGN540 -- LAB 2 */ 
                command_processed = true;
            }
            break;
            
        case '-':
            if( USB_Msg_Length() >= _Message_Length( '-' ) ) {
                // then process your minus...
                
                // remove the command from the usb recieved buffer using the
                // usb_msg_get() function
                USB_Msg_Get();  // removes the first character from the received buffer,
                                // we already know it was a * so no need to save it as a
                                // variable

                // Build a meaningful structure to put your data in. Here we want two
                // floats.
                struct __attribute__( ( __packed__ ) ) {
                    float v1;
                    float v2;
                } data;

                // Copy the bytes from the usb receive buffer into our structure so we
                // can use the information
                USB_Msg_Read_Into( &data, sizeof( data ) );

                // Call MEGN540_Lab_Task Function
                Subtract_And_Send( data.v1, data.v2 );
                // /* MEGN540 -- LAB 2 */ 
                command_processed = true;
            }
            break;

        case '~':
            if( USB_Msg_Length() >= _Message_Length( '~' ) ) {
                // then process your reset by setting the task_restart flag defined in Lab1_Tasks.h
                Task_Activate(&task_restart);
                
                // /* MEGN540 -- LAB 2 */ 
                command_processed = true;
            }
            break;
        case 't':
            if (USB_Msg_Length() >= _Message_Length( 't' ) ) {
                
                // remove the command from the usb recieved buffer using the
                // usb_msg_get() function
                USB_Msg_Get();

                // Build a meaningful structure to put your data in.
                struct __attribute__( ( __packed__ ) ) {
                    char task;
                } data;

                // Copy the bytes from the usb receive buffer into our structure so we
                // can use the information
                USB_Msg_Read_Into( &data, sizeof( data ) );

                switch( data.task ) {
                    // Activate the send time task if the input is 0
                    case '0':
                        // Set the run period to -1 since 't' should only run once
                        task_send_time.run_period = -1;
                        Task_Activate(&task_send_time);
                        command_processed = true;
                        break;

                    // Activate the loop time task if the input is 1
                    case '1':
                        // Set the run period to -1 since 't' should only run once
                        task_time_loop.run_period = -1;
                        Task_Activate(&task_time_loop);
                        command_processed = true;
                        break;

                    // Handle undefined behavior
                    default:
                        USB_Send_Msg( "cc", command, &q_mark, sizeof(q_mark));
                        command_processed = true;
                        break;
                }
            }
            break;
        case 'T':
            if (USB_Msg_Length() >= _Message_Length( 'T' ) ) {

                // remove the command from the usb recieved buffer using the
                // usb_msg_get() function
                USB_Msg_Get();

                // Build a meaningful structure to put your data in.
                struct __attribute__( ( __packed__ ) ) {
                    char action;
                    float period;
                } data;

                // Copy the bytes from the usb receive buffer into our structure so we
                // can use the information
                USB_Msg_Read_Into( &data, sizeof( data ) );
                
                switch( data.action ) {
                    // Activate the send time task if the input is 0
                    case '0':
                        // Set the task's run period to the period specified in the input
                        task_send_time.run_period = data.period;
                        Task_Activate(&task_send_time);
                        command_processed = true;
                        break;

                    // Activate the loop time task if the input is 1
                    case '1':
                        // Set the task's run period to the period specified in the input
                        task_time_loop.run_period = data.period;
                        Task_Activate(&task_time_loop);
                        command_processed = true;
                        break;

                    // Handle undefined behavior
                    default:
                        USB_Send_Msg( "cc", command, &q_mark, sizeof(q_mark));
                        command_processed = true;
                        break;
                }
            }
            break;
            
	    case 'e':
            if( USB_Msg_Length() >= _Message_Length( 'e' ) ) {

                USB_Msg_Get();  // Possibly unnecessary, but I think needed to get rid of the 'e'

                Task_Activate( &task_send_encoder_count );
            }
            break;
            
        case 'E':
            if( USB_Msg_Length() >= _Message_Length( 'E' ) ) {

                USB_Msg_Get();

                struct __attribute__( ( __packed__ ) ) {
                    float v1;
                } data;

                USB_Msg_Read_Into( &data, sizeof( data ) );

                task_send_encoder_count.run_period = data.v1;
                Task_Activate( &task_send_encoder_count );
            }
            break;
            
        case 'b':
            if( USB_Msg_Length() >= _Message_Length( 'b' ) ) {

                USB_Msg_Get();  // Possibly unnecessary, but I think needed to get rid of the 'e'

                Task_Activate( &task_send_battery_voltage );
            }
            break;
            
        case 'B':
            if( USB_Msg_Length() >= _Message_Length( 'B' ) ) {

                USB_Msg_Get();

                struct __attribute__( ( __packed__ ) ) {
                    float v1;
                } data;

                USB_Msg_Read_Into( &data, sizeof( data ) );

                task_send_battery_voltage.run_period = data.v1;
                Task_Activate( &task_send_battery_voltage );
            }
            break;
        
        case 'p':
            if (USB_Msg_Length() >= _Message_Length('p')) {
                USB_Msg_Get();

                struct __attribute__ ((__packed__)) {
                    int16_t left;
                    int16_t right;
                } data;

                USB_Msg_Read_Into(&data, sizeof(data));
                
                MotorPWM_Set_Left(data.left);
                MotorPWM_Set_Right(data.right);
                command_processed = true;
            }
            break;
        case 'P':
            if (USB_Msg_Length() >= 1) {
                USB_Msg_Get();

                struct __attribute__ ((__packed__)) {
                    int16_t left;
                    int16_t right;
                    float time_ms;
                } data;

                USB_Msg_Read_Into(&data, sizeof(data));

                MotorPWM_Set_Left(data.left);
                MotorPWM_Set_Right(data.right);

                task_schedule_stop.run_period = data.time_ms;
                Task_Activate(&task_schedule_stop);

                command_processed = true;
            }
            break;

        case 'S':
        case 's':
            if (USB_Msg_Length() >= 1) {
                USB_Msg_Get();

                MotorPWM_Set_Left(0);
                MotorPWM_Set_Right(0);
                command_processed = true;
            }
            break;
        
        case 'q':
            if (USB_Msg_Length() >= _Message_Length('q')) {
                USB_Msg_Get();

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
                command_processed = true;
            }
            break;

        case 'Q':
            if (USB_Msg_Length() >= _Message_Length('Q')) {
                USB_Msg_Get();

                float ms;
                USB_Msg_Read_Into(&ms, sizeof(float));

                task_send_system_data.run_period = ms;
                Task_Activate(&task_send_system_data);
                command_processed = true;
            }
            break;

        case 'd':
            if (USB_Msg_Length() >= _Message_Length('d')) {
                USB_Msg_Get();

                struct __attribute__ ((__packed__)) {
                    float linear;
                    float angular;
                } data;

                USB_Msg_Read_Into(&data, sizeof(data));
                
                Skid_Steer_Command_Displacement(data.linear, data.angular);
                Task_Activate(&task_ctrl);
                command_processed = true;
            }
            break;

        case 'D':
            if (USB_Msg_Length() >= _Message_Length('D')) {
                USB_Msg_Get();

                struct __attribute__ ((__packed__)) {
                    float linear;
                    float angular;
                    float time_ms;
                } data;

                USB_Msg_Read_Into(&data, sizeof(data));
                
                Skid_Steer_Command_Displacement(data.linear, data.angular);
                Task_Activate(&task_ctrl);

                task_ctrl_stop.run_period = data.time_ms;
                Task_Activate(&task_ctrl_stop);

                command_processed = true;
            }
            break;

        case 'v':
            if (USB_Msg_Length() >= _Message_Length('v')) {
                USB_Msg_Get();

                struct __attribute__ ((__packed__)) {
                    float linear;
                    float angular;
                } data;

                USB_Msg_Read_Into(&data, sizeof(data));
                
                Skid_Steer_Command_Velocity(data.linear, data.angular);
                Task_Activate(&task_ctrl);
                command_processed = true;
            }
            break;

        case 'V':
            if (USB_Msg_Length() >= _Message_Length('V')) {
                USB_Msg_Get();

                struct __attribute__ ((__packed__)) {
                    float linear;
                    float angular;
                    float time_ms;
                } data;

                USB_Msg_Read_Into(&data, sizeof(data));
                
                Skid_Steer_Command_Velocity(data.linear, data.angular);
                Task_Activate(&task_ctrl);

                task_ctrl_stop.run_period = data.time_ms;
                Task_Activate(&task_ctrl_stop);

                command_processed = true;
            }
            break;

        default:
            // What to do if you dont recognize the command character
            // remove the command from the usb recieved buffer using the
            // usb_msg_get() function
            USB_Msg_Get();
            USB_Send_Msg( "cc", command, &q_mark, sizeof(q_mark));
            break;
    }

    //********* MEGN540 -- LAB 2 ************//
    if( command_processed ) {
         // RESET the WATCHDOG TIMER
         Task_Activate( &task_message_handling_watchdog );
    }
}

/**
 * @brief Function Task_Message_Handling_Watchdog clears the USB recieve (deleting all messages) to flush the buffer if a complete message is not recieved
 * whithin an appropriate amount of time (say 250ms)
 *
 * @param _unused_
 */
void Task_Message_Handling_Watchdog( float _unused_ )
{
    USB_Flush_Input_Buffer();
}

/**
 * Function _Message_Length returns the number of bytes associated with a
 * command string per the class documentation;
 * @param cmd
 * @return Size of expected string. Returns 0 if unreconized.
 */
static uint8_t _Message_Length( char cmd )
{
    switch( cmd ) {
        case '~': return 1; break;
        case '*': return 9; break;
        case '/': return 9; break;
        case '+': return 9; break;
        case '-': return 9; break;
        case 't': return 2; break;
        case 'T': return 6; break;
        case 'e': return 1; break;
        case 'E': return 5; break;
        case 'b': return 1; break;
        case 'B': return 5; break;
        case 'p': return 5; break;
        case 'P': return 9; break;
        case 's': return 1; break;
        case 'S': return 1; break;
        case 'q': return 1; break;
        case 'Q': return 5; break;
        case 'd': return 9; break;
        case 'D': return 13; break;
        case 'v': return 9; break;
        case 'V': return 13; break;
        default: return 0; break;
    }
}
