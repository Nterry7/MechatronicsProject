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

/*
 * This file defines lab-spcific tasks for execution
 */

#ifndef LAB3_TASKS_H
#define LAB3_TASKS_H

#include "Task_Management.h"

// Include your lab-specific headers here
#include "Battery_Monitor.h"  // Battery stuff we wrote for this lab
#include "Encoder.h"          // Encoder stuff we wrote for this lab
#include "SerialIO.h"         // USB interfacing :)

// Put your lab-specific tasks here
Task_t task_send_encoder_count;    // Task to send the current left and right encoder values
Task_t task_send_battery_voltage;  // Task to send the current battery voltage
Task_t task_battery_monitoring;    // Task to send a warning if their is a low battery voltage

// Put your lab-specific task functionality and data_structures (if necessary) here so it is accessable to both
// message handeling and the Lab main loops.
void Send_Encoder_Count( float _unused );
void Send_Battery_Voltage( float _unused );
void Battery_Monitoring( float _unused );

#endif  // ifndef LAB3_TASKS_H
