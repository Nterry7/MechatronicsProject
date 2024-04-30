#include "Task_Management.h"

#include "Timing.h"

/** Function Initialize_Task initializes the task to a default state that is inactive.
 *  Note that a negative run_period indicates the task should only be performed once, while
 *  a run_period of 0 indicates the task should be run every time.
 */
void Initialize_Task( Task_t* task, float run_period, void ( *task_fcn_ptr )( float ) )
{
    task->is_active              = false;
    task->run_period             = run_period;
    task->time_last_ran.microsec = 0;
    task->time_last_ran.millisec = 0;
    task->task_fcn_ptr           = task_fcn_ptr;
}

/** Function Task_Activate changes the internal state to enable the task **/
void Task_Activate( Task_t* task )
{
    // Set the task to be active and update the time of the task to the current time
    task->is_active = true;
    task->time_last_ran.microsec = Timing_Get_Micro();
    task->time_last_ran.millisec = Timing_Get_Milli();
}

/** Function Task_Cancel changes the internal state to disable the task **/
void Task_Cancel( Task_t* task )
{
    // Set the task to inactive
    task->is_active = false;
}

/** Function Task_Is_Ready indicates if the task should be run. It checks both
 * the active status and the timing.
 */
bool Task_Is_Ready( Task_t* task )
{
    // Check if the run period is negative to be run once
    if (task->run_period < 0) {
        return task->is_active;
    }

    // Check if the task should be run based on the last time it was run
    else if (Timing_Seconds_Since(&task->time_last_ran) >= task->run_period) {
        return task->is_active;
    }

    return false; 
}

/** Function Task_Run_If_Ready Function Task_Run_If_Ready checks to see if the given task is ready for execution, executes the task,
 *  and resets the time_last_ran appropriately. If the task function pointer is NULL then it just
 *  returns if the task is ready and resets the time_last_ran.
 */
bool Task_Run_If_Ready( Task_t* task )
{
    //****** MEGN540 --  START IN LAB 1, UPDATE IN Lab 2   ******//
    // Check to see if the task is ready to run.
    // Note that a negative run_period indicates the task should only be performed once, while
    // a run_period of 0 indicates the task should be run every time.
    //
    // Run it if it is ready
    // Update time_last_ran and is_active as appropriate.
    // To call a void functor (function pointer):  (*functor_variable)();
    //bool ready = Task_Is_Ready(task);

    // If the task is ready and the function assiciated with the task is not NULL,
    // run the function associated with the task and update the last time the task
    // was run
    if(Task_Is_Ready(task) && task->task_fcn_ptr){
        task->task_fcn_ptr(task->time_last_ran.microsec * 0.001 + task->time_last_ran.millisec);
        task->time_last_ran.microsec = Timing_Get_Micro();
        task->time_last_ran.millisec = Timing_Get_Milli();
        
        // Cancel the task if it was only meant to run once
        if(task->run_period < 0){
            Task_Cancel(task);
        }

        return true;
    }

    return false;  // true if it ran, false if it did not run
}
