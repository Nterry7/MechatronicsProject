#include "Lab5_Tasks.h"

void Task_Ctrl(float _time_since_last){
    Skid_Steer_Control_Update(_time_since_last);
}

void Task_Ctrl_Stop(float _time_since_last){
    Controller_Set_Target_Position(&Skid_Controller.controller_left, Skid_Controller.measurement_left_fcn_ptr());
    Controller_Set_Target_Position(&Skid_Controller.controller_right, Skid_Controller.measurement_left_fcn_ptr());
}