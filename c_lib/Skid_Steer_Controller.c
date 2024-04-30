#include "Skid_Steer_Controller.h"

void Initialize_Skid_Steer( Skid_Steer_Controller_t* p_skid_steer_cntr, float* z_transform_numerator, float* z_transform_denominator, uint8_t z_transform_order,
                            float wheel_base_width, float wheel_diameter, float kp, float max_abs_control, float update_period, float ( *measurement_left_fcn_ptr )( void ),
                            float ( *measurement_right_fcn_ptr )( void ), void ( *control_left_fcn_ptr )( int16_t ), void ( *control_right_fcn_ptr )( int16_t ) ) {
    
    Initialize_Controller(&p_skid_steer_cntr->controller_left, kp, z_transform_numerator, z_transform_denominator, z_transform_order, update_period);
    Initialize_Controller(&p_skid_steer_cntr->controller_right, kp, z_transform_numerator, z_transform_denominator, z_transform_order, update_period);
    
    p_skid_steer_cntr->wheel_base_width = wheel_base_width;
    p_skid_steer_cntr->wheel_diameter = wheel_diameter;
    p_skid_steer_cntr->kp = kp;
    p_skid_steer_cntr->max_abs_control = max_abs_control;
    p_skid_steer_cntr->measurement_left_fcn_ptr = measurement_left_fcn_ptr;
    p_skid_steer_cntr->measurement_right_fcn_ptr = measurement_right_fcn_ptr;
    p_skid_steer_cntr->control_left_fcn_ptr = control_left_fcn_ptr;
    p_skid_steer_cntr->control_right_fcn_ptr = control_right_fcn_ptr;
}

void Skid_Steer_Command_Displacement( float linear, float angular ) {
    Skid_Controller.controller_left.target_pos += 2 * ((linear - angular * Skid_Controller.wheel_base_width) / Skid_Controller.wheel_diameter);
    Skid_Controller.controller_right.target_pos += 2 * ((linear + angular * Skid_Controller.wheel_base_width) / Skid_Controller.wheel_diameter);
}

void Skid_Steer_Command_Velocity( float linear, float angular ) {
    float v_l = (linear - angular * (Skid_Controller.wheel_base_width / 2));
    float v_r = (linear + angular * (Skid_Controller.wheel_base_width / 2));

    Controller_Set_Target_Velocity(&Skid_Controller.controller_left, v_l);
    Controller_Set_Target_Velocity(&Skid_Controller.controller_right, v_r);
    Skid_Controller.controller_left.target_pos = Skid_Controller.measurement_left_fcn_ptr();
    Skid_Controller.controller_right.target_pos = Skid_Controller.measurement_right_fcn_ptr();
}

void Skid_Steer_Control_Update( float ellapsed_time ) {
    float left_measurement = Skid_Controller.measurement_left_fcn_ptr();
    float right_measurement = Skid_Controller.measurement_right_fcn_ptr();

    float left_ctrl_val = Controller_Update(&Skid_Controller.controller_left, left_measurement, ellapsed_time);
    float right_ctrl_val = Controller_Update(&Skid_Controller.controller_right, right_measurement, ellapsed_time);

    int16_t left_pwm = Saturate(left_ctrl_val, Skid_Controller.max_abs_control);
    int16_t right_pwm = Saturate(right_ctrl_val, Skid_Controller.max_abs_control);
    Skid_Controller.control_left_fcn_ptr(left_pwm);
    Skid_Controller.control_right_fcn_ptr(right_pwm);
    
}

