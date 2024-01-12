#include "robot_def.h"

#include "test_motor.h"
#include "DJI_motor.h"
#include "message_center.h"

static DJIMotor_Instance *test_motor;

void TestInit(void)
{
    Motor_Init_Config_s motor_config = {
        // .can_init_config = {
        //     .can_handle = &hcan1,
        //     .tx_id      = 5,
        // },
        // .controller_param_init_config = {
        //     .angle_PID = {
        //         .Kp            = 15,
        //         .Ki            = 0,
        //         .Kd            = 0,
        //         .IntegralLimit = 100,
        //         .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
        //         .MaxOut        = 500,
        //     },
        //     .speed_PID = {
        //         .Kp            = 11,
        //         .Ki            = 3,
        //         .Kd            = 0,
        //         .IntegralLimit = 3000,
        //         .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
        //         .MaxOut        = 20000,
        //     },
        // },
        // .controller_setting_init_config = {
        //     .angle_feedback_source = MOTOR_FEED,
        //     .speed_feedback_source = MOTOR_FEED,
        //     .outer_loop_type       = ANGLE_LOOP,
        //     .close_loop_type       = SPEED_LOOP | ANGLE_LOOP,
        //     .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
        // },
        // .motor_type = GM6020,
    };
    test_motor = DJIMotorInit(&motor_config);
}

void TestTask(void)
{
}