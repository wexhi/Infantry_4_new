#include "test_motor.h"
#include "DJI_motor.h"

static DJIMotor_Instance *yaw_motor;

void TestInit(void)
{
    Motor_Init_Config_s motor_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id      = 5,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp            = 8,
                .Ki            = 0,
                .Kd            = 0,
                .IntegralLimit = 3000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 200,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = SPEED_LOOP,
            .close_loop_type       = SPEED_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020,
    };
    yaw_motor = DJIMotorInit(&motor_config);
}

void TestTask(void)
{
}