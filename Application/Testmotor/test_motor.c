#include "test_motor.h"
#include "DJI_motor.h"

static DJIMotor_Instance *yaw_motor;
static float init_angle;
static uint8_t flag;
static int16_t speed;
void TestInit(void)
{
    Motor_Init_Config_s motor_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id      = 5,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp            = 15,
                .Ki            = 0,
                .Kd            = 0,
                .IntegralLimit = 100,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 500,
            },
            .speed_PID = {
                .Kp            = 11,
                .Ki            = 3,
                .Kd            = 0,
                .IntegralLimit = 3000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 20000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020,
    };
    yaw_motor = DJIMotorInit(&motor_config);
}

void TestTask(void)
{
    if (!flag) {
        init_angle = yaw_motor->measure.total_angle;
        flag       = 1;
    }
    speed = yaw_motor->measure.speed_aps;
    DJIMotorSetRef(yaw_motor, init_angle + 3600);
}