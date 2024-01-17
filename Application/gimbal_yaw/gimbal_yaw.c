#include "gimbal_yaw.h"
#include "robot_def.h"
#include "DJI_motor.h"
#include "message_center.h"

static DJIMotor_Instance *yaw_motor;

static Subscriber_t *gimbal_yaw_sub;              // cmd控制消息订阅者
static Gimbal_Ctrl_Yaw_Cmd_s gimbal_yaw_cmd_recv; // 来自cmd的控制信息

/**
 * @brief 初始化云台yaw轴控制任务
 *
 */
void GimbalYawInit(void)
{
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id      = 5,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp            = 15, // 15
                .Ki            = 0,
                .Kd            = 0,
                .IntegralLimit = 100,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 500,
            },
            .speed_PID = {
                .Kp            = 10, // 20
                .Ki            = 1,  // 1
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
            .motor_reverse_flag    = MOTOR_DIRECTION_REVERSE,
        },
        .motor_type = GM6020,
    };

    yaw_motor      = DJIMotorInit(&yaw_config);
    gimbal_yaw_sub = SubRegister("gimbal_yaw_cmd", sizeof(Gimbal_Ctrl_Yaw_Cmd_s));
}

/**
 * @brief 云台yaw轴控制任务
 *
 */
void GimbalYawTask(void)
{
    // 获取云台控制数据
    SubGetMessage(gimbal_yaw_sub, &gimbal_yaw_cmd_recv);
    DJIMotorChangeFeed(yaw_motor,ANGLE_LOOP,OTHER_FEED,&gimbal_yaw_cmd_recv.up_yaw);
    DJIMotorSetRef(yaw_motor, gimbal_yaw_cmd_recv.yaw);
}