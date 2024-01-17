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
            .tx_id      = 3,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp            = 20, // 8
                .Ki            = 0,
                .Kd            = 0,
                .DeadBand      = 4,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,

                .MaxOut = 360,
            },
            .speed_PID = {
                .Kp            = 25,  // 50
                .Ki            = 0, // 200
                .Kd            = 0,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 3000,
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

    switch (gimbal_yaw_cmd_recv.gimbal_mode) {
        case GIMBAL_ZERO_FORCE:
            DJIMotorStop(yaw_motor);
            break;
        // 云台自由模式, 使用编码器反馈, 底盘和云台分离, 仅云台旋转, 一般用于调整云台姿态(英雄吊射等) / 能量机关
        case GIMBAL_FREE_MODE:
            DJIMotorEnable(yaw_motor);
            DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, MOTOR_FEED, NULL);
            DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, MOTOR_FEED, NULL);
            DJIMotorOuterLoop(yaw_motor, SPEED_LOOP);
            DJIMotorSetRef(yaw_motor, gimbal_yaw_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
            break;
        case GIMBAL_GYRO_MODE:
            DJIMotorEnable(yaw_motor);
            DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED, &gimbal_yaw_cmd_recv.up_yaw);
            // DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED, &gimbal_yaw_cmd_recv.up_speed);
            DJIMotorSetRef(yaw_motor, gimbal_yaw_cmd_recv.yaw);
    }
}