#include "gimbal_pitch.h"
#include "robot_def.h"
#include "DJI_motor.h"
#include "message_center.h"
#include "ins_task.h"

static DJIMotor_Instance *pitch_motor;
static attitude_t *gimba_IMU_data; // 云台IMU数据指针
static Subscriber_t *gimbal_pitch_sub;          // cmd控制消息订阅者
static Gimbal_Ctrl_Pitch_Cmd_s gimbal_pitch_cmd_recv; // 来自cmd的控制信息

/**
 * @brief 初始化云台pitch轴控制任务
 *
 */
void GimbalPitchInit(void)
{
    gimba_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源

    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id      = 5,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp            = 10,
                .Ki            = 0,
                .Kd            = 0,
                .IntegralLimit = 100,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 500,
            },
            .speed_PID = {
                .Kp            = 10,
                .Ki            = 1,
                .Kd            = 0,
                .IntegralLimit = 3000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 20000,
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->Pitch, // 云台pitch轴使用IMU数据
            .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[0],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020,
    };

    pitch_motor      = DJIMotorInit(&pitch_config);
    gimbal_pitch_sub = SubRegister("gimbal_pitch_cmd", sizeof(Gimbal_Ctrl_Pitch_Cmd_s));
}

/**
 * @brief 云台pitch轴控制任务
 *
 */
void GimbalPitchTask(void)
{
    // 获取云台控制数据
    SubGetMessage(gimbal_pitch_sub, &gimbal_pitch_cmd_recv);
    // 云台pitch轴控制
    DJIMotorSetRef(pitch_motor, gimbal_pitch_cmd_recv.pitch);
}
