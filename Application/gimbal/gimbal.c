#include "gimbal.h"
#include "DJI_motor.h"
#include "robot_def.h"
#include "message_center.h"

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
static DJIMotor_Instance *yaw_motor;
static Subscriber_t *gimbal_yaw_sub;          // cmd控制消息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_yaw_cmd_recv; // 来自cmd的控制信息
#endif

#if defined(GIMBAL_BOARD) || defined(ONE_BOARD)
static DJIMotor_Instance *pitch_motor;
#endif

#ifdef GIMBAL_BOARD
#include "ins_task.h"
#include "C_comm.h"
static Up_To_Down_Data_s up_send_data; // 上板发送给下板的数据
static Down_To_Up_Data_s up_recv_data; // 上板收到的下板数据
static attitude_t *gimba_IMU_data; // 云台IMU数据
static CAN_Comm_Instance *gimbal_can_comm;
#endif //! Only GIMBAL_BOARD ! Only

void GimbalInit(void)
{
#if defined(GIMBAL_BOARD) || defined(ONE_BOARD)
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id      = 5,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp            = 1,
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

    pitch_motor = DJIMotorInit(&pitch_config);
#endif

#ifdef GIMBAL_BOARD
    gimba_IMU_data                   = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    CAN_Comm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id      = 0x311,
            .rx_id      = 0x312,
        },
        .recv_data_len = sizeof(Down_To_Up_Data_s),
        .send_data_len = sizeof(Up_To_Down_Data_s),
    };
    gimbal_can_comm = CANCommInit(&comm_conf);
#endif // GIMBAL_BOARD

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id      = 3,
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
                .Kp            = 20,
                .Ki            = 1,
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

    yaw_motor = DJIMotorInit(&yaw_config);

    gimbal_yaw_sub = SubRegister("gimbal_yaw_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
#endif
}

void GimbalTask(void)
{
#ifdef GIMBAL_BOARD
    up_recv_data = *(Down_To_Up_Data_s *)CANCommGet(gimbal_can_comm);
    
    switch(up_recv_data.gimbal_cmd.gimbal_mode) {
        case GIMBAL_ZERO_FORCE:
            DJIMotorStop(pitch_motor);
            break;
        case GIMBAL_FREE_MODE:
            DJIMotorEnable(pitch_motor);
            DJIMotorSetRef(pitch_motor, up_recv_data.gimbal_cmd.pitch);
            break;
        default:
            break;
    }
#endif // GIMBAL_BOARD

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    SubGetMessage(gimbal_yaw_sub, &gimbal_yaw_cmd_recv);

    switch (gimbal_yaw_cmd_recv.gimbal_mode) {
        case GIMBAL_ZERO_FORCE:
            DJIMotorStop(yaw_motor);
            break;
        // 云台自由模式, 使用编码器反馈, 底盘和云台分离, 仅云台旋转, 一般用于调整云台姿态(英雄吊射等) / 能量机关
        case GIMBAL_FREE_MODE:
            DJIMotorEnable(yaw_motor);
            DJIMotorSetRef(yaw_motor, gimbal_yaw_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
            break;

        default:
            break;
    }
#endif

#ifdef GIMBAL_BOARD
    up_send_data.yaw = gimba_IMU_data->Yaw;
    CANCommSend(gimbal_can_comm, (void *)&up_send_data);
#endif // GIMBAL_BOARD
}