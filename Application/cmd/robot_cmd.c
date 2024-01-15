/**
 * @file robot_cmd.c
 * @author your name (you@domain.com)
 * @brief 机器人核心控制任务
 * @attention 因为底盘接的是遥控器，但是云台需要进行视觉传输，因此在预编译时不应该屏蔽掉RobotCMDInit，
 *             否则会出现内存访问错误，应在该文件中修改预编译指令。
 * @version 0.1
 * @date 2024-01-15
 *
 * @copyright Copyright (c) 2024
 *
 */
// application layer for robot command
#include "robot_cmd.h"
#include "robot_def.h"

// module layer
#include "remote.h"
#include "miniPC_process.h"
#include "message_center.h"
/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef CHASSIS_BOARD // 对双板的兼容,条件编译
// 初始化底盘控制消息发布者
#include "C_comm.h"
static CAN_Comm_Instance *cmd_can_comm; // 双板通信
#endif


#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
static void RemoteControlSet(void); // 遥控器控制量设置
static RC_ctrl_t *rc_data;                    // 遥控器数据指针,初始化时返回
static Publisher_t *chassis_cmd_pub;          // 底盘控制消息发布者
static Chassis_Ctrl_Cmd_s chassis_cmd_send;   // 传递给底盘的控制信息
static Publisher_t *gimbal_yaw_cmd_pub;       // 云台控制消息发布者
static Gimbal_Ctrl_Cmd_s gimbal_yaw_cmd_send; // 传递给云台的控制信息
#endif

#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
static Vision_Recv_s *vision_recv_data; // 视觉接收数据指针,初始化时返回
#endif
/**
 * @brief 机器人核心控制任务初始化,会被RobotInit()调用
 *
 */
void RobotCMDInit(void)
{
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD) // 底盘接上遥控器
    // 初始化遥控器,使用串口3
    rc_data = RemoteControlInit(&huart3); // 初始化遥控器,C板上使用USART3
    // 初始化底盘控制消息发布者
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    // 初始化云台控制消息发布者
    gimbal_yaw_cmd_pub = PubRegister("gimbal_yaw_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
#endif

#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    // 初始化视觉接收,使用串口6
    Vision_Init_Config_s vision_init_config = {
        .recv_config = {
            .header = VISION_RECV_HEADER,
        },
        .send_config = {
            .header        = VISION_SEND_HEADER,
            .detect_color  = VISION_DETECT_COLOR_RED,
            .reset_tracker = VISION_RESET_TRACKER_YES,
            .is_shoot      = VISION_SHOOTING,
            .tail          = VISION_SEND_TAIL,
        },
        .usart_config = {
            .recv_buff_size = VISION_RECV_SIZE,
            .usart_handle   = &huart1,
        },

    };
    vision_recv_data = VisionInit(&vision_init_config);
#endif
#ifdef CHASSIS_BOARD
    // 初始化底盘控制消息发布者
    CAN_Comm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id      = 0x312,
            .rx_id      = 0x311,
        },
        .recv_data_len = 0,
        .send_data_len = sizeof(Gimbal_Ctrl_Cmd_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);
#endif
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask(void)
{
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    RemoteControlSet(); // 遥控器控制量设置

    // 发布底盘控制消息
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
    PubPushMessage(gimbal_yaw_cmd_pub, (void *)&gimbal_yaw_cmd_send);
#endif
#ifdef CHASSIS_BOARD
    // test
    gimbal_yaw_cmd_send.yaw+=0.01f;
    gimbal_yaw_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    gimbal_yaw_cmd_send.pitch-=0.1f;
    CANCommSend(cmd_can_comm, (void *)&gimbal_yaw_cmd_send);
#endif
}

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet(void)
{
    // 底盘参数,目前没有加入小陀螺(调试似乎暂时没有必要),系数需要调整
    if (switch_is_up(rc_data[TEMP].rc.switch_right)) {

    } else if (switch_is_mid(rc_data[TEMP].rc.switch_right)) {
        chassis_cmd_send.chassis_mode   = CHASSIS_NO_FOLLOW;
        gimbal_yaw_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
    } else if (switch_is_down(rc_data[TEMP].rc.switch_right)) {
        chassis_cmd_send.chassis_mode   = CHASSIS_ZERO_FORCE;
        gimbal_yaw_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
    }

    chassis_cmd_send.vx = 10.0f * (float)rc_data[TEMP].rc.rocker_l_; // _水平方向
    chassis_cmd_send.vy = 10.0f * (float)rc_data[TEMP].rc.rocker_l1; // 1数值方向
    chassis_cmd_send.wz = 10.0f * (float)rc_data[TEMP].rc.dial;      // _水平方向

    // 云台参数
    // 按照摇杆的输出大小进行角度增量,增益系数需调整
    gimbal_yaw_cmd_send.yaw += 0.002f * (float)rc_data[TEMP].rc.rocker_r_;
}
#endif