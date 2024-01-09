// application layer for robot command
#include "robot_cmd.h"
#include "robot_def.h"

// module layer
#include "remote.h"
#include "miniPC_process.h"
#include "message_center.h"

static Publisher_t *gimbal_cmd_pub; // 云台控制消息发布者

static Gimbal_Ctrl_Cmd_s gimbal_cmd_send; // 传递给云台的控制信息

static RC_ctrl_t *rc_data;              // 遥控器数据指针,初始化时返回
static Vision_Recv_s *vision_recv_data; // 视觉接收数据指针,初始化时返回

/**
 * @brief 机器人核心控制任务初始化,会被RobotInit()调用
 *
 */
void RobotCMDInit(void)
{
    rc_data = RemoteControlInit(&huart3); // 初始化遥控器,C板上使用USART3

    // 初始化视觉接收,使用串口6
    Vision_Init_Config_s vision_init_config = {
        .recv_config = {
            .header     = VISION_HEADER,
            .tracking   = VISION_TRACKING,
            .id         = VISION_OUTPOST,
            .armors_num = VISION_ARMORS_NUM_BALANCE,
            .reserved   = 0x00,
            .tail       = VISION_TAIL,
        },
        .send_config = {
            .header        = VISION_HEADER,
            .detect_color  = VISION_DETECT_COLOR_RED,
            .reset_tracker = VISION_RESET_TRACKER_NO,
            .reserved      = 0xff,
            .tail          = VISION_TAIL,
        },
        .usart_config = {
            .recv_buff_size = VISION_RECV_SIZE,
            .usart_handle   = &huart6,
        },

    };
    vision_recv_data = VisionInit(&vision_init_config);

    // 初始化云台控制消息发布者
    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask(void)
{
    // 测试
    gimbal_cmd_send.yaw = vision_recv_data->yaw;
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
}

/*********    下面为测试代码      **********/
