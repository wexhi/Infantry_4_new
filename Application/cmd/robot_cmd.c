// application layer for robot command
#include "robot_cmd.h"

// module layer
#include "miniPC_process.h"

#include "led.h"
static void VisionRecvCallback(void);

static Vision_Recv_s *vision_recv_data; // 视觉接收数据指针,初始化时返回

/**
 * @brief 机器人核心控制任务初始化,会被RobotInit()调用
 *
 */
void RobotCMDInit(void)
{
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
            .module_callback = VisionRecvCallback,
            .recv_buff_size  = VISION_RECV_SIZE,
            .usart_handle    = &huart6,
        },

    };
    vision_recv_data = VisionInit(&vision_init_config);
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask(void)
{
    VisionSend();
}

/*********    下面为测试代码      **********/
static void VisionRecvCallback(void)
{
    LEDSet(LED_COLOR_R, 255, 255);
}