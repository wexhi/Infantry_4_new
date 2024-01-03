// application layer for robot command
#include "robot_cmd.h"

// module layer
#include "miniPC_process.h"

static Vision_Recv_s *vision_recv_data; // 视觉接收数据指针,初始化时返回


/**
 * @brief 机器人核心控制任务初始化,会被RobotInit()调用
 *
 */
void RobotCMDInit(void)
{
    vision_recv_data = VisionInit(&huart6);
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask(void)
{
    VisionSend();
}
