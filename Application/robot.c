#include "robot.h"
#include "roboTask.h"
#include "robot_def.h"
#include "robot_cmd.h"
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
#include "chassis.h"
#include "gimbal_yaw.h"
#endif

#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
#include "gimbal_pitch.h"
// #include "shoot.h"
#endif

// #include "gimbal.h"

#include "bsp_init.h"

// 编译warning,提醒开发者修改机器人参数
#ifndef ROBOT_DEF_PARAM_WARNING
#define ROBOT_DEF_PARAM_WARNING
#pragma message "check if you have configured the parameters in robot_def.h, IF NOT, please refer to the comments AND DO IT, otherwise the robot will have FATAL ERRORS!!!"
#endif // !ROBOT_DEF_PARAM_WARNING

/**
 * @brief 机器人初始化
 *
 */
void RobotInit(void)
{
    // 关闭中断,防止在初始化过程中发生中断
    // 请不要在初始化过程中使用中断和延时函数！
    // 若必须,则只允许使用DWT_Delay()
    __disable_irq();
    // BSP初始化
    BSPInit();
    // 应用层初始化
    RobotCMDInit();
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    // 云台pitch轴初始化
    GimbalPitchInit();
    // 发射机构初始化
#endif

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    GimbalYawInit(); // 云台yaw轴初始化
    ChassisInit(); // 底盘初始化
#endif
    
    // GimbalInit(); // 云台初始化，因为云台上板控制的是Pitch轴，但是云台下板控制的是Yaw轴，因此云台初始化任务必须执行

    // rtos创建任务
    OSTaskInit();
    // 初始化完成,开启中断
    __enable_irq();
}

/**
 * @brief 机器人任务入口
 *
 */
void RobotTask()
{
    // 应用层任务
    RobotCMDTask();
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    // 底盘任务
    ChassisTask();
    // 云台yaw轴任务
    GimbalYawTask();
#endif

#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    // 云台pitch轴任务
    GimbalPitchTask();
    // 发射机构任务
#endif
    // GimbalTask();
    // 测试代码
}

/*  下面为测试代码,可忽略    */
