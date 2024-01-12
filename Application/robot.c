#include "robot.h"
#include "roboTask.h"
#include "test_motor.h"
#include "robot_cmd.h"
#include "chassis.h"
#include "gimbal.h"

#include "bsp_init.h"

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
    ChassisInit(); // 底盘初始化
    GimbalInit();  // 云台初始化
    TestInit();
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
    ChassisTask();
    GimbalTask();
    // 测试代码
    // TestTask();
}

/*  下面为测试代码,可忽略    */

// static void AAAAA()
// {
//     LEDSet(LED_COLOR_B, 255, 255);
// }
