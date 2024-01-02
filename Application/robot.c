# include "robot.h"
# include "roboTask.h"


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
    // 测试代码

    // BSP初始化
    
    // 应用层初始化

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

}