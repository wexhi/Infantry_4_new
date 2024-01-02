#include "robot.h"
#include "roboTask.h"

static void TestInit(void);

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
    TestInit();
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

/*  下面为测试代码,可忽略    */

static void AAAAA()
{
    LEDSet(LED_COLOR_B,255,255);
}

static void TestInit(void)
{
    KEY_Config_s key_config = {
        .gpio_config = {
            .GPIOx     = GPIOA,
            .GPIO_Pin  = GPIO_PIN_0,
            .pin_state = GPIO_PIN_RESET,
            .exti_mode = GPIO_MODE_INPUT,
        },
        .init_state = 0,
    };
    key_config.gpio_config.gpio_model_callback = AAAAA;
    KEYRegister(&key_config);
}
