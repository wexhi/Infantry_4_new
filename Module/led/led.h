/**
 * @file led.h
 * @author Bi Kaixiang (wexhicy@gmail.com)
 * @brief led灯的模块任务
 * @version 0.1
 * @date 2024-01-02
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef LED_H
#define LED_H

#include "stdint.h"
#include "bsp_pwm.h"

#define LED_MAX_NUM 3 // 最大led数量

/* LED实例结构体 */
typedef struct
{
    PWM_Instance *pwm;    // pwm实例
    uint8_t transparency; // 透明度
    uint8_t brightness;   // 亮度,通过电压改变
    uint8_t color;        // 颜色,rgb value 0~255
    uint8_t state;        // 状态,0:关闭 1:打开
} LED_Instance;

/* LED初始化配置结构体 */
typedef struct
{
    PWM_Config_s pwm_config; // pwm初始化配置
    uint8_t init_state;      // 初始化状态
} LED_Config_s;

/**
 * @brief LED注册
 *
 * @param config
 * @return LED_Instance*
 */
LED_Instance *LEDRegister(LED_Config_s *config);

/**
 * @brief 所有LED初始化
 *
 */
void LEDInit(void);

#endif // LED_H