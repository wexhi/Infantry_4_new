#include "led.h"
#include "stdlib.h"
#include "memory.h"

static uint8_t idx;
static LED_Instance *led_instances[LED_MAX_NUM] = {NULL}; // 一个指针数组，用于存放LED实例的指针

/**
 * @brief LED注册
 *
 * @param config
 * @return LED_Instance*
 */
LED_Instance *LEDRegister(LED_Config_s *config)
{
    LED_Instance *led = (LED_Instance *)malloc(sizeof(LED_Instance));
    memset(led, 0, sizeof(LED_Instance)); // 清零,防止原先的地址有脏数据

    // 初始化pwm
    led->pwm   = PWMRegister(&config->pwm_config);
    led->state = config->init_state;

    led_instances[idx++] = led;
    return led;
}

/**
 * @brief 所有LED初始化
 *
 */
void LEDInit(void)
{
    LED_Config_s led_config = {
        .pwm_config = {
            .htim      = &htim5,
            .channel   = TIM_CHANNEL_1,
            .period    = 0.001,
            .dutycycle = 0.5,
            .callback  = NULL,
            .id        = NULL,
        },
        .init_state = 0,
    };
    LEDRegister(&led_config);

    led_config.pwm_config.channel = TIM_CHANNEL_2;
    LEDRegister(&led_config);

    led_config.pwm_config.channel = TIM_CHANNEL_3;
    LEDRegister(&led_config);
}

void LEDShow(uint32_t aRGB)
{
    static uint8_t alpha;
    static uint16_t red, green, blue;

    alpha = (aRGB & 0xFF000000) >> 24;
    red   = ((aRGB & 0x00FF0000) >> 16) * alpha;
    green = ((aRGB & 0x0000FF00) >> 8) * alpha;
    blue  = ((aRGB & 0x000000FF) >> 0) * alpha;

    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, blue);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, green);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, red);
}