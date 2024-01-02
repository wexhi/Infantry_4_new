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
 * @ note 初始化顺序BGR
 */
void LEDInit(void)
{
    LED_Config_s led_config = {
        .pwm_config = {
            .htim      = &htim5,
            .channel   = TIM_CHANNEL_1,
            .period    = 0.001,
            .dutycycle = 0,
            .callback  = NULL,
            .id        = NULL,
        },
        .init_state = LED_ON,
    };
    LEDRegister(&led_config);

    led_config.pwm_config.channel = TIM_CHANNEL_2;
    LEDRegister(&led_config);

    led_config.pwm_config.channel = TIM_CHANNEL_3;
    LEDRegister(&led_config);
}

/**
 * @brief 设置LED状态
 *
 * @param color 0-2 0:B 1:G 2:R
 * @param state 0:关闭 1:打开
 */
void LEDSetState(uint8_t color, uint8_t state)
{
    LED_Instance *_led = led_instances[color];
    _led->state        = state;
}

/**
 * @brief 设置LED颜色,亮度
 *
 * @param color 0-2 0:B 1:G 2:R
 * @param color_value 0-255
 * @param brightness 0-255
 */
void LEDSet(uint8_t color, uint8_t color_value, uint8_t brightness)
{
    LED_Instance *_led = led_instances[color];
    _led->color        = color_value;
    _led->brightness   = brightness;
}

/**
 * @brief LED显示函数
 *
 */
void LEDTask(void)
{
    LED_Instance *led;
    for (uint8_t i = 0; i < idx; i++) 
    {
        led = led_instances[i];
        if (led->state) {
            PWMSetDutyRatio(led->pwm, led->color * led->brightness / 65025.0);
        } else {
            PWMSetDutyRatio(led->pwm, 0);
        }
    }
}