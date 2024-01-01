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
    led->pwm = PWMRegister(&config->pwm_config);
    led->state = config->init_state;

    led_instances[idx++] = led;
    return led;
}