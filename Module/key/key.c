#include "key.h"
#include "stdlib.h"
#include "memory.h"

static uint8_t idx;
static KEY_Instance *key_instances[KEY_MAX_NUM] = {NULL}; // 一个指针数组，用于存放KEY实例的指针

/**
 * @brief 按键注册
 *
 * @param config
 * @return KEY_Instance*
 */
KEY_Instance *KEYRegister(KEY_Config_s *config)
{
    KEY_Instance *key = (KEY_Instance *)malloc(sizeof(KEY_Instance));
    memset(key, 0, sizeof(KEY_Instance)); // 清零,防止原先的地址有脏数据

    // 初始化gpio
    key->gpio  = GPIORegister(&config->gpio_config);
    key->state = config->init_state;

    key_instances[idx++] = key;
    return key;
}