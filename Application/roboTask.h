/**
 * @file roboTask.h
 * @author Bi Kaixiang (wexhicy@gmail.com)
 * @brief 机器人RTOS任务
 * @version 0.1
 * @date 2024-01-02
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "robot.h"
#include "led.h"
#include "key.h"

#include "bsp_usart.h"

osThreadId daemonTaskHandle;

void StartDAEMONTASK(void const *argument);

/**
 * @brief 初始化机器人RTOS任务，所有外加的RTOS任务都在这里初始化
 *
 */
void OSTaskInit(void)
{
    osThreadDef(daemontask, StartDAEMONTASK, osPriorityNormal, 0, 128);
    daemonTaskHandle = osThreadCreate(osThread(daemontask), NULL);
}

/**
 * @brief 守护线程任务,100Hz,相当于看门狗
 *
 */
__attribute__((noreturn)) void StartDAEMONTASK(void const *argument)
{
    // 初始化LED外设
    LEDInit();
    for (;;) {
        LEDTask();
        osDelay(10);
    }
}