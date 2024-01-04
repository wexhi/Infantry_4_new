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
#include "daemon.h"

#include "bsp_usart.h"
#include "bsp_dwt.h"

osThreadId robotTaskHandle;
osThreadId daemonTaskHandle;

void StartROBOTTASK(void const *argument);
void StartDAEMONTASK(void const *argument);

/**
 * @brief 初始化机器人RTOS任务，所有外加的RTOS任务都在这里初始化
 *
 */
void OSTaskInit(void)
{
    osThreadDef(robottask, StartROBOTTASK, osPriorityNormal, 0, 1024);
    robotTaskHandle = osThreadCreate(osThread(robottask), NULL);

    osThreadDef(daemontask, StartDAEMONTASK, osPriorityNormal, 0, 128);
    daemonTaskHandle = osThreadCreate(osThread(daemontask), NULL);
}

/**
 * @brief 机器人任务入口
 *
 */
__attribute__((noreturn)) void StartROBOTTASK(void const *argument)
{
    // 200Hz-500Hz,若有额外的控制任务如平衡步兵可能需要提升至1kHz
    for (;;) {
        RobotTask();
        osDelay(5);
    }
}

/**
 * @brief 守护线程任务,100Hz,相当于看门狗
 *
 */
__attribute__((noreturn)) void StartDAEMONTASK(void const *argument)
{
    static float daemon_dt;
    static float daemon_start;
    // 初始化LED外设
    LEDInit();
    for (;;) {
        // 100Hz
        daemon_start = DWT_GetTimeline_ms();
        DaemonTask();
        LEDTask();
        daemon_dt = DWT_GetTimeline_ms() - daemon_start;
        osDelay(10);
    }
}