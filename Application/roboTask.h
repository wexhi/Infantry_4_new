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
#include "robot_def.h"
#include "led.h"
#include "key.h"
#include "daemon.h"
#include "ins_task.h"
#include "miniPC_process.h"
#include "motor_task.h"

#include "bsp_usart.h"
#include "bsp_dwt.h"

osThreadId insTaskHandle;
osThreadId motorTaskHandle;
osThreadId robotTaskHandle;
osThreadId daemonTaskHandle;

void StartINSTASK(void const *argument);
void StartMOTORTASK(void const *argument);
void StartROBOTTASK(void const *argument);
void StartDAEMONTASK(void const *argument);

/**
 * @brief 初始化机器人RTOS任务，所有外加的RTOS任务都在这里初始化
 *
 */
void OSTaskInit(void)
{
    osThreadDef(instask, StartINSTASK, osPriorityRealtime, 0, 1024);
    insTaskHandle = osThreadCreate(osThread(instask), NULL); // 由于是阻塞读取传感器,为姿态解算设置较高优先级,确保以1khz的频率执行

    osThreadDef(motortask, StartMOTORTASK, osPriorityNormal, 0, 512);
    motorTaskHandle = osThreadCreate(osThread(motortask), NULL);

    osThreadDef(robottask, StartROBOTTASK, osPriorityAboveNormal, 0, 1024);
    robotTaskHandle = osThreadCreate(osThread(robottask), NULL);

    osThreadDef(daemontask, StartDAEMONTASK, osPriorityNormal, 0, 128);
    daemonTaskHandle = osThreadCreate(osThread(daemontask), NULL);
}

__attribute__((noreturn)) void StartINSTASK(void const *argument)
{
    static float ins_start;
    static float ins_dt __attribute__((unused)); // for cancel warning
    INS_Init();                                  // 确保BMI088被正确初始化.
    for (;;) {
        // 1kHz
        ins_start = DWT_GetTimeline_ms();
        INS_Task();
        ins_dt = DWT_GetTimeline_ms() - ins_start;
#if (defined(ONE_BOARD) || defined(GIMBAL_BOARD))
        VisionSend(); // 解算完成后发送视觉数据,但是当前的实现不太优雅,后续若添加硬件触发需要重新考虑结构的组织
#endif
        osDelay(1);
    }
}

__attribute__((noreturn)) void StartMOTORTASK(void const *argument)
{
    for (;;) {
        MotorControlTask();
        osDelay(1);
    }
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
    static float daemon_dt __attribute__((unused)); // for cancel warning
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