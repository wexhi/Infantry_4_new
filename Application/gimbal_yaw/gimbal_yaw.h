/**
 * @file gimbal_yaw.h
 * @author Bi Kaixiang (wexhicy@gmail.com)
 * @brief 云台yaw轴控制任务
 * @version 0.1
 * @date 2024-01-17
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef __GIMBAL_YAW_H
#define __GIMBAL_YAW_H

/**
 * @brief 初始化云台yaw轴控制任务
 *
 */
void GimbalYawInit(void);

/**
 * @brief 云台yaw轴控制任务
 *
 */
void GimbalYawTask(void);

#endif // !__GIMBAL_YAW_H