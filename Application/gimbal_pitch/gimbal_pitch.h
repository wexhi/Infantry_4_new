/**
 * @file gimbal_pitch.h
 * @author Bi Kaixiang (wexhicy@gmail.com)
 * @brief   云台pitch轴控制任务
 * @version 0.1
 * @date 2024-01-17
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef __GIMBAL_PITCH_H
#define __GIMBAL_PITCH_H

/**
 * @brief 初始化云台pitch轴控制任务
 *
 */
void GimbalPitchInit(void);

/**
 * @brief 云台pitch轴控制任务
 *
 */
void GimbalPitchTask(void);

#endif // !__GIMBAL_PITCH_H