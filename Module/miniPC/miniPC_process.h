/**
 * @file miniPC_process.h
 * @author Bi Kaixiang (wexhicy@gmail.com)
 * @brief   用于处理miniPC的数据，包括解析和发送
 * @version 0.1
 * @date 2024-01-03
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef MINIPC_PROCESS_H
#define MINIPC_PROCESS_H

#include "stdint.h"
#include "bsp_usart.h"

#define VISION_HEADER    0xA5u // 头帧校验位
#define VISION_TAIL      0x5Au // 尾帧校验位

#define VISION_RECV_SIZE 18u // 当前为固定值,18字节
#define VISION_SEND_SIZE 36u

#pragma pack(1) // 1字节对齐

/* 是否追踪 */
typedef enum {
    VISION_NO_TRACKING = 0u,
    VISION_TRACKING    = 1u,
} VISION_TRACKING_e;

/* 目标ID */
typedef enum {
    VISION_OUTPOST = 0u,
    VISION_GUARD   = 6u,
    VISION_BASE    = 7u,
} VISION_ID_e;

/* 装甲板数量 */
typedef enum {
    VISION_ARMORS_NUM_BALANCE = 2u,
    VISION_ARMORS_NUM_OUTPOST = 3u,
    VISION_ARMORS_NUM_NORMAL  = 4u,
} VISION_ARMORS_NUM_e;

/* 敌方装甲板颜色 */
typedef enum {
    VISION_DETECT_COLOR_RED  = 0u,
    VISION_DETECT_COLOR_BLUE = 1u,
} VISION_DETECT_COLOR_e;

/* 是否重置追踪 */
typedef enum {
    VISION_RESET_TRACKER_NO  = 0u,
    VISION_RESET_TRACKER_YES = 1u,
} VISION_RESET_TRACKER_e;

/* 视觉通信初始化接收结构体 */
typedef struct
{
    uint8_t header;     // 头帧校验位
    uint8_t tracking;   // 是否在追踪
    uint8_t id;         // 0-outpost 6-guard 7-base  目标序号
    uint8_t armors_num; // 一共有多少块装甲板 2-balance平衡 3-outpost前哨战 4-normal正常
    uint8_t reserved;   // 没用
    uint8_t tail;       // 尾帧校验位，不记得是多少了，和电控对一下
} Vision_Recv_Init_Config_s;

/* 视觉通信初始化发送结构体 */
typedef struct
{
    uint8_t header;        // 头帧校验位
    uint8_t detect_color;  // 0-red 1-blue
    uint8_t reset_tracker; // 是否需要重置reset
    uint8_t reserved;      // 没用
    uint8_t tail;          // 尾帧校验位，不记得是多少了，和电控对一下
} Vision_Send_Init_Config_s;

/* minipc -> stm32 (接收结构体) */
typedef struct
{
    uint8_t header;     // 头帧校验位
    uint8_t tracking;   // 是否在追踪
    uint8_t id;         // 0-outpost 6-guard 7-base  目标序号
    uint8_t armors_num; // 一共有多少块装甲板 2-balance平衡 3-outpost前哨战 4-normal正常
    uint8_t reserved;   // 没用
    float x;            // 目标中心x
    float y;            // 目标中心y
    float z;            // 目标中心z
    float yaw;          // 目标转角yaw
    float vx;           // 目标中心x方向速度
    float vy;           // 目标中心y方向速度
    float vz;           // 目标中心z方向速度
    float v_yaw;        // 目标yaw转速
    float r1;           // 目标半径1
    float r2;           // 目标半径2
    float dz;           // 两侧装甲板高度差
    uint8_t tail;       // 尾帧校验位，不记得是多少了，和电控对一下
} Vision_Recv_s;

/* stm32 -> minipc (发送结构体) */
typedef struct
{
    uint8_t header;        // 头帧校验位
    uint8_t detect_color;  // 0-red 1-blue
    uint8_t reset_tracker; // 是否需要重置reset
    uint8_t reserved;      // 没用
    float roll;            // 陀螺仪roll值
    float pitch;           // 陀螺仪pitch值
    float yaw;             // 陀螺仪yaw值
    float aim_x;           // 在经过下位机弹道补偿后的目标的x
    float aim_y;           // 在经过下位机弹道补偿后的目标的y
    float aim_z;           // 在经过下位机弹道补偿后的目标的z
    uint8_t tail;          // 尾帧校验位，不记得是多少了，和电控对一下
} Vision_Send_s;

/* 视觉通信模块实例 */
typedef struct
{
    Vision_Recv_s *recv_data; // 接收数据结构体指针
    Vision_Send_s *send_data; // 发送数据结构体指针
    USART_Instance *usart;    // 串口实例指针
} Vision_Instance;

#pragma pack() // 取消1字节对齐

/**
 * @brief 用于注册一个视觉接收数据结构体,返回一个视觉接收数据结构体指针
 *
 * @param recv_config
 * @return Vision_Recv_s*
 */
Vision_Recv_s *VisionRecvRegister(Vision_Recv_Init_Config_s *recv_config);

/**
 * @brief 调用此函数初始化和视觉的串口通信
 *
 * @param handle 用于和视觉通信的串口handle(C板上一般为USART1,丝印为USART2,4pin)
 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle);

/**
 * @brief 发送函数
 *
 *
 */
void VisionSend();

#endif // MINIPC_PROCESS_H