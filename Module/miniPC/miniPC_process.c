#include "miniPC_process.h"
#include "memory.h"

#include "led.h"

static Vision_Instance *vision_instance; // 用于和视觉通信的串口实例

// static Vision_Recv_s recv_data;
// static Vision_Send_s send_data;

static void RecvProcess(Vision_Recv_s *recv, uint8_t *rx_buff)
{
    /* 读取目标颜色，是否重置等数据 */
    recv->tracking   = rx_buff[1];
    recv->id         = rx_buff[2];
    recv->armors_num = rx_buff[3];
    recv->reserved   = rx_buff[4];

    /* 使用memcpy读取浮点型小数 */
    memcpy(&recv->x, &rx_buff[5], 4);
    memcpy(&recv->y, &rx_buff[9], 4);
    memcpy(&recv->z, &rx_buff[13], 4);
    memcpy(&recv->yaw, &rx_buff[17], 4);
    memcpy(&recv->vx, &rx_buff[21], 4);
    memcpy(&recv->vy, &rx_buff[25], 4);
    memcpy(&recv->vz, &rx_buff[29], 4);
    memcpy(&recv->v_yaw, &rx_buff[33], 4);
    memcpy(&recv->r1, &rx_buff[37], 4);
    memcpy(&recv->r2, &rx_buff[41], 4);
    memcpy(&recv->dz, &rx_buff[45], 4);
}

/**
 * @brief 回调函数，确认帧头后用于解析视觉数据
 *
 */
static void DecodeVision(void)
{
    if (vision_instance->usart->recv_buff[0] == vision_instance->recv_data->header) {
        // 读取视觉数据
        RecvProcess(vision_instance->recv_data, vision_instance->usart->recv_buff);

        // 调试用，记得删掉
        LEDSetState(LED_COLOR_G, LED_ON);
        LEDSet(LED_COLOR_G, 255, 255);
    } else {
        LEDSetState(LED_COLOR_G, LED_OFF);
    }
}

/**
 * @brief 用于注册一个视觉接收数据结构体,返回一个视觉接收数据结构体指针
 *
 * @param recv_config
 * @return Vision_Recv_s*
 */
Vision_Recv_s *VisionRecvRegister(Vision_Recv_Init_Config_s *recv_config)
{
    Vision_Recv_s *recv_data = (Vision_Recv_s *)malloc(sizeof(Vision_Recv_s));
    memset(recv_data, 0, sizeof(Vision_Recv_s));

    recv_data->header     = recv_config->header;
    recv_data->tracking   = recv_config->tracking;
    recv_data->id         = recv_config->id;
    recv_data->armors_num = recv_config->armors_num;
    recv_data->reserved   = recv_config->reserved;
    recv_data->tail       = recv_config->tail;

    return recv_data;
}

/**
 * @brief 用于注册一个视觉发送数据结构体,返回一个视觉发送数据结构体指针
 *
 * @param send_config
 * @return Vision_Send_s*
 */
Vision_Send_s *VisionSendRegister(Vision_Send_Init_Config_s *send_config)
{
    Vision_Send_s *send_data = (Vision_Send_s *)malloc(sizeof(Vision_Send_s));
    memset(send_data, 0, sizeof(Vision_Send_s));

    send_data->header        = send_config->header;
    send_data->detect_color  = send_config->detect_color;
    send_data->reset_tracker = send_config->reset_tracker;
    send_data->reserved      = send_config->reserved;
    send_data->tail          = send_config->tail;

    return send_data;
}

/**
 * @brief 用于注册一个视觉通信模块实例,返回一个视觉接收数据结构体指针
 *
 * @param init_config
 * @return Vision_Recv_s*
 */
Vision_Recv_s *VisionInit(Vision_Init_Config_s *init_config)
{
    vision_instance = (Vision_Instance *)malloc(sizeof(Vision_Instance));
    memset(vision_instance, 0, sizeof(Vision_Instance));

    init_config->usart_config.module_callback = DecodeVision;

    vision_instance->usart     = USARTRegister(&init_config->usart_config);
    vision_instance->recv_data = VisionRecvRegister(&init_config->recv_config);
    vision_instance->send_data = VisionSendRegister(&init_config->send_config);
    return vision_instance->recv_data;
}

/**
 * @brief 发送数据处理函数
 *
 * @param send 待发送数据
 * @param tx_buff 发送缓冲区
 *
 */
static void SendProcess(Vision_Send_s *send, uint8_t *tx_buff)
{
    /* 发送帧头，目标颜色，是否重置等数据 */
    tx_buff[0] = send->header;
    tx_buff[1] = send->detect_color;
    tx_buff[2] = send->reset_tracker;
    tx_buff[3] = send->reserved;

    /* 使用memcpy发送浮点型小数 */
    memcpy(&tx_buff[4], &send->roll, 4);
    memcpy(&tx_buff[8], &send->pitch, 4);
    memcpy(&tx_buff[12], &send->yaw, 4);
    memcpy(&tx_buff[16], &send->aim_x, 4);
    memcpy(&tx_buff[20], &send->aim_y, 4);
    memcpy(&tx_buff[24], &send->aim_z, 4);

    /* 发送帧尾 */
    tx_buff[25] = send->tail;
}

/**
 * @brief 发送函数
 *
 * @param send 待发送数据
 *
 */
void VisionSend()
{
    static uint8_t send_buff[VISION_SEND_SIZE];
    SendProcess(vision_instance->send_data, send_buff);
    USARTSend(vision_instance->usart, send_buff, VISION_SEND_SIZE, USART_TRANSFER_BLOCKING);
}