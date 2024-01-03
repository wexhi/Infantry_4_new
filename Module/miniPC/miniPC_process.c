#include "miniPC_process.h"
#include "memory.h"

static USART_Instance *vision_usart_instance; // 用于和视觉通信的串口实例

static Vision_Recv_s recv_data;
static Vision_Send_s send_data;

/**
 * @brief 用于注册一个视觉接收数据结构体,返回一个视觉接收数据结构体指针
 *
 * @param recv_config
 * @return Vision_Recv_s*
 */
Vision_Recv_s *VisionRecvRegister(Vision_Recv_Init_Config_s *recv_config)
{
    recv_data.header     = recv_config->header;
    recv_data.tracking   = recv_config->tracking;
    recv_data.id         = recv_config->id;
    recv_data.armors_num = recv_config->armors_num;
    recv_data.reserved   = recv_config->reserved;
    recv_data.tail       = recv_config->tail;

    return &recv_data;
}

/**
 * @brief 用于注册一个视觉发送数据结构体,返回一个视觉发送数据结构体指针
 *
 * @param send_config
 * @return Vision_Send_s*
 */
Vision_Send_s *VisionSendRegister(Vision_Send_Init_Config_s *send_config)
{
    send_data.header        = send_config->header;
    send_data.detect_color  = send_config->detect_color;
    send_data.reset_tracker = send_config->reset_tracker;
    send_data.reserved      = send_config->reserved;
    send_data.tail          = send_config->tail;

    return &send_data;
}

/**
 * @brief 调用此函数初始化和视觉的串口通信
 *
 * @param handle 用于和视觉通信的串口handle(C板上一般为USART1,丝印为USART2,4pin)
 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback  = NULL;
    conf.recv_buff_size   = VISION_RECV_SIZE;
    conf.usart_handle     = _handle;
    vision_usart_instance = USARTRegister(&conf);

    return &recv_data;
}

static void SendProcess(Vision_Send_s *send, uint8_t *tx_buff)
{
    tx_buff[0] = send->header;
    tx_buff[1] = send->detect_color;
    tx_buff[2] = send->reset_tracker;
    tx_buff[3] = send->reserved;
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
    SendProcess(&send_data, send_buff);
    USARTSend(vision_usart_instance, send_buff, 4, USART_TRANSFER_BLOCKING);
}