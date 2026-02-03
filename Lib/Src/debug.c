#include "debug.h"
#include "usart.h"

void debug_upload_data(debug_data *data, uint8_t upload_type)
{
    uint8_t cur_data, i;
    uint8_t upload_data[37];                                            /* 数据上传数组 */
    upload_data[0] = DEBUG_DATA_HEAD;                                   /* 数据包第一个字节（数组第0个元素），固定为帧头 */
    cur_data = 2;                                                       /* 数据区域从第三个字节（数组第2个元素）开始 */

    switch (upload_type)                                                /* 判断数据类型 */
    {
        case TYPE_PID1:                                                                 /* PID参数组1 */
        case TYPE_PID2:
            upload_data[1] = upload_type;

            for (i = 0; i < 3; i++)                                            /* 循环存储P、I、D系数值，每个系数占4个字节 */
            {
                upload_data[cur_data++] = data->pid[upload_type - TYPE_PID1].pid.pidi8[i * 4 + 0];
                upload_data[cur_data++] = data->pid[upload_type - TYPE_PID1].pid.pidi8[i * 4 + 1];
                upload_data[cur_data++] = data->pid[upload_type - TYPE_PID1].pid.pidi8[i * 4 + 2];
                upload_data[cur_data++] = data->pid[upload_type - TYPE_PID1].pid.pidi8[i * 4 + 3];
            }
            break;

        case TYPE_USER_DATA:                                                   /* 波形数据 */
            upload_data[1] = upload_type;

            for (i = 0; i < 16; i++)                                           /* 循环存储1~16通道波形数据 */
            {
                upload_data[cur_data++] = (data->user_data[i] >> 8) & 0xFF;    /* 存储波形数据高8位 */
                upload_data[cur_data++] =  data->user_data[i] & 0xFF;          /* 存储波形数据低8位 */
            }
            break;

        default :
            upload_data[1] = 0xFE;                                             /* 数据类型错误，存储错误码0xFE */
            break;
    }

    if (upload_data[1] == 0xFE)                                                /* 数据类型错误，直接退出 */
    {
        return;
    }
    else                                                                       /* 数据类型正确 */
    {
        // uint16_t crc_res = crc16_calc(&(upload_data[0]), cur_data);            /* 进行CRC校验 */
        // upload_data[cur_data++] = (crc_res >> 8) & 0xFF;                       /* 存储校验结果高8位 */
        // upload_data[cur_data++] = (crc_res) & 0xFF;                            /* 存储校验结果低8位 */
        upload_data[cur_data++] = DEBUG_DATA_END;                              /* 存储帧尾 */
        HAL_UART_Transmit(&huart1, upload_data, cur_data, 0xFFFF);     /* 发送数据到上位机 */
    }
}
