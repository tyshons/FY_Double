//
// Created by tyshon on 25-8-5.
//

#include "crc.h"
#include "main.h"
#include "spi.h"
#include "Reverse.h"
#include <string.h>

/**
 * @brief 获取X，Y轴角度位置
 *
 * 此函数通过SPI通信读取X轴的角度数据，并进行解析和校验。
 * 如果CRC校验失败，则返回上一次有效的角度值。
 *
 * @return 返回当前X轴的角度值（单位：度）
 */
float get_pos_x(void) {
    static float last_valid_angle_x = 0.0f;
    pos_x = 0;
    if (spi_xfer_done_x) {
        spi_xfer_done_x = 0;
        HAL_GPIO_WritePin(EN0_GPIO_Port, EN0_Pin, GPIO_PIN_SET);

        // 使用DMA方式发送和接收数据
        if (HAL_SPI_TransmitReceive_DMA(&hspi1,
                                        (uint8_t *)txBuffer_x,
                                        (uint8_t *)rxBuffer_x,
                                        DATA_SEQUENCE_SIZE) != HAL_OK)
        {
            Error_Handler();
        }
    }
    if (data_ready_x) {
        data_ready_x = 0;
        AngleResult res_x = Angle_Data_Processing(rxBuffer_x);
        unsigned int crc_x = MakeCrcPos(25, res_x.err, 0, 0, 0, res_x.angle);
        if (crc_x == res_x.crc5)
        {
            float angle_deg = (float)(res_x.angle) * 360.0f / 33554431.00f;
            pos_x = angle_deg;
            last_valid_angle_x = pos_x;
        }
        else
        {
            pos_x = last_valid_angle_x;
        }
    }

    return pos_x;
}

float get_pos_y(void) {
    static float last_valid_angle_y = 0.0f;
    pos_y = 0;
    if (spi_xfer_done_y) {
        spi_xfer_done_y = 0;
        HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, GPIO_PIN_SET);

        if (HAL_SPI_TransmitReceive_DMA(&hspi2,
                                        (uint8_t *)txBuffer_y,
                                        (uint8_t *)rxBuffer_y,
                                        DATA_SEQUENCE_SIZE) != HAL_OK)
        {
            Error_Handler();
        }
    }
    if (data_ready_y) {
        data_ready_y = 0;
        AngleResult res_y = Angle_Data_Processing(rxBuffer_y);
        unsigned int crc_y = MakeCrcPos(25, res_y.err, 0, 0, 0, res_y.angle);
        if (crc_y == res_y.crc5)
        {
            float angle_deg = (float)(res_y.angle) * 360.0f / 33554431.00f;
            pos_y = angle_deg;
            last_valid_angle_y = pos_y;
        }
        else
        {
            pos_y = last_valid_angle_y;
        }
    }

    return pos_y;
}

/**
 * @brief 处理角度数据缓冲区
 *
 * 此函数对接收到的原始数据进行位反转和解析，
 * 提取出角度值和错误标志，并计算CRC校验值。
 *
 * @param buffer 输入的原始数据缓冲区
 * @return 返回解析后的角度结果结构体
 */
AngleResult Angle_Data_Processing(volatile uint8_t *buffer)
{
    AngleResult result = {0};
    const uint8_t original_buffer5 = buffer[5];
    result.crc5 = (original_buffer5 & 0x3e) >> 1;
    buffer[2] = BitReverseTable256[buffer[2]];
    buffer[3] = BitReverseTable256[buffer[3]];
    buffer[4] = BitReverseTable256[buffer[4]];
    //buffer[5] = buffer[5]&0xe0;
    buffer[5] = BitReverseTable256[buffer[5]];
    result.angle = (buffer[5] & 0x3)<<23 | buffer[4]<<15 | buffer[3]<<7 | (buffer[2] & 0xfe)>>1;
    result.err = buffer[2] & 0x01;

    return result;
}