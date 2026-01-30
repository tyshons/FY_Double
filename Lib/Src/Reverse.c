//
// Created by tyshon on 25-8-5.
//

#include <stdio.h>
#include "crc.h"
#include "main.h"
#include "spi.h"
#include "Reverse.h"

#include <string.h>


float get_pos_x(void) {
    static float last_valid_angle = 0.0f;
    pos_x = 0;
    if (spi_xfer_done_x) {// 启动 X 轴读取
        spi_xfer_done_x = 0;
        HAL_GPIO_WritePin(EN0_GPIO_Port, EN0_Pin, GPIO_PIN_SET); // 拉高使能

        if (HAL_SPI_TransmitReceive_DMA(&hspi1,
                                        (uint8_t *)txBuffer_x,
                                        (uint8_t *)rxBuffer_x,
                                        DATA_SEQUENCE_SIZE) != HAL_OK)
        {
            Error_Handler();
        }
    }
    if (data_ready_x) {// 解析 X 轴数据
        //for (volatile int i = 0; i < 500; i++);
        //printf("RX: %02X %02X %02X %02X\n", rxBuffer_x[0], rxBuffer_x[1], rxBuffer_x[2], rxBuffer_x[3]);
        data_ready_x = 0;
        AngleResult res_x = Angle_Data_Processing(rxBuffer_x);
        unsigned int crc_x = MakeCrcPos(25, res_x.err, 0, 0, 0, res_x.angle);
        if (crc_x == res_x.crc5)
        {
            float angle_deg = (float)(res_x.angle) * 360.0f / 33554431.00f;
            pos_x = angle_deg;
            last_valid_angle = pos_x;
        }
        else
        {
            pos_x = last_valid_angle;
        }
    }

    return pos_x;
}

float get_pos_y(void){

    float pos_y = 0;
    if (spi_xfer_done_y) {// 启动 X 轴读取
        spi_xfer_done_y = 0;
        HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, GPIO_PIN_SET); // 拉高使能
        if (HAL_SPI_TransmitReceive_DMA(&hspi2,
                                        (uint8_t *)txBuffer_y,
                                        (uint8_t *)rxBuffer_y,
                                        DATA_SEQUENCE_SIZE) != HAL_OK)
        {
            Error_Handler();
        }
    }
    if (data_ready_y) {// 解析 X 轴数据
        data_ready_y = 0;
        AngleResult res_y = Angle_Data_Processing(rxBuffer_y);
        unsigned int crc_y = MakeCrcPos(25, res_y.err, 0, 0, 0, res_y.angle);
        if (crc_y == res_y.crc5)
        {
            float angle_deg = (float)(res_y.angle) * 360.0f / 33554431.00f;
            pos_y = angle_deg;
        }
        else
        {
            biaozhi = 1; // CRC 错误，标记无效
        }
    }

    return pos_y;
}


AngleResult Angle_Data_Processing(uint8_t *buffer)
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