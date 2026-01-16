//
// Created by tyshon on 25-8-5.
//

#include <stdio.h>
#include "crc.h"
#include "main.h"
#include "spi.h"
#include "Reverse.h"

AngleResult res_x;
AngleResult res_y;

void Angle_Update_Task(void)
{
    if (spi_xfer_done_x) {// 启动 X 轴读取
        HAL_GPIO_WritePin(EN0_GPIO_Port, EN0_Pin, GPIO_PIN_SET); // 拉高使能
        if (HAL_SPI_TransmitReceive_DMA(&hspi1, txBuffer_x, rxBuffer_x, DATA_SEQUENCE_SIZE) == HAL_OK) {
            spi_xfer_done_x = 0;
            data_ready_x = 0;

        } else {
        }
    }
    if (data_ready_x) {// 解析 X 轴数据
        res_x = Angle_Data_Processing(rxBuffer_x);
        if (res_x.crc5 == MakeCrcPos(25, res_x.err, 0, 0, 0, res_x.angle)) {
            float angle_deg = (float)(res_x.angle) * 360.0f / 33554431.00f;
            latest_angle_sp = angle_deg;
            angle_valid_sp = 1;
        } else {
            angle_valid_sp = 0; // CRC 错误，标记无效
        }
    }

    if (spi_xfer_done_y) {
        HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, GPIO_PIN_SET);
        if (HAL_SPI_TransmitReceive_DMA(&hspi2, txBuffer_y, rxBuffer_y, DATA_SEQUENCE_SIZE) == HAL_OK) {
            spi_xfer_done_y = 0;
            data_ready_y = 0;
        } else {
        }
    } else {

            }

    if (data_ready_y) {
        res_y = Angle_Data_Processing(rxBuffer_y);
        if (res_y.crc5 == MakeCrcPos(25, res_y.err, 0, 0, 0, res_y.angle)) {
            float angle_deg = (float)(res_y.angle) * 360.0f / 33554431.00f;
            latest_angle_el = angle_deg;
            angle_valid_el = 1;
        } else {
            angle_valid_el = 0;
        }

    }
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