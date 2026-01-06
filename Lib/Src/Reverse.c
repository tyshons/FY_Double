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
uint8_t txBuffer_x[DATA_SEQUENCE_SIZE] = {0x07, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t txBuffer_y[DATA_SEQUENCE_SIZE] = {0x07, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t rxBuffer_x[DATA_SEQUENCE_SIZE] = {0};
uint8_t rxBuffer_y[DATA_SEQUENCE_SIZE] = {0};

float get_pos(void) {
    float pos = 0;
    if (spi_xfer_done_x)
    {
        spi_xfer_done_x = 0;
        sck_edge_count_x = 0;
        HAL_GPIO_WritePin(EN0_GPIO_Port, EN0_Pin, GPIO_PIN_SET);


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
        unsigned int crc = 0;
        res_x = Angle_Data_Processing(rxBuffer_x);
        crc = MakeCrcPos(25, res_x.err, 0, 0, 0, res_x.angle);
        // printf("Calculated CRC: 0x%02X, Received CRC: 0x%02X\r\n", crc, res.crc5);
        if (crc == res_x.crc5)
        {
            const float angle_deg = (float)res_x.angle * 360.00f / 33554431.00f;
            pos = angle_deg;
            printf("Angle: %f\n", angle_deg);
        }
        else
        {
            uint8_t crc_error = 1;  //错误处理
        }
    }

    return pos;
}

float get_pos_pitch(void) {
    float pos_y = 0;
    if (spi_xfer_done_y)
    {
        spi_xfer_done_y = 0;
        sck_edge_count_y = 0;
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
        unsigned int crc = 0;

        res_y = Angle_Data_Processing(rxBuffer_y);
        crc = MakeCrcPos(25, res_y.err, 0, 0, 0, res_y.angle);

        if (crc == res_y.crc5)
        {
            const float angle_deg = (float)res_y.angle * 360.00f / 33554431.00f;
            pos_y = angle_deg;
            printf("Angle: %f\n", angle_deg);
        }
        else
        {
            uint8_t crc_error = 1;  //错误处理

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