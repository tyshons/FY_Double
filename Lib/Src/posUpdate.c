//
// Created by tyshon on 25-8-5.
//

#include "crc.h"
#include "main.h"
#include "spi.h"
#include "posUpdate.h"

#include <math.h>
#include <string.h>

//编码器传输变量定义
uint8_t txBuffer_x[DATA_SEQUENCE_SIZE] = {0x07, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t txBuffer_y[DATA_SEQUENCE_SIZE] = {0x07, 0x00, 0x00, 0x00, 0x00, 0x00};
volatile uint8_t rxBuffer_x[DATA_SEQUENCE_SIZE] = {0};
volatile uint8_t rxBuffer_y[DATA_SEQUENCE_SIZE] = {0};
volatile uint8_t spi_xfer_done_x = 1;
volatile uint8_t spi_xfer_done_y = 1;
volatile uint8_t sck_edge_count_x = 0;
volatile uint8_t sck_edge_count_y = 0;
volatile uint8_t data_ready_x = 0;
volatile uint8_t data_ready_y = 0;

//速度估算处理变量定义
static float last_valid_angle_x = 0.0f;
static float last_valid_speed_x = 0.0f;   // rmp
static uint32_t last_valid_time_x = 0;    // ms
static float last_valid_angle_y = 0.0f;
static float last_valid_speed_y = 0.0f;   // rmp
static uint32_t last_valid_time_y = 0;    // ms

/**
 * @brief 处理角度差
 *
 * 将角度差限制在180度内，并返回最小角度差。
 *
 * @param a 当前位置
 * @param b 前一刻位置
 * @return 返回两个位置间的最小角度差
 */
static float get_min_angle_diff(const float a, const float b){
    float diff = fmodf(a - b + 180.0f, 360.0f) - 180.0f;
    if (diff > 180.0f) diff -= 360.0f;
    else if (diff < -180.0f) diff += 360.0f;
    return diff;
}

/**
 * @brief 获取X，Y轴角度位置以及速度
 *
 * 此函数通过SPI通信读取X轴的角度数据，并进行解析和校验并推算速度。
 * 如果CRC校验失败，则使用前一刻的速度进行外推。
 *
 * @return 返回当前X轴的角度值（单位：度）
 */
float get_pos_x(float* out_speed) {
    static uint32_t last_call_time = 0;
    const uint32_t current_time = HAL_GetTick();

    if (last_call_time == 0) {
        last_call_time = current_time;
    }

    float current_pos = last_valid_angle_x;
    float current_speed = last_valid_speed_x;

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
    if (data_ready_x){
        data_ready_x = 0;
        const AngleResult res_x = Angle_Data_Processing(rxBuffer_x);
        const unsigned int crc_x = MakeCrcPos(25, res_x.err, 0,
                                             0, 0, res_x.angle);
        if (crc_x == res_x.crc5){
            const float angle_deg = (float)(res_x.angle) * 360.0f / 33554431.00f;
            if (last_valid_time_x > 0) {
                const uint32_t dt_ms = current_time - last_valid_time_x;
                if (dt_ms > 0 && dt_ms < 1000) {
                    const float dt_s = (float)dt_ms / 1000.0f;
                    const float dt_a = get_min_angle_diff(angle_deg, last_valid_angle_x);
                    current_speed = dt_a / (dt_s * 6);
                }
            }
            last_valid_angle_x = angle_deg;
            last_valid_speed_x = current_speed;
            last_valid_time_x = current_time;
            current_pos = angle_deg;
        }
    }
    if (last_valid_time_x == 0) {
        current_pos = 0.0f;
        current_speed = 0.0f;
    } else {
        const uint32_t time_since_good = current_time - last_valid_time_x;
        if (time_since_good >= 10) {

        } else {
            const float dt_s = (float)time_since_good / 1000.0f;
            current_pos = last_valid_angle_x + last_valid_speed_x * dt_s;
            current_speed = last_valid_speed_x;
        }
    }

    if (out_speed != NULL) {
        *out_speed = current_speed;
    }

    last_call_time = current_time;
    return current_pos;
}

float get_pos_y(float* out_speed) {
    static uint32_t last_call_time = 0;
    const uint32_t current_time = HAL_GetTick();

    if (last_call_time == 0) {
        last_call_time = current_time;
    }

    float current_pos = last_valid_angle_y;
    float current_speed = last_valid_speed_y;

    if (spi_xfer_done_y) {
        spi_xfer_done_y = 0;
        HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, GPIO_PIN_SET);
        // 使用DMA方式发送和接收数据
        if (HAL_SPI_TransmitReceive_DMA(&hspi2,
                                        (uint8_t *)txBuffer_y,
                                        (uint8_t *)rxBuffer_y,
                                        DATA_SEQUENCE_SIZE) != HAL_OK)
        {
            Error_Handler();
        }
    }
    if (data_ready_y){
        data_ready_y = 0;
        const AngleResult res_y = Angle_Data_Processing(rxBuffer_y);
        const unsigned int crc_y = MakeCrcPos(25, res_y.err, 0,
                                             0, 0, res_y.angle);
        if (crc_y == res_y.crc5){
            const float angle_deg = (float)(res_y.angle) * 360.0f / 33554431.00f;
            if (last_valid_time_y > 0) {
                const uint32_t dt_ms = current_time - last_valid_time_y;
                if (dt_ms > 0 && dt_ms < 1000) {
                    const float dt_s = (float)dt_ms / 1000.0f;
                    const float dt_a = get_min_angle_diff(angle_deg, last_valid_angle_y);
                    current_speed = dt_a / (dt_s * 6);
                }
            }
            last_valid_angle_y = angle_deg;
            last_valid_speed_y = current_speed;
            last_valid_time_y = current_time;
            current_pos = angle_deg;
        }
    }
    if (last_valid_time_y == 0) {
        current_pos = 0.0f;
        current_speed = 0.0f;
    } else {
        const uint32_t time_since_good = current_time - last_valid_time_y;
        if (time_since_good >= 10) {

        } else {
            const float dt_s = (float)time_since_good / 1000.0f;
            current_pos = last_valid_angle_y + last_valid_speed_y * dt_s;
            current_speed = last_valid_speed_y;
        }
    }

    if (out_speed != NULL) {
        *out_speed = current_speed;
    }

    last_call_time = current_time;
    return current_pos;
}

/**
  * @brief SPI传输完成回调函数
  * @param hspi SPI句柄
  */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        spi_xfer_done_x = 1;
        data_ready_x = 1;
    }
    if (hspi->Instance == SPI2) {
        spi_xfer_done_y = 1;
        data_ready_y = 1;
    }
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