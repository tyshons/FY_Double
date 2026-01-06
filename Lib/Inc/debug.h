#ifndef DEBUG_H
#define DEBUG_H
#include <stdint.h>

#define DEBUG_DATA_HEAD   0xA5       /* Ö¡Í· */
#define DEBUG_DATA_END    0x5A       /* Ö¡Î² */

typedef enum
{
    TYPE_PID1           = 0x20,      /* PID1 */
    TYPE_PID2           = 0x21,      /* PID2 */
    TYPE_USER_DATA      = 0x30,      /* 波形 */
}upload_type;

typedef struct
{
    union
    {
        float pidf[3];               /* PID发送 */
        int8_t pidi8[12];            /* PID接收 */
    }pid;
}pid_struct;

typedef struct
{
    pid_struct pid[10];              /* pid */
    int16_t user_data[16];           /* 波形 */
}debug_data;

extern debug_data g_debug;


/* 上位机—>板子 */
typedef enum
{
    CMD_SET_SPEED       = 0x23,      /* 设置速度 */
    CMD_SET_LOCATION    = 0x24,      /* 设置位置 */
    CMD_SET_PID1        = 0x31,      /* PID1 */
    CMD_SET_PID2        = 0x32,      /* PID2 */

}cmd_type;


typedef enum
{
    HALT_CODE           = 0x01,      /* 停机 */
    RUN_CODE            = 0x02,      /* 运行 */
    BREAKED             = 0x03,      /* 刹车 */
}cmd_code;


typedef struct
{
    uint8_t Ctrl_code;
    float *speed;
    float *location;
    float pid[3];
}debug_data_rev;

extern debug_data_rev debug_rev;

#define DEBUG_REV_MAX_LEN   17


void debug_obj_init(debug_data *data);                                              /* 初始化 */
void debug_handle(uint8_t *data);                                                   /* 接收 */
void debug_upload_data(debug_data * data, uint8_t upload_type);                     /* 上传 */

void debug_init(void);                                                              /* ³õÊ¼»¯µ÷ÊÔ */

void debug_send_initdata(upload_type PIDx,float *SetPoint,float P,float I,float D); /* PID³õÊ¼»¯Êý¾ÝÉÏ´« */
void debug_send_speed(float speed);                                                 /* ËÙ¶ÈÊý¾ÝÉÏ´« */
void debug_send_wave_data(uint8_t chx,int16_t wave);                                /* ²¨ÐÎÊý¾ÝÉÏ´« */

void debug_receive_pid(upload_type PIDx,float *P,float *I,float *D);                /* PID接收 */
uint8_t debug_receive_ctrl_code(void);                                              /* 控制接收 */

#endif


