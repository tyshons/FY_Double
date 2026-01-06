#include "debug.h"
#include "usart.h"

void debug_upload_data(debug_data *data, uint8_t upload_type)
{
    uint8_t cur_data, i;
    uint8_t upload_data[37];                                            /* Êý¾ÝÉÏ´«Êý×é */
    upload_data[0] = DEBUG_DATA_HEAD;                                   /* Êý¾Ý°üµÚ1¸ö×Ö½Ú£¨Êý×éµÚ0¸öÔªËØ£©£¬¹Ì¶¨ÎªÖ¡Í· */
    cur_data = 2;                                                       /* Êý¾ÝÓò´ÓµÚ3¸ö×Ö½Ú£¨Êý×éµÚ2¸öÔªËØ£©¿ªÊ¼ */

    switch (upload_type)                                                /* ÅÐ¶ÏÊý¾ÝÀà±ð */
    {
        case TYPE_PID1:                                                                 /* PID²ÎÊý×é±ð */
        case TYPE_PID2:
            upload_data[1] = upload_type;

            for (i = 0; i < 3; i++)                                            /* Ñ­»·´æÈëP¡¢I¡¢DÏµÊýÖµ£¬Ã¿¸öÏµÊýÕ¼4¸ö×Ö½Ú */
            {
                upload_data[cur_data++] = data->pid[upload_type - TYPE_PID1].pid.pidi8[i * 4 + 0];
                upload_data[cur_data++] = data->pid[upload_type - TYPE_PID1].pid.pidi8[i * 4 + 1];
                upload_data[cur_data++] = data->pid[upload_type - TYPE_PID1].pid.pidi8[i * 4 + 2];
                upload_data[cur_data++] = data->pid[upload_type - TYPE_PID1].pid.pidi8[i * 4 + 3];
            }
            break;

        case TYPE_USER_DATA:                                                   /* ²¨ÐÎÊý¾Ý */
            upload_data[1] = upload_type;

            for (i = 0; i < 16; i++)                                           /* Ñ­»·´æÈë1~16¸öÍ¨µÀ²¨ÐÎÊý¾Ý */
            {
                upload_data[cur_data++] = (data->user_data[i] >> 8) & 0xFF;    /* ´æÈë²¨ÐÎÊý¾Ý¸ß8Î» */
                upload_data[cur_data++] =  data->user_data[i] & 0xFF;          /* ´æÈë²¨ÐÎÊý¾ÝµÍ8Î» */
            }
            break;

        default :
            upload_data[1] = 0xFE;                                             /* Êý¾ÝÀà±ð´íÎó£¬´æÈë´íÎóÂë0xFE */
            break;
    }

    if (upload_data[1] == 0xFE)                                                /* Êý¾ÝÀà±ð´íÎó£¬Ö±½ÓÌø³ö */
    {
        return;
    }
    else                                                                       /* Êý¾ÝÀà±ðÕýÈ· */
    {
        // uint16_t crc_res = crc16_calc(&(upload_data[0]), cur_data);            /* ½øÐÐCRCÐ£Ñé */
        // upload_data[cur_data++] = (crc_res >> 8) & 0xFF;                       /* ´æÈëÐ£Ñé½á¹û¸ß8Î» */
        // upload_data[cur_data++] = (crc_res) & 0xFF;                            /* ´æÈëÐ£Ñé½á¹ûµÍ8Î» */
        upload_data[cur_data++] = DEBUG_DATA_END;                              /* ´æÈëÖ¡Î² */
        HAL_UART_Transmit(&huart1, upload_data, cur_data, 0xFFFF);     /* ·¢ËÍÊý¾Ýµ½ÉÏÎ»»ú */
    }
}