//
// Created by tyshon on 25-8-5.
//

#include "crc.h"
unsigned int MakeCrcPos(
    unsigned int clocks,
    unsigned int error1,
    unsigned int error2,
    unsigned int endat22,
    unsigned long highpos,
    unsigned long lowpos)
{
    unsigned int ff[5];      /* 5-bit shift register (CRC state) */
    unsigned int code[66];   /* bit stream, max 66 bits */
    unsigned int ex;
    unsigned int crc = 0;
    int i;

    for (i = 0; i < 5; i++)
        ff[i] = 1;

    if (endat22) {
        code[0] = error1;
        code[1] = error2;
    } else {
        code[1] = error1;
    }

    for (i = 2; i < 34; i++) {
        code[i] = (lowpos & 1) ? 1 : 0;
        lowpos >>= 1;
    }

    for (i = 34; i < 66; i++) {
        code[i] = (highpos & 1) ? 1 : 0;
        highpos >>= 1;
    }

    for (i = (endat22 ? 0 : 1); i <= (int)(clocks + 1); i++) {
        ex = ff[4] ^ code[i];
        ff[4] = ff[3];
        ff[3] = ff[2] ^ ex;
        ff[2] = ff[1];
        ff[1] = ff[0] ^ ex;
        ff[0] = ex;
    }

    for (i = 4; i >= 0; i--) {
        ff[i] = ff[i] ? 0 : 1;
        crc <<= 1;
        crc |= ff[i];
    }
    return crc & 0x1F;
}