/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== temperature.c ========
 */

/* For usleep() */
#include <unistd.h>
#include <stdint.h>
#include <stddef.h>

/* POSIX Header files */
#include <pthread.h>

/* RTOS header files */
#include <ti/sysbios/BIOS.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>

/* Board Header file */
#include "Board.h"

/***** GLOBAL VARIABLES *****/
extern uint32_t LUX;
extern pthread_mutex_t hold;

uint32_t calcLux(uint16_t rawLux)
{
    uint16_t    exponent = 0;
    uint32_t    result = 0;

    /*Convert to LUX*/
    //extract result & exponent data from raw readings
    result = rawLux & 0x0FFF;
    exponent = (rawLux>>12) & 0x000F;

    //convert raw readings to LUX
    switch(exponent){
        case 0: //*0.015625
            result = result>>6;
            break;
        case 1: //*0.03125
            result = result>>5;
            break;
        case 2: //*0.0625
            result = result>>4;
            break;
        case 3: //*0.125
            result = result>>3;
            break;
        case 4: //*0.25
            result = result>>2;
            break;
        case 5: //*0.5
            result = result>>1;
            break;
        case 6:
            result = result;
            break;
        case 7: //*2
            result = result<<1;
            break;
        case 8: //*4
            result = result<<2;
            break;
        case 9: //*8
            result = result<<3;
            break;
        case 10: //*16
            result = result<<4;
            break;
        case 11: //*32
            result = result<<5;
            break;
    }
    return result;
}

/*
 *  ======== lighthread ========
 */
void *lightThread(void *arg0)
{
    uint8_t         txBuffer[3];
    uint8_t         rxBuffer[2];
    I2C_Handle      i2c;
    I2C_Params      i2cParams;
    I2C_Transaction i2cTransaction;

    /* Local copy of LUX */
    uint32_t   localLUX;

    /* 0.005 second delay */
    uint32_t time = 5000;

    /* Call driver init functions */
    GPIO_init();
    I2C_init();

    /* Create I2C for usage */
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2c = I2C_open(Board_I2C, &i2cParams);
    if (i2c == NULL) {
        while (1);
    }

    /* Point to the configuration register and write default configuration */
    txBuffer[0] = OPT3001_CONFIG_REG;
    txBuffer[1] = (OPT3001_DEFAULT_CONFIG_100 & 0xFF00) >> 8;
    txBuffer[2] = (OPT3001_DEFAULT_CONFIG_100 & 0x00FF);
    i2cTransaction.slaveAddress = Board_OPT3001_ADDR;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 3;
    i2cTransaction.readBuf = NULL;
    i2cTransaction.readCount = 0;
    if (!I2C_transfer(i2c, &i2cTransaction))
    {
        while(1);
    }

    /* Point to the result register and read its 2 bytes */
    txBuffer[0] = OPT3001_RESULT_REG;
    i2cTransaction.slaveAddress = Board_OPT3001_ADDR;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 2;

    /* Turn on user LED */
    GPIO_write(Board_GPIO_LED_LP, Board_GPIO_LED_ON);

    while (1) {
        GPIO_toggle(Board_GPIO_LED_LP);

        if (I2C_transfer(i2c, &i2cTransaction)) {

            /* Extract lux from the received data; see OPT3001 datasheet */
            localLUX = calcLux( (rxBuffer[0]<<8) + (rxBuffer[1]) );

            // MUTEX usage to avoid conflicts with globals
            pthread_mutex_lock(&hold);

            LUX =  localLUX;

            pthread_mutex_unlock(&hold);
        }

        // usleep (useconds)
        usleep(time);
    }
}
