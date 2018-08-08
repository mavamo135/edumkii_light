/*
 * Copyright (c) 2015-2017, Texas Instruments Incorporated
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
 *  ======== gpiomain.c ========
 */

/* For usleep() */
#include <unistd.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

/* POSIX Header files */
#include <pthread.h>

/* RTOS header files */
#include <ti/sysbios/BIOS.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/display/Display.h>

/* Board Header file */
#include "Board.h"

/* Boosterpack EDUMKII BSP LCD */
#include "BSP/hal_lcd.h"
#include "BSP/lcd.h"
#include <ti/grlib/grlib.h>

/***** GLOBAL VARIABLES *****/
extern uint32_t LUX;
extern pthread_mutex_t hold;

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Local copy of LUX */
    uint32_t    localLUX;
    float       floatLUX;

    /* Graphic library context */
    Graphics_Context  g_sContext;

    /* configure & open Display driver */
    Display_Handle    myDisplay;
    Display_Params    params;
    Display_Params_init(&params);
    myDisplay = Display_open(Display_Type_UART, &params);

    /* Initializes display*/
    Crystalfontz128x128_Init();

    /* Set default screen orientation*/
    Crystalfontz128x128_SetOrientation(0);

    /* Initializes graphics context*/
    Graphics_initContext(&g_sContext, &g_sCrystalfontz128x128, &g_sCrystalfontz128x128_funcs);
    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
    Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
    GrContextFontSet(&g_sContext, &g_sFontFixed6x8);
    Graphics_clearDisplay(&g_sContext);
    Graphics_drawStringCentered(&g_sContext,
                                (int8_t *)"Light Sensor:",
                                AUTO_STRING_LENGTH,
                                64,
                                30,
                                OPAQUE_TEXT);

    while (1) {

        pthread_mutex_lock(&hold);

        localLUX = LUX;

        pthread_mutex_unlock(&hold);

        floatLUX = (float)localLUX;
        Display_printf(myDisplay, 0, 0, "Light Sensor: %f lux", floatLUX);

        /* Display light */
        char string[10];
        sprintf(string, "%f", floatLUX);
        Graphics_drawStringCentered(&g_sContext,
                                    (int8_t *)string,
                                    6,
                                    48,
                                    70,
                                    OPAQUE_TEXT);
        sprintf(string, "lux");
        Graphics_drawStringCentered(&g_sContext,
                                    (int8_t *)string,
                                    3,
                                    86,
                                    70,
                                    OPAQUE_TEXT);
        usleep(10000); // Only update Serial terminal every 0.01s
  }
}
