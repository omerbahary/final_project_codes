/*
 * Copyright (c) 2018-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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
 *  ======== spimaster.c ========
 */
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

/* POSIX Header files */
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <ti/display/Display.h>
#include <ti/drivers/UART2.h>

#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC27XX.h>

//#include <ti/sysbios/knl/Task.h>


/* Driver configuration */
#include "ti_drivers_config.h"
#include "ArduCAM.h"

//Display_Handle display;
UART2_Handle  uartHandle;

extern uint8_t rxBuffer[LINELEN];
extern SemaphoreP_Handle sem;
extern void test(void);
extern uint32_t spiLen;
UART2_Params     params;
/*
 *  ======== spiThread ========
 */
void *spiThread(void *arg0)
{
    /* Call driver init functions. */
    GPIO_init();
    SPI_init();
    I2C_init();

    /* Configure the LED pin */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    UART2_Params_init(&params);
    params.baudRate      = 115200;
    params.readMode = UART2_Mode_BLOCKING;
    params.writeMode = UART2_Mode_BLOCKING;

    uartHandle = UART2_open(CONFIG_UART2_0, &params);
    if (!uartHandle)
    {
        while (1);
    }

    Power_setConstraint(PowerLPF3_DISALLOW_STANDBY);

    /* Configure the LED pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    /* Turn off user LED */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);

    Arducam_init();

    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);

    uint8_t notready;

     while (1) {
        GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
        Arducam_start_capture();
        while (1) {
            notready = Arducam_image_ready();
            if (notready)
            {
                GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
                break;
            }
        }
        Arducam_read_image();
    }
}
/*
 *  ======== inferenceThread ========
 */
void *inferenceThread(void *arg0)
{
    SemaphoreP_Status isAvailable = SemaphoreP_TIMEOUT;
    uint8_t localBuffer[LINELEN];

    while (1) {
        /* Skip till spiThread reads a row data */
        if(spiLen != 0) {
            /* Acquire semaphore */
            isAvailable = SemaphoreP_pend(sem, SemaphoreP_TIMEOUT);
            if (isAvailable != SemaphoreP_OK) {
                break;
            }

            /* Copy rxbuffer data to local buffer */
            memcpy(localBuffer, rxBuffer, sizeof(rxBuffer));

            /* Release semaphore after copying rxBuffer data to localBuffer */
            SemaphoreP_post(sem);

            tinie_image_processing(localBuffer, spiLen);
        }
        sleep(1);
    }
}
