/*
 * Copyright (c) 2016-2020, Texas Instruments Incorporated
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
 *  ======== main_tirtos.c ========
 */
#include <stdint.h>

/* POSIX Header files */
#include <pthread.h>

/* RTOS header files */
#include <FreeRTOSConfig.h>
#include <FreeRTOS.h>
#include <task.h>

#include <ti/drivers/Board.h>
/* include semaphore library */
#include <ti/drivers/dpl/SemaphoreP.h>

extern void *spiThread(void *arg0);
extern void *inferenceThread(void *arg0);
extern void *mobileNetThread(void *arg0);

/* Stack size in bytes */
#define THREADSTACKSIZE    2048

SemaphoreP_Handle sem;

/*
 *  ======== main ========
 */
int main(void)
{
    pthread_t           thread_1, thread_2;
    pthread_attr_t      attrs_1, attrs_2;
    struct sched_param  priParam_1, priParam_2;
    int                 retc_1, retc_2;

    Board_init();

    /* Create binary semaphore */
    sem = SemaphoreP_createBinary(1);
    if (sem == NULL) {
        /* failed to create binary semaphore */
       while (1) {}
    }

    /* Initialize the attributes structure with default values */
    pthread_attr_init(&attrs_1);

    /* Set priority, detach state, and stack size attributes */
    priParam_1.sched_priority = 4;
    retc_1 = pthread_attr_setschedparam(&attrs_1, &priParam_1);
    retc_1 |= pthread_attr_setdetachstate(&attrs_1, PTHREAD_CREATE_DETACHED);
    retc_1 |= pthread_attr_setstacksize(&attrs_1, THREADSTACKSIZE*5);
    if (retc_1 != 0) {
        /* failed to set attributes */
        while (1) {}
    }

    retc_1 = pthread_create(&thread_1, &attrs_1, spiThread, NULL);
    if (retc_1 != 0) {
        /* pthread_create() failed */
        while (1) {}
    }

    /* Initialize the attributes structure with default values */
    pthread_attr_init(&attrs_2);

    /* Set priority, detach state, and stack size attributes */
    priParam_2.sched_priority = 3;
    retc_2 = pthread_attr_setschedparam(&attrs_2, &priParam_2);
    retc_2 |= pthread_attr_setdetachstate(&attrs_2, PTHREAD_CREATE_DETACHED);
    retc_2 |= pthread_attr_setstacksize(&attrs_2, THREADSTACKSIZE*5);
    if (retc_2 != 0) {
        /* failed to set attributes */
        while (1) {}
    }

    retc_2 = pthread_create(&thread_2, &attrs_2, inferenceThread, NULL);
    if (retc_2 != 0) {
        /* pthread_create() failed */
        while (1) {}
    }

    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler();

    return (0);
}
