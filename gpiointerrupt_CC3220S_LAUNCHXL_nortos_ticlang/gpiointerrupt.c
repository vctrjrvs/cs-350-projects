/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
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
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/Timer.h>
#include <ti/drivers/UART2.h>
#include <ti/drivers/I2C.h>
#include "ti_drivers_config.h"

/* Global variables */
volatile bool buttonPressed = false;
volatile uint32_t secondsSinceReset = 0;
volatile uint8_t setPoint = 25; // Default set-point temperature
volatile uint8_t roomTemp = 25; // Simulated room temperature
volatile bool heaterOn = false;

/* UART Global Variables */
UART2_Handle uart;
char output[64];

/* I2C Global Variables */
I2C_Handle i2c;
I2C_Transaction i2cTransaction;
uint8_t txBuffer[1];
uint8_t rxBuffer[2];

/* Timers */
volatile unsigned char TimerFlag = 0;

/* Scheduler tasks */
typedef struct {
    void (*taskFunction)(void);
    uint32_t period;
    uint32_t elapsedTime;
} Task;

#define NUM_TASKS 3
Task tasks[NUM_TASKS];

void initUART(void)
{
    UART2_Params uartParams;
    UART2_Params_init(&uartParams);
    uartParams.baudRate = 115200;
    uart = UART2_open(CONFIG_UART2_0, &uartParams);
    if (uart == NULL) {
        while (1); // UART2_open() failed
    }
}


void initI2C(void)
{
    I2C_Params i2cParams;
    I2C_init();
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL) {
        while (1); // I2C_open() failed
    }
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 2;
}

int16_t readTemp(void)
{
    int16_t temperature = 0;
    txBuffer[0] = 0x00; // Command to read temperature
    if (I2C_transfer(i2c, &i2cTransaction)) {
        temperature = (rxBuffer[0] << 8) | rxBuffer[1];
        temperature *= 0.0078125; // Convert to Celsius
    } else {
        snprintf(output, sizeof(output), "Error reading temperature\n");
        UART2_write(uart, output, strlen(output), NULL);
    }
    return temperature;
}

void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    TimerFlag = 1;
}

void initTimer(void)
{
    Timer_Params params;
    Timer_init();
    Timer_Params_init(&params);
    params.period = 200000; // 200ms
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;
    Timer_Handle timer0 = Timer_open(CONFIG_TIMER_0, &params);
    if (timer0 == NULL || Timer_start(timer0) == Timer_STATUS_ERROR) {
        while (1); // Timer failed
    }
}

void gpioButtonFxn0(uint_least8_t index)
{
    buttonPressed = true;
}

void reportToUART(void)
{
    snprintf(output, sizeof(output), "<%02d,%02d,%d,%04d>\n", roomTemp, setPoint, heaterOn, secondsSinceReset);
    UART2_write(uart, output, strlen(output), NULL);
}

void updateHeaterState(void)
{
    if (roomTemp < setPoint) {
        heaterOn = true;
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON); // Turn on heater (LED)
    } else {
        heaterOn = false;
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF); // Turn off heater (LED)
    }
}

void checkButtons(void)
{
    static uint32_t buttonCounter = 0;
    buttonCounter++;
    if (buttonPressed && buttonCounter >= 1) { // 200ms
        if (GPIO_read(CONFIG_GPIO_BUTTON_0) == 0) { // Button pressed
            setPoint++;
        } else {
            setPoint--;
        }
        buttonPressed = false;
        buttonCounter = 0;
    }
}

void readTemperature(void)
{
    roomTemp = readTemp();
    updateHeaterState();
}

void uartTask(void)
{
    static uint32_t uartCounter = 0;
    uartCounter++;
    if (uartCounter >= 5) { // 1 second
        reportToUART();
        secondsSinceReset++;
        uartCounter = 0;
    }
}

void *mainThread(void *arg0)
{
    /* Initialize peripherals */
    GPIO_init();
    initUART();
    initI2C();
    initTimer();

    /* Configure GPIO */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    /* Initialize tasks */
    tasks[0].taskFunction = checkButtons;
    tasks[0].period = 1; // 200ms (1 period)
    tasks[0].elapsedTime = 0;

    tasks[1].taskFunction = readTemperature;
    tasks[1].period = 2; // 500ms (2 periods)
    tasks[1].elapsedTime = 0;

    tasks[2].taskFunction = uartTask;
    tasks[2].period = 5; // 1 second (5 periods)
    tasks[2].elapsedTime = 0;

    while (1) {
        if (TimerFlag) {
            TimerFlag = 0;
            for (int i = 0; i < NUM_TASKS; i++) {
                tasks[i].elapsedTime++;
                if (tasks[i].elapsedTime >= tasks[i].period) {
                    tasks[i].taskFunction();
                    tasks[i].elapsedTime = 0;
                }
            }
        }
    }
}
