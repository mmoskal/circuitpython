/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Scott Shawcroft for Adafruit Industries
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "py/runtime.h"
#include "supervisor/filesystem.h"
#include "supervisor/shared/stack.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if CIRCUITPY_DISPLAYIO
#include "shared-module/displayio/__init__.h"
#endif

#if CIRCUITPY_PULSEIO
#include "common-hal/pulseio/PulseIn.h"
#endif

#include "shared-bindings/microcontroller/__init__.h"


void port_background_task(void) {
    // Zero delay in case FreeRTOS wants to switch to something else.
    vTaskDelay(0);
    #if CIRCUITPY_PULSEIO
    pulsein_background();
    #endif

    if (codalLogStore.ptr) {
        char *dmesgCopy = malloc(sizeof(codalLogStore));

        uint32_t len;

        common_hal_mcu_disable_interrupts();
        len = codalLogStore.ptr;
        memcpy(dmesgCopy, codalLogStore.buffer, len);
        codalLogStore.ptr = 0;
        codalLogStore.buffer[0] = 0;
        common_hal_mcu_enable_interrupts();

        if (len) {
            if (dmesgCopy[len - 1] == '\n') {
                len--;
            }
            dmesgCopy[len] = 0;
            if (strchr(dmesgCopy, '\n')) {
                ESP_LOGW("JD", "DMESG:\n%s", dmesgCopy);
            } else {
                ESP_LOGW("JD", "DMESG: %s", dmesgCopy);
            }
        }
        free(dmesgCopy);
    }
}

void port_start_background_task(void) {
}

void port_finish_background_task(void) {
}
