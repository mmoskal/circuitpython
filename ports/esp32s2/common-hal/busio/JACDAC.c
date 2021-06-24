/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Microsoft Corp.
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

#include "shared-bindings/microcontroller/__init__.h"
#include "shared-bindings/busio/JACDAC.h"

#include "lib/utils/interrupt_char.h"
#include "py/mpconfig.h"
#include "py/gc.h"
#include "py/mperrno.h"
#include "py/runtime.h"
#include "supervisor/shared/translate.h"

#include "common-hal/busio/JACDAC.h"

static void jd_timer(busio_jacdac_obj_t *context) {
    busio_jacdac_base_callback_t f = context->timer_cb;
    if (f) {
        context->timer_cb = NULL;
        f(context);
    }
}

void common_hal_busio_jacdac_construct(busio_jacdac_obj_t *context, const mcu_pin_obj_t *pin) {
    busio_jacdac_init(&context->base);

    esp_timer_create_args_t args;
    args.callback = (esp_timer_cb_t)jd_timer;
    args.arg = context;
    args.dispatch_method = ESP_TIMER_TASK;
    args.name = "JD timeout";
    esp_timer_create(&args, &context->timer);

    // TODO
}

void common_hal_busio_jacdac_deinit(busio_jacdac_obj_t *context) {
    busio_jacdac_deinit(&context->base);
    esp_timer_stop(context->timer);
    esp_timer_delete(context->timer);
    context->timer = NULL;
    // TODO
}

bool common_hal_busio_jacdac_deinited(busio_jacdac_obj_t *context) {
    return context->base.rxFrame != NULL;
}

void common_hal_busio_jacdac_set_timer(busio_jacdac_obj_t *context, uint32_t us, busio_jacdac_base_callback_t callback) {
    common_hal_mcu_disable_interrupts();
    context->timer_cb = callback;
    esp_timer_start_once(context->timer, us);
    common_hal_mcu_enable_interrupts();
}

extern int common_hal_busio_jacdac_start_tx(busio_jacdac_obj_t *context, const void *data, uint32_t numbytes);
extern void common_hal_busio_jacdac_start_rx(busio_jacdac_obj_t *context, void *data, uint32_t maxbytes);
extern void common_hal_busio_jacdac_cancel(busio_jacdac_obj_t *context);
extern int common_hal_busio_jacdac_wait_high(busio_jacdac_obj_t *context);
