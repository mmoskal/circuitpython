/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Microsoft
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

#ifndef MICROPY_INCLUDED_ESP32S2_COMMON_HAL_BUSIO_JACDAC_H
#define MICROPY_INCLUDED_ESP32S2_COMMON_HAL_BUSIO_JACDAC_H

#include "common-hal/microcontroller/Pin.h"
#include "py/obj.h"
#include "shared-bindings/busio/JACDAC.h"

#include "components/esp_timer/include/esp_timer.h"
#include "hal/uart_ll.h"
#include "driver/uart.h"

typedef struct busio_jacdac_hw_alloc {
    esp_timer_handle_t timer;
    intr_handle_t intr_handle;
    uint8_t uart_num;
    uint8_t pin_num;
} busio_jacdac_hw_alloc_t;

struct busio_jacdac_obj {
    busio_jacdac_base_obj_t base;
    const mcu_pin_obj_t *pinobj;
    uint8_t pin_num;
    uint8_t uart_num;
    bool seen_low;
    bool rx_ended;
    uint16_t tx_len;
    uint16_t rx_len;
    uint16_t data_left;
    uint8_t *fifo_buf;
    busio_jacdac_base_callback_t timer_cb;
    uart_dev_t *uart_hw;
    busio_jacdac_hw_alloc_t *hw_alloc;
};

extern void jacdac_reset(void);
extern bool common_hal_busio_jacdac_is_free(uart_port_t port);


#define JD_WR_OVERHEAD 20

#endif // MICROPY_INCLUDED_ESP32S2_COMMON_HAL_BUSIO_JACDAC_H
