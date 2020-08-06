/*
 * This file is part of the Micro Python project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 J Devine, M Lambrichts
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

#ifndef MICROPY_INCLUDED_SHARED_BINDINGS_BUSIO_JACDAC_H
#define MICROPY_INCLUDED_SHARED_BINDINGS_BUSIO_JACDAC_H

#include "common-hal/microcontroller/Pin.h"
#include "common-hal/busio/JACDAC.h"
#include "py/ringbuf.h"

extern const mp_obj_type_t busio_uart_type;

// Construct an underlying UART object.
extern void common_hal_busio_jacdac_construct(busio_uart_obj_t *self,
    const mcu_pin_obj_t * tx, uint16_t receiver_buffer_size, byte* receiver_buffer,
    bool sigint_enabled);

extern void common_hal_busio_jacdac_deinit(busio_uart_obj_t *self);
extern bool common_hal_busio_jacdac_deinited(busio_uart_obj_t *self);

// Read characters. len is in characters NOT bytes!
extern int common_hal_busio_jacdac_receive(busio_uart_obj_t *self,
    uint8_t *data, size_t len);

// Write characters. len is in characters NOT bytes!
extern int common_hal_busio_jacdac_send(busio_uart_obj_t *self, const uint8_t *data);
extern bool common_hal_busio_jacdac_ready_to_tx(busio_uart_obj_t *self);

#endif  // MICROPY_INCLUDED_SHARED_BINDINGS_BUSIO_JACDAC_H
