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
#include "py/ringbuf.h"
#include "supervisor/background_callback.h"

// 255 minus size of the serial header, rounded down to 4
#define JD_SERIAL_PAYLOAD_SIZE 236
#define JD_MAX_FRAME_SIZE 252

// structure for a jacdac frame
struct _jd_frame_t {
    uint16_t crc;
    uint8_t size;
    uint8_t flags;

    uint64_t device_identifier;

    uint8_t service_size;
    uint8_t service_number;
    uint16_t service_command;

    uint8_t _data[JD_SERIAL_PAYLOAD_SIZE];
} __attribute__((__packed__, aligned(4)));
typedef struct _jd_frame_t jd_frame_t;
#define JD_FRAME_SIZE(pkt) ((pkt)->size + 12)

typedef struct jd_linked_frame {
    struct jd_linked_frame *next;
    uint32_t timestamp_ms;
    jd_frame_t frame;
} jd_linked_frame_t;
#define JD_LINKED_FRAME_HEADER_SIZE (sizeof(uint32_t) + sizeof(void *))

typedef struct busio_jacdac_obj busio_jacdac_obj_t; // defined in common-hal

typedef struct {
    mp_obj_base_t base;
    background_callback_t callback;
    jd_frame_t *rxBuffer;
    ringbuf_t rxTmpQueue;
    jd_linked_frame_t *txQueue;
    jd_linked_frame_t *rxQueue;
    jd_linked_frame_t *frameToSplit;
} busio_jacdac_base_obj_t;

extern const mp_obj_type_t busio_jacdac_type;

typedef void (*busio_jacdac_base_callback_t)(busio_jacdac_base_obj_t *);

// to be called from common-hal only:
extern void busio_jacdac_init(busio_jacdac_base_obj_t *self);
extern void busio_jacdac_deinit(busio_jacdac_base_obj_t *self);

// standard MP stuff
extern void common_hal_busio_jacdac_construct(busio_jacdac_obj_t *context, const mcu_pin_obj_t *pin);
extern void common_hal_busio_jacdac_deinit(busio_jacdac_obj_t *context);
extern bool common_hal_busio_jacdac_deinited(busio_jacdac_obj_t *context);

// these follow jacdac-c
extern int common_hal_busio_jacdac_set_timer(busio_jacdac_obj_t *context, uint32_t us, busio_jacdac_base_callback_t callback);
extern int common_hal_busio_jacdac_start_tx(busio_jacdac_obj_t *context, const void *data, uint32_t numbytes);
extern void common_hal_busio_jacdac_start_rx(busio_jacdac_obj_t *context, void *data, uint32_t maxbytes);
extern void common_hal_busio_jacdac_cancel(busio_jacdac_obj_t *context);
extern int common_hal_busio_jacdac_wait_high(busio_jacdac_obj_t *context);

#endif  // MICROPY_INCLUDED_SHARED_BINDINGS_BUSIO_JACDAC_H
