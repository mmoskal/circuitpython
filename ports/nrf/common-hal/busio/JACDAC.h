/*
 * This file is part of the MicroPython project, http://micropython.org/
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

#ifndef MICROPY_INCLUDED_NRF_COMMON_HAL_BUSIO_JACDAC_H
#define MICROPY_INCLUDED_NRF_COMMON_HAL_BUSIO_JACDAC_H

#include "common-hal/microcontroller/Pin.h"


#include "py/obj.h"
#include "py/ringbuf.h"

#include "peripherals/nrf/timers.h"






// 255 minus size of the serial header, rounded down to 4
#define JD_SERIAL_PAYLOAD_SIZE 236








/*****************************************************************/
// implemented by SWS.h






/*****************************************************************/
// implemented by JACDAC.h

#define JD_RX_ARRAY_SIZE 10
#define JD_TX_ARRAY_SIZE 10

struct _jd_frame_t {
    uint16_t crc;
    uint8_t size;
    uint8_t flags;

    uint64_t device_identifier;

    uint8_t data[JD_SERIAL_PAYLOAD_SIZE + 4];
} __attribute__((__packed__, aligned(4)));
typedef struct _jd_frame_t jd_frame_t;


typedef struct {
    uint8_t txHead;
    uint8_t txTail;
    uint8_t rxHead;
    uint8_t rxTail;

    jd_frame_t* rxBuf; // holds the pointer to the current rx buffer
    jd_frame_t* txBuf; // holds the pointer to the current tx buffer
    jd_frame_t* rxArray[JD_RX_ARRAY_SIZE];
    jd_frame_t* txArray[JD_TX_ARRAY_SIZE];
} busio_jacdac_buffer_obj_t;

typedef struct {
    mp_obj_base_t base;
    nrfx_uarte_t* uart;
    nrfx_timer_t* timer;
    uint8_t pin;
    uint16_t sws_status;
    uint16_t status;

    busio_jacdac_buffer_obj_t* buffer;

} busio_jacdac_obj_t;

typedef struct {
    uint32_t bus_state;
    uint32_t bus_lo_error;
    uint32_t bus_uart_error;
    uint32_t bus_timeout_error;
    uint32_t packets_sent;
    uint32_t packets_received;
    uint32_t packets_dropped;
} jd_diagnostics_t;
jd_diagnostics_t *jd_get_diagnostics(void);


typedef void (*cb_t)(void);

/*

// Provided by JACDAC_util.c
uint32_t jd_random_around(uint32_t v);
uint32_t jd_random(void);
void jd_seed_random(uint32_t s);
uint32_t jd_hash_fnv1a(const void *data, unsigned len);
uint16_t jd_crc16(const void *data, uint32_t size);

// Provided by JACDAC_timer.c
void tim_init(void);
void tim_callback(void* e);
void tim_set_timer(int delta, cb_t cb);
uint64_t tim_get_micros(void);
*/
void jacdac_reset(void);


#endif // MICROPY_INCLUDED_NRF_COMMON_HAL_BUSIO_JACDAC_H
