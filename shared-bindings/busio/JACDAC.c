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

#include <stdint.h>

#include "shared-bindings/busio/JACDAC.h"
#include "shared-bindings/microcontroller/Pin.h"
#include "shared-bindings/util.h"

#include "lib/utils/context_manager_helpers.h"
#include "lib/utils/interrupt_char.h"

#include "py/ioctl.h"
#include "py/objproperty.h"
#include "py/runtime.h"
#include "py/stream.h"
#include "supervisor/shared/translate.h"

#define STREAM_DEBUG(...) (void)0
// #define STREAM_DEBUG(...) mp_printf(&mp_plat_print __VA_OPT__(,) __VA_ARGS__)

//| class JACDAC:
//|     """A bidirectional serial protocol"""
//|     def __init__(self, tx: microcontroller.Pin, receiver_buffer_size: int = 64) -> None:
//|         """A common bidirectional serial protocol that uses an an agreed upon speed
//|         rather than a shared clock line.
//|
//|         :param ~microcontroller.Pin tx: the pin to transmit with, or ``None`` if this ``UART`` is receive-only.
//|         :param int receiver_buffer_size: the character length of the read buffer (0 to disable). (When a character is 9 bits the buffer will be 2 * receiver_buffer_size bytes.)
//|         ...
//|
STATIC mp_obj_t busio_jacdac_make_new(const mp_obj_type_t *type, size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    // Always initially allocate the UART object within the long-lived heap.
    // This is needed to avoid crashes with certain UART implementations which
    // cannot accomodate being moved after creation. (See
    // https://github.com/adafruit/circuitpython/issues/1056)
    busio_jacdac_obj_t *self = m_new_ll_obj(busio_jacdac_obj_t);
    self->base.type = &busio_jacdac_type;
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_tx, MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_receiver_buffer_size, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 10} }
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    const mcu_pin_obj_t* tx = validate_obj_is_free_pin_or_none(args[ARG_tx].u_obj);

    if ( (tx == NULL)) {
        mp_raise_ValueError(translate("tx and rx cannot both be None"));
    }

    common_hal_busio_jacdac_construct(self, tx, args[ARG_receiver_buffer_size].u_int, NULL, false);
    return (mp_obj_t)self;
}

//|     def deinit(self) -> None:
//|         """Deinitialises JACDAC and releases any hardware resources for reuse."""
//|         ...
//|
STATIC mp_obj_t busio_jacdac_obj_deinit(mp_obj_t self_in) {
    busio_jacdac_obj_t *self = MP_OBJ_TO_PTR(self_in);
    common_hal_busio_jacdac_deinit(self);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(busio_jacdac_deinit_obj, busio_jacdac_obj_deinit);

STATIC void check_for_deinit(busio_jacdac_obj_t *self) {
    if (common_hal_busio_jacdac_deinited(self)) {
        raise_deinited_error();
    }
}

//|     def __enter__(self) -> UART:
//|         """No-op used by Context Managers."""
//|         ...
//|
//  Provided by context manager helper.

//|     def __exit__(self) -> None:
//|         """Automatically deinitializes the hardware when exiting a context. See
//|         :ref:`lifetime-and-contextmanagers` for more info."""
//|         ...
//|
STATIC mp_obj_t busio_jacdac_obj___exit__(size_t n_args, const mp_obj_t *args) {
    (void)n_args;
    common_hal_busio_jacdac_deinit(args[0]);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(busio_jacdac___exit___obj, 4, 4, busio_jacdac_obj___exit__);

// These are standard stream methods. Code is in py/stream.c.
//
//|     def receive(self):
//|         """Read a single JACDAC packet.
//|
//|         :return: JACDAC packet
//|         :rtype: bytes or None"""
//|         ...
//|

//|     def send(self, buf: WriteableBuffer) -> Optional[int]:
//|         """Write the buffer of bytes to the bus.
//|
//|       *New in CircuitPython 4.0:* ``buf`` must be bytes, not a string.
//|
//|         :return: the number of bytes written
//|         :rtype: int or None"""
//|         ...
//|

// These three methods are used by the shared stream methods.
STATIC mp_uint_t busio_jacdac_read(mp_obj_t self_in, void *buf_in) {
    STREAM_DEBUG("busio_uart_read stream %d\n", size);
    busio_jacdac_obj_t *self = MP_OBJ_TO_PTR(self_in);
    check_for_deinit(self);
    byte *buf = buf_in;
    return common_hal_busio_jacdac_read(self, buf);
}

STATIC mp_uint_t busio_jacdac_send(mp_obj_t self_in, const void *buf_in) {
    busio_jacdac_obj_t *self = MP_OBJ_TO_PTR(self_in);
    check_for_deinit(self);
    const byte *buf = buf_in;
    return common_hal_busio_jacdac_write(self, buf);
}

STATIC const mp_rom_map_elem_t busio_jacdac_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_deinit),       MP_ROM_PTR(&busio_jacdac_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR___enter__),    MP_ROM_PTR(&default___enter___obj) },
    { MP_ROM_QSTR(MP_QSTR___exit__),     MP_ROM_PTR(&busio_jacdac___exit___obj) },

    { MP_ROM_QSTR(MP_QSTR_send),     MP_ROM_PTR(&mp_stream_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_receive), MP_ROM_PTR(&mp_stream_unbuffered_readline_obj)},

    // Properties
    { MP_NEW_QSTR(MP_QSTR_packets),     MP_ROM_PTR(&busio_jacdac_packets_obj) },
};
STATIC MP_DEFINE_CONST_DICT(busio_jacdac_locals_dict, busio_jacdac_locals_dict_table);

const mp_obj_type_t busio_jacdac_type = {
    { &mp_type_type },
    .name = MP_QSTR_JACDAC,
    .make_new = busio_jacdac_make_new,
    .locals_dict = (mp_obj_dict_t*)&busio_jacdac_locals_dict,
};
