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
#include "shared-bindings/time/__init__.h"
#include "shared-bindings/microcontroller/__init__.h"

#include "lib/utils/buffer_helper.h"
#include "lib/utils/context_manager_helpers.h"
#include "lib/utils/interrupt_char.h"

#include "py/ioctl.h"
#include "py/objproperty.h"
#include "py/runtime.h"
#include "supervisor/shared/translate.h"

#include "common-hal/busio/JACDAC.h"

#define JD_TX_QUEUE_SIZE 1024
#define JD_RX_QUEUE_SIZE 1024
#define JD_RX_TMP_QUEUE_SIZE 512

//| class JACDAC:
//|     """A bidirectional single wire serial protocol"""
//|     def __init__(self, pin: microcontroller.Pin) -> None:
//|         """A 1mbaud packet transfer protocol.
//|
//|         :param ~microcontroller.Pin pin: the pin to transmit with."""
//|         ...
//|
STATIC mp_obj_t busio_jacdac_make_new(const mp_obj_type_t *type, size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    // Always initially allocate the UART (so also JACDAC) object within the long-lived heap.
    // This is needed to avoid crashes with certain UART implementations which
    // cannot accommodate being moved after creation. (See
    // https://github.com/adafruit/circuitpython/issues/1056)
    busio_jacdac_obj_t *self = m_new_ll_obj(busio_jacdac_obj_t);
    self->base.base.type = &busio_jacdac_type;

    enum { ARG_pin };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_pin, MP_ARG_OBJ, {.u_obj = mp_const_none} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    const mcu_pin_obj_t *pin = validate_obj_is_free_pin(args[ARG_pin].u_obj);

    common_hal_busio_jacdac_construct(self, pin);
    return (mp_obj_t)self;
}

//|     def deinit(self) -> None:
//|         """Deinitializes JACDAC and releases any hardware resources for reuse."""
//|         ...
//|
STATIC mp_obj_t busio_jacdac_obj_deinit(mp_obj_t self_in) {
    busio_jacdac_obj_t *self = MP_OBJ_TO_PTR(self_in);
    common_hal_busio_jacdac_deinit(self);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(busio_jacdac_deinit_obj, busio_jacdac_obj_deinit);

STATIC void check_for_deinit(busio_jacdac_base_obj_t *self) {
    if (common_hal_busio_jacdac_deinited((busio_jacdac_obj_t *)self)) {
        raise_deinited_error();
    }
}

//|     def __enter__(self) -> JACDAC:
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


// https://wiki.nicksoft.info/mcu:pic16:crc-16:home
static uint16_t jd_crc16(const void *data, uint32_t size) {
    const uint8_t *ptr = (const uint8_t *)data;
    uint16_t crc = 0xffff;
    while (size--) {
        uint8_t b = *ptr++;
        uint8_t x = (crc >> 8) ^ b;
        x ^= x >> 4;
        crc = (crc << 8) ^ (x << 12) ^ (x << 5) ^ x;
    }
    return crc;
}

#define PACKET_OVERHEAD (JD_LINKED_FRAME_HEADER_SIZE + 12 + 4)

static int copy_and_append(jd_linked_frame_t *volatile *q, const uint8_t *data, int max_bytes) {
    size_t frm_sz = data[2] + 12;
    jd_linked_frame_t *buf = m_malloc(frm_sz + JD_LINKED_FRAME_HEADER_SIZE, false);

    buf->timestamp_ms = common_hal_time_monotonic_ms();

    memcpy(&buf->frame, data, frm_sz);

    common_hal_mcu_disable_interrupts();
    jd_linked_frame_t *last = *q;
    int num = 0;
    buf->next = NULL;
    while (last && last->next) {
        num += PACKET_OVERHEAD + last->frame.size;
        last = last->next;
    }
    if (last) {
        num += PACKET_OVERHEAD + last->frame.size;
    }
    if (num < max_bytes) {
        if (last) {
            last->next = buf;
        } else {
            *q = buf;
        }
        buf = NULL;
    }
    common_hal_mcu_enable_interrupts();

    if (buf == NULL) {
        return 0;
    } else {
        m_free(buf);
        return -1;
    }
}

//|     def send(self, buf: _typing.WriteableBuffer) -> None:
//|         """Queue a packet to be sent. Rises an exception if the transmit queue is full."""
//|         ...
//|
STATIC mp_obj_t busio_jacdac_send(mp_obj_t self_, mp_obj_t buffer) {
    busio_jacdac_base_obj_t *self = MP_OBJ_TO_PTR(self);
    check_for_deinit(self);

    // setup buffer
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buffer, &bufinfo, MP_BUFFER_RW);

    if (bufinfo.len < 16 || bufinfo.len > 252) {
        mp_raise_ValueError(translate("Invalid argument"));
    }

    uint8_t *p = bufinfo.buf;
    p[2] = bufinfo.len - 12;
    uint16_t crc = jd_crc16(p + 2, bufinfo.len - 2);
    p[0] = crc & 0xff;
    p[1] = crc >> 8;

    if (copy_and_append(&self->txQueue, bufinfo.buf, JD_TX_QUEUE_SIZE) != 0) {
        mp_raise_ValueError(translate("Jacdac TX queue full"));
    }

    // TODO poke queue

    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_2(busio_jacdac_send_obj, busio_jacdac_send);

#define JD_FRAME_FLAG_ACK_REQUESTED 0x02
#define ALIGN(x) (((x) + 3) & ~3)
static int jd_shift_frame(jd_frame_t *frame) {
    int psize = frame->size;
    int oldsz = frame->service_size + 4;
    if (ALIGN(oldsz) >= psize) {
        return 0; // nothing to shift
    }

    uint8_t *data = &frame->service_size;

    int ptr;
    if (data[oldsz] == 0xff) {
        ptr = data[oldsz + 1];
        if (ptr >= psize) {
            return 0; // End-of-frame
        }
        if (ptr <= oldsz) {
            return 0; // don't let it go back, must be some corruption
        }
    } else {
        ptr = ALIGN(oldsz);
    }

    // assume the first one got the ACK sorted
    frame->flags &= ~JD_FRAME_FLAG_ACK_REQUESTED;

    uint8_t *src = data + ptr;
    int newsz = *src + 4;
    if (ptr + newsz > psize) {
        return 0;
    }
    uint32_t *dst = (uint32_t *)(void *)data;
    uint32_t *srcw = (uint32_t *)(void *)src;
    // don't trust memmove()
    for (int i = 0; i < newsz; i += 4) {
        *dst++ = *srcw++;
    }
    // store ptr
    ptr += ALIGN(newsz);
    data[newsz] = 0xff;
    data[newsz + 1] = ptr;

    return 1;
}

//|     def receive(self) -> Optional[bytes]:
//|         """Fetches a Jacdac packet from reception queue. Returns None if queue is empty."""
//|         ...
//|
STATIC mp_obj_t busio_jacdac_receive(mp_obj_t self_) {
    busio_jacdac_base_obj_t *self = MP_OBJ_TO_PTR(self);
    check_for_deinit(self);
    mp_obj_t res = mp_const_none;
    jd_linked_frame_t *lnk = self->frameToSplit;

    if (lnk == NULL) {
        common_hal_mcu_disable_interrupts();
        lnk = self->rxQueue;
        if (lnk) {
            self->frameToSplit = lnk;
            self->rxQueue = lnk->next;
        }
        common_hal_mcu_enable_interrupts();
    }

    if (lnk) {
        res = mp_obj_new_bytes((void *)&lnk->frame, lnk->frame.service_size + 16);
        if (jd_shift_frame(&lnk->frame) == 0) {
            m_free(lnk);
            self->frameToSplit = NULL;
        }
    }

    return res;
}
MP_DEFINE_CONST_FUN_OBJ_1(busio_jacdac_receive_obj, busio_jacdac_receive);

// https://en.wikipedia.org/wiki/Fowler%E2%80%93Noll%E2%80%93Vo_hash_function
static uint32_t hash_fnv1(const void *data, unsigned len) {
    const uint8_t *d = (const uint8_t *)data;
    uint32_t h = 0x811c9dc5;
    while (len--) {
        h = (h * 0x1000193) ^ *d++;
    }
    return h;
}

static uint32_t jd_hash(uint8_t *buf, size_t length, int bits) {
    if (bits < 1) {
        return 0;
    }

    uint32_t h = hash_fnv1(buf, length);

    if (bits >= 32) {
        return h;
    } else {
        return (h ^ (h >> bits)) & ((1 << bits) - 1);
    }
}


STATIC mp_obj_t busio_jacdac_hash(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_buffer };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_buffer,     MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
    };

    busio_jacdac_base_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    check_for_deinit(self);

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // setup buffer
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(args[ARG_buffer].u_obj, &bufinfo, MP_BUFFER_READ);
    int32_t start = 0;
    size_t length = bufinfo.len;
    normalize_buffer_bounds(&start, INT_MAX, &length);

    // empty buffer
    if (length == 0) {
        return 0;
    }

    uint32_t h = jd_hash(((uint8_t *)bufinfo.buf), length, 30);

    vstr_t vstr;
    vstr_init_len(&vstr, 4);

    vstr.buf[0] = 0x41 + h % 26;
    vstr.buf[1] = 0x41 + (h / 26) % 26;
    vstr.buf[2] = 0x30 + (h / (26 * 26)) % 10;
    vstr.buf[3] = 0x30 + (h / (26 * 26 * 10)) % 10;

    return mp_obj_new_str_from_vstr(&mp_type_bytes, &vstr);
}
MP_DEFINE_CONST_FUN_OBJ_KW(busio_jacdac_hash_obj, 2, busio_jacdac_hash);


STATIC const mp_rom_map_elem_t busio_jacdac_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_deinit),       MP_ROM_PTR(&busio_jacdac_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR___enter__),    MP_ROM_PTR(&default___enter___obj) },
    { MP_ROM_QSTR(MP_QSTR___exit__),     MP_ROM_PTR(&busio_jacdac___exit___obj) },

    { MP_ROM_QSTR(MP_QSTR_send),     MP_ROM_PTR(&busio_jacdac_send_obj) },
    { MP_ROM_QSTR(MP_QSTR_receive),  MP_ROM_PTR(&busio_jacdac_receive_obj) },
    { MP_ROM_QSTR(MP_QSTR_hash),  MP_ROM_PTR(&busio_jacdac_hash_obj) },
};
STATIC MP_DEFINE_CONST_DICT(busio_jacdac_locals_dict, busio_jacdac_locals_dict_table);

const mp_obj_type_t busio_jacdac_type = {
    { &mp_type_type },
    .name = MP_QSTR_JACDAC,
    .make_new = busio_jacdac_make_new,
    .locals_dict = (mp_obj_dict_t *)&busio_jacdac_locals_dict,
};
