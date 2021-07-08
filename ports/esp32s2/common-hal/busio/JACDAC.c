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
#include "shared-bindings/microcontroller/Pin.h"

#include "lib/utils/interrupt_char.h"
#include "py/mpconfig.h"
#include "py/gc.h"
#include "py/mperrno.h"
#include "py/runtime.h"
#include "supervisor/shared/translate.h"

#include "common-hal/busio/JACDAC.h"
#include "common-hal/busio/UART.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "components/driver/include/driver/periph_ctrl.h"

// #define PIN_LOG_0 GPIO_NUM_3
// #define PIN_LOG_1 GPIO_NUM_4

#define LOG NOLOG
// #define LOG DMESG

#define UART_EMPTY_THRESH_DEFAULT (10)
#define UART_FULL_THRESH_DEFAULT (120)
#define UART_TOUT_THRESH_DEFAULT (10)

#define NUM_HW_ALLOC 2

static busio_jacdac_hw_alloc_t jd_hw_alloc[NUM_HW_ALLOC];

static IRAM_ATTR void uart_isr(busio_jacdac_obj_t *context);

static void log_pin_pulse(int pinid, int numpulses) {
    #ifdef PIN_LOG_0
    uint32_t mask = pinid == 0 ? 1 << PIN_LOG_0 : 1 << PIN_LOG_1;
    while (numpulses--) {
        GPIO.out_w1ts = mask;
        GPIO.out_w1tc = mask;
    }
    #endif
}

static void jd_timer(busio_jacdac_obj_t *context) {
    busio_jacdac_base_callback_t f = context->timer_cb;
    if (f) {
        context->timer_cb = NULL;
        f(context);
    }
}

static void jd_panic(void) {
    abort();
}

#define CHK(e)                                                                                     \
    if ((e) != ESP_OK)                                                                             \
    jd_panic()


void common_hal_busio_jacdac_construct(busio_jacdac_obj_t *context, const mcu_pin_obj_t *pin) {
    if (!(1 <= pin->number && pin->number < 32)) {
        // we do not support pins from out1_*, and GPIO0 doesn't sound like a good idea either
        mp_raise_ValueError(translate("Invalid argument"));
    }

    context->uart_num = UART_NUM_MAX;
    for (uart_port_t num = 0; num < UART_NUM_MAX; num++) {
        if (!uart_is_used(num)) {
            context->uart_num = num;
        }
    }
    if (context->uart_num == UART_NUM_MAX) {
        mp_raise_ValueError(translate("All UART peripherals are in use"));
    }

    DMESG("Jacdac on UART%d IO%d", context->uart_num, pin->number);

    context->uart_hw = UART_LL_GET_HW(context->uart_num);

    context->hw_alloc = NULL;

    for (int i = 0; i < NUM_HW_ALLOC; ++i) {
        if (!jd_hw_alloc[i].timer) {
            context->hw_alloc = &jd_hw_alloc[i];
            break;
        }
    }

    if (!context->hw_alloc) {
        // this is very unlikely - we would first run out of UARTs
        mp_raise_ValueError(translate("Invalid argument"));
    }

    busio_jacdac_init(&context->base);

    esp_timer_create_args_t args;
    args.callback = (esp_timer_cb_t)jd_timer;
    args.arg = context;
    args.dispatch_method = ESP_TIMER_TASK;
    args.name = "JD timeout";
    esp_timer_create(&args, &context->hw_alloc->timer);

    claim_pin(pin);
    context->pinobj = pin;
    context->pin_num = pin->number;

    context->hw_alloc->pin_num = pin->number;
    context->hw_alloc->uart_num = context->uart_num;

    uart_mark_used(context->uart_num, true);

    periph_module_enable(uart_periph_signal[context->uart_num].module);

    const uart_config_t uart_config =
    {.baud_rate = 1000000,
     .data_bits = UART_DATA_8_BITS,
     .parity = UART_PARITY_DISABLE,
     .stop_bits = UART_STOP_BITS_1,
     .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    CHK(uart_param_config(context->uart_num, &uart_config));
    CHK(uart_isr_register(context->uart_num, (void (*)(void *))uart_isr, context, 0, &context->hw_alloc->intr_handle));

    uart_intr_config_t uart_intr =
    {.intr_enable_mask = 0,
     .rxfifo_full_thresh = UART_FULL_THRESH_DEFAULT,
     .rx_timeout_thresh = 30,        // 30us
     .txfifo_empty_intr_thresh = UART_EMPTY_THRESH_DEFAULT};
    CHK(uart_intr_config(context->uart_num, &uart_intr));

    gpio_config_t cfg = {
        .pin_bit_mask = BIT64(context->pin_num),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = true,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);

    // gpio_set_pull_mode(context->pin_num, GPIO_PULLUP_ONLY);
    // gpio_set_direction(context->pin_num, GPIO_MODE_INPUT);

    #ifdef PIN_LOG_0
    gpio_set_direction(PIN_LOG_0, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_LOG_1, GPIO_MODE_OUTPUT);
    #endif

    common_hal_busio_jacdac_cancel(context);
}

static void jd_deinit_hw_alloc(busio_jacdac_hw_alloc_t *hw_alloc) {
    if (!hw_alloc->timer) {
        return;
    }

    uart_dev_t *uart_hw = UART_LL_GET_HW(hw_alloc->uart_num);

    esp_timer_stop(hw_alloc->timer);
    esp_timer_delete(hw_alloc->timer);
    hw_alloc->timer = NULL;
    uart_mark_used(hw_alloc->uart_num, false);

    uart_hw->int_clr.val = 0xffffffff;
    uart_hw->int_ena.val = 0;

    esp_intr_free(hw_alloc->intr_handle);
    hw_alloc->intr_handle = NULL;

    periph_module_disable(uart_periph_signal[hw_alloc->uart_num].module);
    common_hal_mcu_pin_reset_number(hw_alloc->pin_num);
}

void jacdac_reset() {
    DMESG("jacdac_reset");
    for (int i = 0; i < NUM_HW_ALLOC; ++i) {
        jd_deinit_hw_alloc(&jd_hw_alloc[i]);
    }
}

void common_hal_busio_jacdac_deinit(busio_jacdac_obj_t *context) {
    if (common_hal_busio_jacdac_deinited(context)) {
        return;
    }

    jd_deinit_hw_alloc(context->hw_alloc);
    context->hw_alloc = NULL;
    context->pinobj = NULL;

    busio_jacdac_deinit(&context->base);
}

bool common_hal_busio_jacdac_deinited(busio_jacdac_obj_t *context) {
    return context->hw_alloc == NULL || context->hw_alloc->timer == NULL;
}

void common_hal_busio_jacdac_set_timer(busio_jacdac_obj_t *context, uint32_t us, busio_jacdac_base_callback_t callback) {
    common_hal_mcu_disable_interrupts();
    if (context->hw_alloc) {
        context->timer_cb = callback;
        esp_timer_stop(context->hw_alloc->timer);
        if (callback) {
            esp_timer_start_once(context->hw_alloc->timer, us);
        }
    }
    common_hal_mcu_enable_interrupts();
}

static IRAM_ATTR esp_err_t xgpio_set_level(gpio_num_t gpio_num, uint32_t level) {
    if (level) {
        GPIO.out_w1ts = (1 << gpio_num);
    } else {
        GPIO.out_w1tc = (1 << gpio_num);
    }
    return ESP_OK;
}

static IRAM_ATTR void pin_rx(busio_jacdac_obj_t *context) {
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[context->pin_num], PIN_FUNC_GPIO);
    REG_SET_BIT(GPIO_PIN_MUX_REG[context->pin_num], FUN_PU);
    PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[context->pin_num]);
    GPIO.enable_w1tc = (0x1 << context->pin_num);
    REG_WRITE(GPIO_FUNC0_OUT_SEL_CFG_REG + (context->pin_num * 4), SIG_GPIO_OUT_IDX);
    gpio_matrix_in(context->pin_num, uart_periph_signal[context->uart_num].rx_sig, 0);
}

static IRAM_ATTR void pin_tx(busio_jacdac_obj_t *context) {
    gpio_matrix_in(GPIO_FUNC_IN_HIGH, uart_periph_signal[context->uart_num].rx_sig, 0); // context->uart_hw
    GPIO.pin[context->pin_num].int_type = GPIO_PIN_INTR_DISABLE;
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[context->pin_num], PIN_FUNC_GPIO);
    gpio_set_level(context->pin_num, 1);
    gpio_matrix_out(context->pin_num, uart_periph_signal[context->uart_num].tx_sig, 0, 0);
}

int common_hal_busio_jacdac_wait_high(busio_jacdac_obj_t *context) {
    // we already started RX at this point
    return 0;
}

static IRAM_ATTR void fill_fifo(busio_jacdac_obj_t *context) {
    if (!context->tx_len) {
        return;
    }

    int space = UART_FIFO_LEN - context->uart_hw->status.txfifo_cnt;
    if (context->tx_len < space) {
        space = context->tx_len;
    }

    uart_ll_write_txfifo(context->uart_hw, context->fifo_buf, space);

    context->fifo_buf += space;
    context->tx_len -= space;

    if (context->tx_len == 0) {
        LOG("txbrk");
        context->uart_hw->idle_conf.tx_brk_num = 14; // 14us
        context->uart_hw->conf0.txd_brk = 1;
        context->uart_hw->int_clr.tx_brk_done = 1;
        context->uart_hw->int_ena.tx_brk_done = 1;
    }

    context->uart_hw->int_clr.txfifo_empty = 1;
    context->uart_hw->conf1.txfifo_empty_thrhd = UART_EMPTY_THRESH_DEFAULT;
    context->uart_hw->int_ena.txfifo_empty = 1;
}

static IRAM_ATTR void read_fifo(busio_jacdac_obj_t *context, int force) {
    uart_dev_t *uart_reg = context->uart_hw;
    int rx_fifo_len = uart_reg->status.rxfifo_cnt;

    if (!force && context->fifo_buf == NULL && rx_fifo_len < UART_FULL_THRESH_DEFAULT - 1) {
        return; // read not started yet and we're not overflowing

    }
    if (rx_fifo_len) {
        LOG("rxfifo %d", rx_fifo_len);
        int n = rx_fifo_len;
        int needs_rst = 0;
        if (n > context->rx_len) {
            n = context->rx_len;
            needs_rst = 1;
        }

        if (n) {
            context->rx_len -= n;
            rx_fifo_len -= n;
            uart_ll_read_rxfifo(uart_reg, context->fifo_buf, n);
            context->fifo_buf += n;
        }

        // and drop the rest of data
        if (needs_rst) {
            uart_ll_rxfifo_rst(uart_reg);
        }
    }
}

void common_hal_busio_jacdac_cancel(busio_jacdac_obj_t *context) {
    context->uart_hw->int_clr.val = 0xffffffff;
    context->uart_hw->int_ena.val = UART_BRK_DET_INT_ENA;
    context->seen_low = 0;
    context->rx_len = context->tx_len = 0;
    context->fifo_buf = NULL;
    context->rx_ended = 0;
    read_fifo(context, 1);
    pin_rx(context);
    log_pin_pulse(1, 1);
}

void common_hal_busio_jacdac_force_read(busio_jacdac_obj_t *context) {
    read_fifo(context, 1);
}

#define END_RX_FLAGS (UART_RXFIFO_TOUT_INT_ST | UART_BRK_DET_INT_ST | UART_FRM_ERR_INT_ST)

static IRAM_ATTR void start_bg_rx(busio_jacdac_obj_t *context) {
    read_fifo(context, 1); // flush any data
    context->seen_low = 1;
    context->uart_hw->int_ena.val |= END_RX_FLAGS | UART_RXFIFO_FULL_INT_ENA;
    if (!context->fifo_buf) {
        busio_jacdac_line_falling(context);
    }
}

static IRAM_ATTR void uart_isr(busio_jacdac_obj_t *context) {
    log_pin_pulse(0, 1);

    if (!context->hw_alloc) {
        return;
    }

    uart_dev_t *uart_reg = context->uart_hw;

    uint32_t uart_intr_status = uart_reg->int_st.val;
    uart_reg->int_clr.val = uart_intr_status; // clear all

    LOG("ISR %x", uart_intr_status);

    read_fifo(context, 0);

    if (!context->seen_low && (uart_intr_status & UART_BRK_DET_INT_ST)) {
        log_pin_pulse(0, 2);
        start_bg_rx(context);
    } else if (uart_intr_status & UART_TX_BRK_DONE_INT_ST) {
        uart_reg->conf0.txd_brk = 0;
        common_hal_busio_jacdac_cancel(context);
        busio_jacdac_tx_completed(context);
    } else if (uart_intr_status & UART_TXFIFO_EMPTY_INT_ST) {
        uart_reg->int_ena.txfifo_empty = 0;
        fill_fifo(context);
    } else if (uart_intr_status & END_RX_FLAGS) {
        log_pin_pulse(0, 4);
        context->data_left = context->rx_len;
        int had_buf = context->fifo_buf != NULL;
        LOG("%d end, rx=%d %d", (int)esp_timer_get_time(), context->rx_len, had_buf);
        common_hal_busio_jacdac_cancel(context);
        if (had_buf) {
            log_pin_pulse(0, 5);
            busio_jacdac_rx_completed(context);
        } else {
            context->rx_ended = 1;
        }
    }
}

static IRAM_ATTR NOINLINE_ATTR void probe_and_set(volatile uint32_t *oe, volatile uint32_t *inp,
    uint32_t mask) {
    *oe = *inp & mask;
}

static void tx_race(busio_jacdac_obj_t *context) {
    // don't reconnect the pin in the middle of the low-pulse
    int timeout = 50000;
    while (timeout-- > 0 && gpio_get_level(context->pin_num) == 0) {
        ;
    }
    pin_rx(context);
    start_bg_rx(context);
}

IRAM_ATTR static void target_wait_us(uint32_t us) {
    int64_t later = esp_timer_get_time() + us;
    while (esp_timer_get_time() < later) {
        ;
    }
}

IRAM_ATTR int common_hal_busio_jacdac_start_tx(busio_jacdac_obj_t *context, const void *data, uint32_t numbytes) {
    if (context->tx_len) {
        jd_panic();
    }

    if (!context->hw_alloc || context->seen_low || context->uart_hw->int_raw.brk_det) {
        LOG("seen low %p %d %d", context->hw_alloc, context->seen_low, context->uart_hw->int_raw.brk_det);
        return -1;
    }

    common_hal_mcu_disable_interrupts();

    gpio_matrix_in(GPIO_FUNC_IN_HIGH, uart_periph_signal[context->uart_num].rx_sig, 0); // context->uart_hw
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[context->pin_num], PIN_FUNC_GPIO);
    GPIO.out_w1tc = (1 << context->pin_num);

    probe_and_set(&GPIO.enable_w1ts, &GPIO.in, 1 << context->pin_num);

    if (!(GPIO.enable & (1 << context->pin_num))) {
        // the line went down in the meantime
        tx_race(context);
        common_hal_mcu_enable_interrupts();
        return -1;
    }

    target_wait_us(12); // low pulse is 14us with wait of 12 here
    xgpio_set_level(context->pin_num, 1);

    target_wait_us(40); // ~55us from end of low pulse to start bit

    pin_tx(context);

    context->fifo_buf = (uint8_t *)data;
    context->tx_len = numbytes;

    context->uart_hw->int_clr.val = 0xffffffff;

    fill_fifo(context);

    common_hal_mcu_enable_interrupts();

    return 0;
}

void uart_flush_rx(busio_jacdac_obj_t *context) {
    common_hal_mcu_disable_interrupts();
    read_fifo(context, 1);
    common_hal_mcu_enable_interrupts();
}

void common_hal_busio_jacdac_start_rx(busio_jacdac_obj_t *context, void *data, uint32_t maxbytes) {
    if (context->rx_len || context->tx_len) {
        jd_panic();
    }

    log_pin_pulse(0, 3);

    context->fifo_buf = data;
    context->rx_len = maxbytes;
    LOG("ini rx=%d", maxbytes);

    uart_flush_rx(context);

    // log_pin_pulse(0, 2);

    if (context->rx_ended) {
        context->rx_ended = 0;
        context->rx_len = 0;
        context->fifo_buf = NULL;
        // log_pin_pulse(0, 2);
        busio_jacdac_rx_completed(context);
    }
}
