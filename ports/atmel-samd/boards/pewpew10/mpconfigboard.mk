USB_VID = 0x239A
USB_PID = 0x80D5
USB_PRODUCT = "PewPew 10.2"
USB_MANUFACTURER = "Radomir Dopieralski"

CHIP_VARIANT = SAMD21E18A
CHIP_FAMILY = samd21

INTERNAL_FLASH_FILESYSTEM = 1
LONGINT_IMPL = NONE
CIRCUITPY_FULL_BUILD = 0

CIRCUITPY_PEW = 1
CIRCUITPY_ANALOGIO = 1
CIRCUITPY_MATH = 1
CIRCUITPY_NEOPIXEL_WRITE = 1
CIRCUITPY_ROTARYIO = 0
CIRCUITPY_RTC = 0
CIRCUITPY_SAMD = 0
CIRCUITPY_USB_MIDI = 0

SUPEROPT_GC = 0

FROZEN_MPY_DIRS += $(TOP)/frozen/pew-pewpew-standalone-10.x

CFLAGS_BOARD = --param max-inline-insns-auto=15
