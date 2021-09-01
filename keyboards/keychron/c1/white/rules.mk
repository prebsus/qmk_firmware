# project specific files
SRC = matrix.c

## chip/board settings
# - the next two should match the directories in
#   <chibios>/os/hal/ports/$(MCU_FAMILY)/$(MCU_SERIES)
MCU_FAMILY = SN32
MCU_SERIES = SN32F260

# Linker script to use
# - it should exist either in <chibios>/os/common/ports/ARMCMx/compilers/GCC/ld/
#   or <this_dir>/ld/
MCU_LDSCRIPT = SN32F260

# Startup code to use
#  - it should exist in <chibios>/os/common/startup/ARMCMx/compilers/GCC/mk/
MCU_STARTUP = sn32f26x

# Board: it should exist either in <chibios>/os/hal/boards/
#  or <this_dir>/boards
BOARD = SN_SN32F260

# Cortex version
MCU  = cortex-m0

# ARM version, CORTEX-M0/M1 are 6, CORTEX-M3/M4/M7 are 7
ARMV = 6

OPT_DEFS = -O2

# Build Options
#   comment out to disable the options.
#
LTO_ENABLE = no
MAGIC_ENABLE = yes
MAGIC_KEYCODE_ENABLE = yes
BOOTMAGIC_ENABLE = full 	# Virtual DIP switch configuration
MOUSEKEY_ENABLE = no    	# Mouse keys
EXTRAKEY_ENABLE = yes   	# Audio control and System control
CONSOLE_ENABLE = no     	# Console for debug
COMMAND_ENABLE = no     	# Commands for debug and configuration
SLEEP_LED_ENABLE = no   	# Breathing sleep LED during USB suspend
NKRO_ENABLE = yes       	# USB Nkey Rollover
AUDIO_ENABLE = no
RGBLIGHT_ENABLE = no
SERIAL_LINK_ENABLE = no
WAIT_FOR_USB = no
DIP_SWITCH_ENABLE = yes

# Custom Key and LED matrix handling
CUSTOM_MATRIX = yes
BACKLIGHT_ENABLE = yes
BACKLIGHT_DRIVER = custom
#LED_MATRIX_ENABLE = yes
#LED_MATRIX_DRIVER = custom

# Reduce code size
USE_PROCESS_STACKSIZE = 0x1E0
USE_EXCEPTIONS_STACKSIZE = 0x180

COMPILEFLAGS += --specs=nano.specs