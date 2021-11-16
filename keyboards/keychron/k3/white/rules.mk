# project specific files
SRC = matrix.c
SRC += rand.c

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

# BOOTLOADER = flash
SN32_BOOTLOADER_ADDRESS = 0x1FFF0009
# NB! NEED TO CHECK BOOTLOADER ADDRESS

# Optimize for size
OPT_DEFS = -Os
# NB! fixes stack size, needed for 260x chip (ref. Glory ion chat) - OK

# Define optical switches matrix 
OPT_DEFS += -DOPTICAL_MATRIX
# NB! (taken from K7 bringup), -D is argument, defined OPTICAL_MATRIX for keyboard, references config.h - OK

# Build Options
#   comment out to disable the options.
#   NB! Need to check what I want to include here, but these seem sensible.
MAGIC_ENABLE = yes
MAGIC_KEYCODE_ENABLE = yes
BOOTMAGIC_ENABLE = full 	# Virtual DIP switch configuration
EXTRAKEY_ENABLE = yes   	# Audio control and System control
NKRO_ENABLE = yes       	# USB Nkey Rollover
DIP_SWITCH_ENABLE = yes

# Custom Key and LED matrix handling
#   NB! Need to check what I want to include here, but these seem sensible.
CUSTOM_MATRIX = yes
RGB_MATRIX_ENABLE = yes
RGB_MATRIX_DRIVER = custom
RGB_MATRIX_CUSTOM_KB = yes

# Some options to reduce RAM usage
#   NB! Seems specific to 26x MCU, so OK
LDFLAGS += --specs=nano.specs
OPT_DEFS += -DCORTEX_ENABLE_WFI_IDLE=TRUE
USE_LINK_GC = yes
LTO_ENABLE = yes

# Reduce code size
#   NB! Seems specific to 26x MCU, so OK
USE_PROCESS_STACKSIZE = 0x1E0
USE_EXCEPTIONS_STACKSIZE = 0x180
