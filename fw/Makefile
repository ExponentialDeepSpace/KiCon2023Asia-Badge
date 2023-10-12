# ------------------------------------------------
#
# @file Makefile (based on gcc)
# @author Dewin
# @version v1.0.0
#
# ChangeLog :
#   2021-07-20
# ------------------------------------------------

######################################
# target
######################################
TARGET = output

######################################
# building variables
######################################
# debug build
ifeq ($(release), y)
DEBUG = 0
else
DEBUG = 1
endif

# optimization

ifeq ($(DEBUG), 1)
OPT = -O0
else
OPT = -Os
endif

#######################################
# Build path
#######################################
BUILD_DIR = build

######################################
# chip platform info
######################################
TARGET_PLATFORM := n32l40x
DEFS += -DN32L40X
DEFS += -DUSE_STDPERIPH_DRIVER

SYSCLK_USE_MSI=0
SYSCLK_USE_HSI=1
SYSCLK_USE_HSE=2
SYSCLK_USE_HSI_PLL=3
SYSCLK_USE_HSE_PLL=4

DEFS += -DSYSCLK_FREQ=48000000 # for using USB

# HSI
DEFS += -DSYSCLK_SRC=$(SYSCLK_USE_HSI_PLL)
DEFS += -DXTALLESS=1

# HSE
# DEFS += -DHSE_VALUE=8000000
# DEFS += -DSYSCLK_SRC=SYSCLK_USE_HSE_PLL

DEFS += -DPMA_ACCESS=1

ifeq ($(DEBUG), 1)
DEFS += -DUSE_FULL_ASSERT
endif

######################################
# Algo libs
######################################
USELIB = 0

######################################
#TOOLS CHAIN
######################################
CROSS_COMPILE = arm-none-eabi-

######################################
# C sources
######################################
C_DIRS += src src/setup src/usb
C_DIRS += freertos
C_DIRS += freertos/portable/GCC/ARM_CM4F
C_DIRS += freertos/portable/MemMang
# C_DIRS += freertos/portable/GCC/MemMang
C_DIRS += CherryUSB/core
C_DIRS += CherryUSB/class/cdc
C_DIRS += CherryUSB/port/fsdev
C_DIRS += bsp/firmware/CMSIS/device
C_DIRS += bsp/firmware/$(TARGET_PLATFORM)_std_periph_driver/src
# C_DIRS += bsp/firmware/$(TARGET_PLATFORM)_usbfs_driver/src
SRC_OBJS_DIRS += $(foreach DIR, $(C_DIRS), $(wildcard $(DIR)/*.c))
C_SOURCES = $(SRC_OBJS_DIRS) 

######################################
# ASM sources
######################################
ASM_SOURCES = bsp/firmware/CMSIS/device/startup/startup_$(TARGET_PLATFORM)_gcc.s

######################################
# C includes
######################################
C_INCS += include
C_INCS += freertos/include
C_INCS += freertos/portable/GCC/ARM_CM4F
C_INCS += CherryUSB/common
C_INCS += CherryUSB/core
C_INCS += CherryUSB/class/cdc
C_INCS += CherryUSB/port/fsdev
C_INCS += bsp/firmware/CMSIS/core
C_INCS += bsp/firmware/CMSIS/device
C_INCS += bsp/firmware/$(TARGET_PLATFORM)_std_periph_driver/inc
# C_INCS += bsp/firmware/$(TARGET_PLATFORM)_usbfs_driver/inc

INCS_OBJS_DIR = $(foreach DIR2, $(C_INCS), $(wildcard $(DIR2)/*.h))
INCS_OBJS_PATH = $(sort $(dir $(INCS_OBJS_DIR)))
C_INCLUDES = $(addprefix -I,$(INCS_OBJS_PATH))

######################################
# Lib files
######################################
ifeq ($(USELIB), 1)
SRC_LIB_DIR += bsp/firmware/$(TARGET_PLATFORM)_algo_lib/src
INC_LIB_DIR += bsp/firmware/$(TARGET_PLATFORM)_algo_lib/inc
LIB_SOURCES = $(foreach DIR3, $(SRC_LIB_DIR), $(wildcard $(DIR3)/*.lib))
LIB_SOURCES_L = $(addprefix -L,$(LIB_SOURCES))
C_LIBS = $(LIB_SOURCES_L)
LIB_INCS = $(foreach DIR4, $(INC_LIB_DIR), $(wildcard $(DIR4)/*.h))
LIB_INCS_PATH = $(sort $(dir $(LIB_INCS)))
LIB_INCS_PATH_I = $(addprefix -I,$(LIB_INCS_PATH))
C_INCLUDES += $(LIB_INCS_PATH_I)
endif

######################################
# Compile & Link flags
######################################
# cpu
CPU = -mcpu=cortex-m4
# fpu
FPU = -mfpu=fpv4-sp-d16
# float-abi
FLOAT-ABI = -mfloat-abi=hard
# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)
#CFLAGS
CFLAGS += $(MCU) -Wall
CFLAGS += $(OPT)
CFLAGS += -ffunction-sections -fdata-sections
#DEBUG
ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif
# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"
#LFLAGS
LFLAGS += $(MCU)
LFLAGS += -Wl,--gc-sections 
LFLAGS += --specs=nosys.specs
LFLAGS += -Xlinker -Map=$(BUILD_DIR)/$(TARGET).map
# link script
LDSCRIPT = bsp/firmware/CMSIS/device/$(TARGET_PLATFORM)_flash.ld

######################################
# Objects
######################################
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CROSS_COMPILE)gcc $(CFLAGS) $(DEFS) $(C_INCLUDES) $(C_LIBS) -c -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(CROSS_COMPILE)gcc $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CROSS_COMPILE)gcc $(OBJECTS) $(LFLAGS) -T$(LDSCRIPT) -o $@
	$(CROSS_COMPILE)size $@

$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	$(CROSS_COMPILE)objcopy -O binary -S $< $@

$(BUILD_DIR)/$(TARGET).hex: $(BUILD_DIR)/$(TARGET).elf
	$(CROSS_COMPILE)objcopy -O ihex -S $< $@

$(BUILD_DIR):
	@-mkdir $@

#######################################
# clean up
#######################################
clean:
	@-rm -rf $(BUILD_DIR)

#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

#######################################
# download .hex/.bin by jlink
#######################################
#Your JLink installation directory
PATH_WINPC = 'C:/Program Files (x86)/SEGGER/JLink_V690a/'
#PATH_LINUX = /opt/SEGGER/JLink_V640b/JLinkExe
JK_DPATH = $(PATH_WINPC)
#Jlink script store directory
JKS_DIR = ../../../../../../jlink
#Chip type
CHIP_TYPE = N32L436MB
download:
	@$(JK_DPATH)JLink.exe -device $(CHIP_TYPE) -if SWD -speed 4000 -autoconnect 1 -CommanderScript $(JKS_DIR)/flash.jlink
	@echo "Download Completed!"

debug:
	@$(JK_DPATH)JLinkGDBServer.exe -select USB -device $(CHIP_TYPE) -if SWD -speed auto -noir -LocalhostOnly

# *** EOF ***
