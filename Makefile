TARGET:=JuFo2015
TOOLCHAIN_ROOT:=~/sat/bin
TOOLCHAIN_PATH:=$(TOOLCHAIN_ROOT)
TOOLCHAIN_PREFIX:=arm-none-eabi

# Optimization level, can be [0, 1, 2, 3, s].
OPTLVL:=2
DBG:=-g

FREERTOS:=$(CURDIR)/FreeRTOS
STARTUP:=$(CURDIR)/hardware
LINKER_SCRIPT:=$(CURDIR)/Utilities/stm32_flash.ld

INCLUDE=-I$(CURDIR)/hardware
INCLUDE+=-I$(FREERTOS)/include
INCLUDE+=-I$(FREERTOS)/portable/GCC/ARM_CM4F
INCLUDE+=-I$(CURDIR)/Libraries/CMSIS/Device/ST/STM32F4xx/Include
INCLUDE+=-I$(CURDIR)/Libraries/CMSIS/Include
INCLUDE+=-I$(CURDIR)/Libraries/STM32F4xx_StdPeriph_Driver/inc
INCLUDE+=-I$(CURDIR)/config
INCLUDE+=-I$(CURDIR)/src/inc
INCLUDE+=-I$(CURDIR)/Utilities/STM32F4-Discovery
INCLUDE+=-I$(CURDIR)/Libraries/lib/inc
INCLUDE+=-I$(CURDIR)/Libraries/SSD1963/inc
INCLUDE+=-I$(CURDIR)/Libraries/ub_lib/inc
INCLUDE+=-I$(CURDIR)/Libraries/ub_lib/ub_font
INCLUDE+=-I$(CURDIR)/Libraries/SLAM/inc

BUILD_DIR = $(CURDIR)/build
BIN_DIR = $(CURDIR)/binary

# vpath is used so object files are written to the current directory instead
# of the same directory as their source files
vpath %.c $(CURDIR)/Libraries/STM32F4xx_StdPeriph_Driver/src \
	  $(CURDIR)/Libraries/syscall \
	  $(CURDIR)/hardware $(FREERTOS) \
	  $(FREERTOS)/portable/MemMang \
	  $(FREERTOS)/portable/GCC/ARM_CM4F \
	  $(CURDIR)/src/src $(CURDIR)/Utilities/STM32F4-Discovery \
	  $(CURDIR)/Libraries/lib/src \
	  $(CURDIR)/Libraries/SSD1963/src \
	  $(CURDIR)/Libraries/ub_lib/src \
	  $(CURDIR)/Libraries/ub_lib/ub_font \
	  $(CURDIR)/Libraries/ub_lib/ub_font/font \
	  $(CURDIR)/Libraries/SLAM/src
	  

vpath %.s $(STARTUP)
ASRC=startup_stm32f4xx.s

# Project Source Files
SRC+=stm32f4xx_it.c
SRC+=system_stm32f4xx.c
SRC+=main.c
SRC+=syscalls.c

# FreeRTOS Source Files
SRC+=port.c
SRC+=list.c
SRC+=queue.c
SRC+=tasks.c
SRC+=event_groups.c
SRC+=timers.c
SRC+=heap_4.c

# Standard Peripheral Source Files
SRC+=misc.c
SRC+=stm32f4xx_adc.c
#SRC+=stm32f4xx_can.c
#SRC+=stm32f4xx_crc.c
#SRC+=stm32f4xx_cryp_aes.c
#SRC+=stm32f4xx_cryp.c
#SRC+=stm32f4xx_cryp_des.c
#SRC+=stm32f4xx_cryp_tdes.c
#SRC+=stm32f4xx_dac.c
#SRC+=stm32f4xx_dbgmcu.c
#SRC+=stm32f4xx_dcmi.c
SRC+=stm32f4xx_dma.c
SRC+=stm32f4xx_exti.c
SRC+=stm32f4xx_flash.c
#SRC+=stm32f4xx_fsmc.c
SRC+=stm32f4xx_gpio.c
#SRC+=stm32f4xx_hash.c
#SRC+=stm32f4xx_hash_md5.c
#SRC+=stm32f4xx_hash_sha1.c
SRC+=stm32f4xx_i2c.c
SRC+=stm32f4xx_iwdg.c
SRC+=stm32f4xx_pwr.c
SRC+=stm32f4xx_rcc.c
#SRC+=stm32f4xx_rng.c
#SRC+=stm32f4xx_rtc.c
SRC+=stm32f4xx_sdio.c
SRC+=stm32f4xx_spi.c
SRC+=stm32f4xx_syscfg.c
SRC+=stm32f4xx_tim.c
SRC+=stm32f4xx_usart.c
SRC+=stm32f4xx_wwdg.c

# Standard Peripheral Utilities
SRC+=stm32f4_discovery.c
SRC+=stm32f4_discovery_lis302dl.c

# Other/own libs
SRC+=debug.c
SRC+=xv11.c
SRC+=gui.c
SRC+=utils.c
SRC+=gui_areaElements.c
SRC+=slam.c

#SSD1963
SRC+=SSD1963.c
SRC+=SSD1963_api.c

#ub_lib
#SRC+=stm32_ub_adc1_single.c
#SRC+=stm32_ub_pwm_tim2.c
SRC+=stm32_ub_pwm_tim3.c
#SRC+=stm32_ub_pwm_tim4.c
#SRC+=stm32_ub_pwm_tim5.c
#ub_font
SRC+=stm32_ub_font.c
#SRC+=ub_font_arial_10x15.c
#SRC+=ub_font_arial_11x18.c
#SRC+=ub_font_arial_13x19.c
#SRC+=ub_font_arial_14x22.c
#SRC+=ub_font_arial_16x25.c
#SRC+=ub_font_arial_18x27.c
#SRC+=ub_font_arial_7x10.c
#SRC+=ub_font_arial_8x13.c
SRC+=ub_font_pArial_10.c
SRC+=ub_font_pArial_13.c
SRC+=ub_font_pArial_16.c
SRC+=ub_font_pArial_21.c
#SRC+=ub_font_pComic_12.c
#SRC+=ub_font_pComic_16.c
#SRC+=ub_font_pComic_19.c
#SRC+=ub_font_pTimes_12.c
#SRC+=ub_font_pTimes_15.c
#SRC+=ub_font_pTimes_18.c

#CoreSLAM
#SRC+=CoreSLAM.c
#SRC+=CoreSLAM_ext.c
#SRC+=CoreSLAM_loop_closing.c
#SRC+=CoreSLAM_random.c
#SRC+=CoreSLAM_state.c

#SLAM
SRC+=slamcore.c

#lib
SRC+=printf.c
SRC+=stm32_ub_touch_ADS7843.c
SRC+=gui_graphics.c

CDEFS=-DUSE_STDPERIPH_DRIVER
CDEFS+=-DSTM32F4XX
CDEFS+=-DHSE_VALUE=8000000
CDEFS+=-D__FPU_PRESENT=1
CDEFS+=-D__FPU_USED=1
CDEFS+=-DARM_MATH_CM4

MCUFLAGS=-mcpu=cortex-m4 -mlittle-endian -mthumb -mthumb-interwork -fsingle-precision-constant -Wdouble-promotion -mfpu=fpv4-sp-d16 -mfloat-abi=hard -std=gnu99
COMMONFLAGS=-O$(OPTLVL) $(DBG) -Wall
CFLAGS=$(COMMONFLAGS) $(MCUFLAGS) $(INCLUDE) $(CDEFS)
LDLIBS=-L/home/jan/sat/arm-none-eabi/lib/thumb/cortex-m4/float-abi-hard/fpuv4-sp-d16/ -lm
LDFLAGS=$(COMMONFLAGS) -fno-exceptions -ffunction-sections -fdata-sections -nostartfiles -Wl,--gc-sections,-T$(LINKER_SCRIPT) -v

CC=$(TOOLCHAIN_PATH)/$(TOOLCHAIN_PREFIX)-gcc
LD=$(TOOLCHAIN_PATH)/$(TOOLCHAIN_PREFIX)-gcc
OBJCOPY=$(TOOLCHAIN_PATH)/$(TOOLCHAIN_PREFIX)-objcopy
AS=$(TOOLCHAIN_PATH)/$(TOOLCHAIN_PREFIX)-as
AR=$(TOOLCHAIN_PATH)/$(TOOLCHAIN_PREFIX)-ar
GDB=$(TOOLCHAIN_PATH)/$(TOOLCHAIN_PREFIX)-gdb

OBJ = $(SRC:%.c=$(BUILD_DIR)/%.o)

$(BUILD_DIR)/%.o: %.c
	@echo [CC] $<
	@$(CC) $(CFLAGS) $< -c -o $@

all: $(OBJ)
	@echo [AS] $(STARTUP)/$(ASRC)
	@$(AS) -o $(ASRC:%.s=$(BUILD_DIR)/%.o) $(STARTUP)/$(ASRC)
	@echo [LD] $(BIN_DIR)/$(TARGET).elf
	@$(CC) -o $(BIN_DIR)/$(TARGET).elf $(LDFLAGS) $(OBJ) $(ASRC:%.s=$(BUILD_DIR)/%.o) $(LDLIBS)
	@echo [OBJCOPY] $(BIN_DIR)/$(TARGET).hex
	@$(OBJCOPY) -O ihex $(BIN_DIR)/$(TARGET).elf $(BIN_DIR)/$(TARGET).hex
	@echo [OBJCOPY] $(BIN_DIR)/$(TARGET).bin
	@$(OBJCOPY) -O binary $(BIN_DIR)/$(TARGET).elf $(BIN_DIR)/$(TARGET).bin
	@$(TOOLCHAIN_PREFIX)-size $(BIN_DIR)/$(TARGET).elf

.PHONY: clean

clean:
	@echo [RM] OBJ
	@rm -f $(OBJ)
	@rm -f $(ASRC:%.s=$(BUILD_DIR)/%.o)
	@echo [RM] BIN
	@rm -f $(BIN_DIR)/$(TARGET).elf
	@rm -f $(BIN_DIR)/$(TARGET).hex
	@rm -f $(BIN_DIR)/$(TARGET).bin

flash:
	@st-flash write $(BIN_DIR)/$(TARGET).bin 0x8000000
