# GCC Makefile build path for CtrBoard-H7-AIBOT-DPC.
# Primary IDE remains Keil5; this file provides an additional command-line build.

PROJECT := CtrBoard-H7-AIBOT-DPC

ifeq ($(V),1)
Q :=
ECHO := @rem
else
Q := @
ECHO := @echo
endif

TOOLCHAIN := D:/Msys2/mingw64/bin
CC       := $(TOOLCHAIN)/arm-none-eabi-gcc.exe
AS       := $(TOOLCHAIN)/arm-none-eabi-gcc.exe
OBJCOPY  := $(TOOLCHAIN)/arm-none-eabi-objcopy.exe
SIZE     := $(TOOLCHAIN)/arm-none-eabi-size.exe

BUILD_DIR := build
OBJ_DIR   := $(BUILD_DIR)/obj

ELF_FILE := $(BUILD_DIR)/$(PROJECT).elf
MAP_FILE := $(BUILD_DIR)/$(PROJECT).map
HEX_FILE := $(BUILD_DIR)/$(PROJECT).hex
BIN_FILE := $(BUILD_DIR)/$(PROJECT).bin

LINKER_SCRIPT := GCC/STM32H723VGTx_FLASH.ld

CPU_FLAGS := -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard

DEFINES := \
  -DUSE_HAL_DRIVER \
  -DSTM32H723xx \
  -DUSE_PWR_LDO_SUPPLY \
  -DDISABLEFLOAT16

INCLUDES := \
  -ICore/Inc \
  -IApp/Inc \
  -IBSP/Inc \
  -IMiddleware/Filter/Inc \
  -IMiddleware/Protocol/Inc \
  -IMiddleware/Motor/Inc \
  -IDrivers/STM32H7xx_HAL_Driver/Inc \
  -IDrivers/STM32H7xx_HAL_Driver/Inc/Legacy \
  -IDrivers/CMSIS/Device/ST/STM32H7xx/Include \
  -IDrivers/CMSIS/Include \
  -IDSP/Include \
  -IDSP/PrivateInclude

C_SOURCES := \
  Core/Src/main.c \
  Core/Src/gpio.c \
  Core/Src/dma.c \
  Core/Src/spi.c \
  Core/Src/tim.c \
  Core/Src/usart.c \
  Core/Src/stm32h7xx_it.c \
  Core/Src/stm32h7xx_hal_msp.c \
  App/Src/app_led_indicator.c \
  App/Src/app_ld2rs_task.c \
  App/Src/app_encoder_speed.c \
  App/Src/app_chassis_motor_ctrl.c \
  BSP/Src/bsp_rs485.c \
  BSP/Src/bsp_dwt.c \
  BSP/Src/bsp_vofa.c \
  BSP/Src/bsp_rc.c \
  BSP/Src/bsp_ws4810.c \
  Middleware/Filter/Src/low_pass_filter.c \
  Middleware/Protocol/Src/modbus_rtu.c \
  Middleware/Motor/Src/ld2_motor.c \
  DSP/Source/BasicMathFunctions/BasicMathFunctions.c \
  DSP/Source/BayesFunctions/BayesFunctions.c \
  DSP/Source/CommonTables/CommonTables.c \
  DSP/Source/ComplexMathFunctions/ComplexMathFunctions.c \
  DSP/Source/ControllerFunctions/ControllerFunctions.c \
  DSP/Source/DistanceFunctions/DistanceFunctions.c \
  DSP/Source/FastMathFunctions/FastMathFunctions.c \
  DSP/Source/FilteringFunctions/FilteringFunctions.c \
  DSP/Source/InterpolationFunctions/InterpolationFunctions.c \
  DSP/Source/MatrixFunctions/MatrixFunctions.c \
  DSP/Source/QuaternionMathFunctions/QuaternionMathFunctions.c \
  DSP/Source/StatisticsFunctions/StatisticsFunctions.c \
  DSP/Source/SupportFunctions/SupportFunctions.c \
  DSP/Source/SVMFunctions/SVMFunctions.c \
  DSP/Source/TransformFunctions/TransformFunctions.c \
  DSP/Source/WindowFunctions/WindowFunctions.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hsem.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_mdma.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c_ex.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_exti.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi_ex.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart_ex.c \
  Core/Src/system_stm32h7xx.c

ASM_SOURCES := \
  GCC/startup_stm32h723xx_gcc.s

OBJECTS := \
  $(patsubst %.c,$(OBJ_DIR)/%.o,$(C_SOURCES)) \
  $(patsubst %.s,$(OBJ_DIR)/%.o,$(ASM_SOURCES))

DEPENDS := $(OBJECTS:.o=.d)

COMMON_FLAGS := $(CPU_FLAGS) $(DEFINES) $(INCLUDES)

CFLAGS := \
  $(COMMON_FLAGS) \
  -std=gnu11 \
  -Og \
  -g3 \
  -Wall \
  -ffunction-sections \
  -fdata-sections \
  -MMD \
  -MP

ASFLAGS := \
  $(COMMON_FLAGS) \
  -x assembler-with-cpp \
  -g3 \
  -MMD \
  -MP

LDFLAGS := \
  $(CPU_FLAGS) \
  -T$(LINKER_SCRIPT) \
  --specs=nano.specs \
  --specs=nosys.specs \
  -Wl,-Map=$(MAP_FILE),--cref \
  -Wl,--gc-sections \
  -Wl,--print-memory-usage

.PHONY: all clean size

all: $(ELF_FILE) $(HEX_FILE) $(BIN_FILE) size

$(ELF_FILE): $(OBJECTS) $(LINKER_SCRIPT)
	@if not exist "$(subst /,\,$(@D))" mkdir "$(subst /,\,$(@D))"
	$(ECHO) [LD] $@
	$(Q)$(CC) $(OBJECTS) $(LDFLAGS) -o $@

$(HEX_FILE): $(ELF_FILE)
	$(ECHO) [HEX] $@
	$(Q)$(OBJCOPY) -O ihex $< $@

$(BIN_FILE): $(ELF_FILE)
	$(ECHO) [BIN] $@
	$(Q)$(OBJCOPY) -O binary -S $< $@

$(OBJ_DIR)/%.o: %.c
	@if not exist "$(subst /,\,$(@D))" mkdir "$(subst /,\,$(@D))"
	$(ECHO) [CC] $<
	$(Q)$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_DIR)/%.o: %.s
	@if not exist "$(subst /,\,$(@D))" mkdir "$(subst /,\,$(@D))"
	$(ECHO) [AS] $<
	$(Q)$(AS) $(ASFLAGS) -c $< -o $@

size: $(ELF_FILE)
	$(ECHO) [SIZE] $<
	$(Q)$(SIZE) $<

clean:
	$(ECHO) [CLEAN] $(BUILD_DIR)
	$(Q)if exist "$(subst /,\,$(BUILD_DIR))" rmdir /s /q "$(subst /,\,$(BUILD_DIR))"

-include $(DEPENDS)


