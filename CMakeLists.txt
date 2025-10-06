cmake_minimum_required(VERSION 3.15)
project(SAI_AudioPlayback C ASM)

set(CMAKE_C_STANDARD 11)

set(CM7_INCLUDES
        ${CMAKE_SOURCE_DIR}/CM7/Inc
        ${CMAKE_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver/Inc
        ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32H7xx/Include
        ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
        ${CMAKE_SOURCE_DIR}/Drivers/BSP/STM32H745I-DISCO
        ${CMAKE_SOURCE_DIR}/Drivers/BSP/Components/Common
)
# ==== CM7 ====
set(CM7_DEFS
        CORE_CM7
        USE_HAL_DRIVER
        STM32H745xx
        USE_PWR_DIRECT_SMPS_SUPPLY
)

set(STM32_LINKER_SCRIPT_CM7 "CM7/stm32h745xx_flash_CM7.ld")  # путь относительно CMAKE_SOURCE_DIR

add_executable(SAI_CM7
        CM7/Src/main.c
        CM7/Src/stm32h7xx_hal_msp.c
        CM7/Src/stm32h7xx_it.c
        CM7/Src/stm32h745i_discovery.c
        CM7/Src/syscalls.c
        CM7/Src/sysmem.c
        drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_sai.c
        drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c
        drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c
        drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c
        CM7/Startup/startup_stm32h745xx_CM7.s
        drivers/BSP/Components/wm8994/wm8994.c
        drivers/BSP/Components/wm8994/wm8994_reg.c
        drivers/BSP/STM32H745I-DISCO/stm32h745i_discovery_bus.c
        drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c
        drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c
        drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c
        drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c
        system_stm32h7xx_dualcore_boot_cm4_cm7.c
        drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c.c
        drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c_ex.c
)
set(PDM_LIB ${CMAKE_SOURCE_DIR}/CM7/Lib/libPDMFilter_CM7_GCC_wc32.a)
# Компиляция только своих флагов, тулчейн уже добавляет остальное
target_compile_definitions(SAI_CM7 PRIVATE ${CM7_DEFS} )
target_compile_options(SAI_CM7 PRIVATE -mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard)
target_include_directories(SAI_CM7 PRIVATE ${CM7_INCLUDES})
target_link_libraries(SAI_CM7
        PRIVATE
        ${PDM_LIB}
        m
)


# Линковка
target_link_options(SAI_CM7 PRIVATE
        -mcpu=cortex-m7
#        -mfpu=fpv5-d16
        -mfloat-abi=hard
        -mthumb
        -T${CMAKE_SOURCE_DIR}/${STM32_LINKER_SCRIPT_CM7}
        -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/SAI_CM7.map
        -Wl,--gc-sections
        -specs=nano.specs
)

# Post-build: .bin
add_custom_command(TARGET SAI_CM7 POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:SAI_CM7> ${CMAKE_CURRENT_BINARY_DIR}/SAI_CM7.bin
)











# ==== CM4 ====

set(CM4_INCLUDES
        ${CMAKE_SOURCE_DIR}/CM4/Inc
        ${CMAKE_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver/Inc
        ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32H7xx/Include
        ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
        ${CMAKE_SOURCE_DIR}/Drivers/BSP/STM32H745I-DISCO
        ${CMAKE_SOURCE_DIR}/Drivers/BSP/Components/Common

)
set(CM4_DEFS
        CORE_CM4
        USE_HAL_DRIVER
        STM32H745xx
        USE_PWR_DIRECT_SMPS_SUPPLY
)

set(STM32_LINKER_SCRIPT_CM4 "CM4/stm32h745xx_flash_CM4.ld")

add_executable(SAI_CM4
        CM4/Src/main.c
        CM4/Src/stm32h7xx_hal_msp.c
        CM4/Src/stm32h7xx_it.c
        CM4/Startup/startup_stm32h745xx_CM4.s
        drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c
        system_stm32h7xx_dualcore_boot_cm4_cm7.c
        drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hsem.c
        drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c
        drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c
        drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c
)

target_compile_definitions(SAI_CM4 PRIVATE ${CM4_DEFS})
target_compile_options(SAI_CM4 PRIVATE -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard)
target_include_directories(SAI_CM4 PRIVATE ${CM4_INCLUDES})

target_link_options(SAI_CM4 PRIVATE
        -mcpu=cortex-m4
        -mfpu=fpv4-sp-d16
        -mfloat-abi=hard
        -mthumb
        -T${CMAKE_SOURCE_DIR}/${STM32_LINKER_SCRIPT_CM4}
        -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/SAI_CM4.map
        -Wl,--gc-sections
)

add_custom_command(TARGET SAI_CM4 POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:SAI_CM4> ${CMAKE_CURRENT_BINARY_DIR}/SAI_CM4.bin
)

