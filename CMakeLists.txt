cmake_minimum_required(VERSION 3.16)

# 使用 stm32-cmake 工具链（必须在 project 之前）
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/../../3dparty/stm32-cmake/cmake/stm32_gcc.cmake")

project(stm32g474_foc C CXX ASM)

# ============================================
# 构建类型
# ============================================
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
    message(STATUS "未指定构建类型，默认使用: ${CMAKE_BUILD_TYPE}")
endif()

# ============================================
# MCU / 语言配置
# ============================================
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(STM32_FAMILY G4)

# 链接脚本与启动文件
file(GLOB LD_SCRIPT "**_FLASH.ld")
file(GLOB STARTUP_FILE "startup_stm32**.s")

# 预定义宏
add_compile_definitions(
    USE_HAL_DRIVER
    STM32G474xx
)

# 头文件目录（兼容 CubeMX + 迁移进来的 motor_control / driver/stm32）
include_directories(
    Core/Inc
    motor_control/include
    Drivers/stm32/include
    Drivers/STM32G4xx_HAL_Driver/Inc
    Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32G4xx/Include
    Drivers/CMSIS/Include
    Drivers/CMSIS/DSP/Include
    USB_Device/App
    USB_Device/Target
    Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
)

# ============================================
# 源文件
# ============================================
# Core 全量收集，剔除 CubeMX 的 main.c（由 motor_control/src/main.cpp 替代）
file(GLOB_RECURSE CORE_SOURCES CONFIGURE_DEPENDS Core/Src/*.c Core/Src/*.cpp)
list(FILTER CORE_SOURCES EXCLUDE REGEX Core/Src/main.c)  # main.c 由 motor_control/src/main.cpp 替代

# 迁移模块
file(GLOB_RECURSE MOTOR_CONTROL_SOURCES CONFIGURE_DEPENDS motor_control/src/*.c motor_control/src/*.cpp)
file(GLOB_RECURSE DRIVER_STM32_SOURCES CONFIGURE_DEPENDS Drivers/stm32/src/*.c Drivers/stm32/src/*.cpp)

# USB Device
set(USB_DEVICE_SOURCES
    USB_Device/App/usb_device.c
    USB_Device/App/usbd_desc.c
    USB_Device/App/usbd_cdc_if.c
    USB_Device/Target/usbd_conf.c
)

set(USB_DEVICE_LIBRARY_SOURCES
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
    Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
)

# HAL
set(HAL_SOURCES
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ex.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ramfunc.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_exti.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma_ex.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_spi.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_spi_ex.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim_ex.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart_ex.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pcd.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pcd_ex.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_usb.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc_ex.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_adc.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_fdcan.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_cordic.c
)

set(SOURCES
    ${CORE_SOURCES}
    ${MOTOR_CONTROL_SOURCES}
    ${DRIVER_STM32_SOURCES}
    ${USB_DEVICE_SOURCES}
    ${USB_DEVICE_LIBRARY_SOURCES}
    ${HAL_SOURCES}
    Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_sin_f32.c
    Drivers/CMSIS/DSP/Source/CommonTables/arm_common_tables.c
    ${STARTUP_FILE}
)

add_executable(${PROJECT_NAME}.elf ${SOURCES})

# ============================================
# 编译选项
# ============================================
target_compile_options(${PROJECT_NAME}.elf PRIVATE
    -mthumb
    -mcpu=cortex-m4
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -Wall
    -fdata-sections
    -ffunction-sections
    $<$<CONFIG:Debug>:-g -gdwarf-2 -Og>
    $<$<CONFIG:Release>:-Os -flto>
    $<$<CONFIG:RelWithDebInfo>:-Os -g1>
)

# ============================================
# 链接选项
# ============================================
target_link_options(${PROJECT_NAME}.elf PRIVATE
    -mthumb
    -mcpu=cortex-m4
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -specs=nano.specs
    -u _printf_float
    -Wl,-gc-sections
    -Wl,-Map=${CMAKE_BINARY_DIR}/${PROJECT_NAME}.map,--cref
    $<$<CONFIG:Release>:-flto>
)

# 链接脚本（stm32-cmake）
stm32_add_linker_script(${PROJECT_NAME}.elf PRIVATE ${LD_SCRIPT})

# 链接库
target_link_libraries(${PROJECT_NAME}.elf
    -lc
    -lm
    -lnosys
    ${MICROROS_LIBRARIES}
)

# 产物导出
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary -S $<TARGET_FILE:${PROJECT_NAME}.elf> ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.bin
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${PROJECT_NAME}.elf> ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.elf
    COMMENT "Generating hex, bin and elf files"
)
