cmake_minimum_required(VERSION 3.20)
project(SBUS LANGUAGES C)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(TOOLCHAIN_PREFIX arm-none-eabi)
set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}-gcc)
set(CMAKE_ASM_COMPILER ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}-g++)
set(CMAKE_AR ${TOOLCHAIN_PREFIX}-ar)
set(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}-objcopy)
set(CMAKE_OBJDUMP ${TOOLCHAIN_PREFIX}-objdump)
set(SIZE ${TOOLCHAIN_PREFIX}-size)

set(CPU_FLAGS "-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16")
set(CMAKE_C_FLAGS "${CPU_FLAGS} -Wall -O0 -g3 -fdata-sections -ffunction-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CPU_FLAGS} -specs=nosys.specs -specs=nano.specs -Wl,--gc-sections -T${CMAKE_SOURCE_DIR}/STM32F427IIHx_FLASH.ld")

file(GLOB_RECURSE SOURCES
    "Core/Src/*.c"
    "Drivers/STM32F4xx_HAL_Driver/Src/*.c"
    "Drivers/CMSIS/*.c"
)

file(GLOB_RECURSE HEADERS
    "Core/Inc/*.h"
    "Drivers/STM32F4xx_HAL_Driver/Inc/*.h"
    "Drivers/CMSIS/*.h"
)

include_directories(
    Core/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
    Drivers/CMSIS/Include
)

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${HEADERS})

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
    COMMAND ${SIZE} ${PROJECT_NAME}.elf
    COMMENT "Generating binary and hex files"
)
