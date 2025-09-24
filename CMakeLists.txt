cmake_minimum_required(VERSION 3.10)

# Basic project setup
set(CMAKE_SYSTEM_NAME    Generic)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
project(choochoo LANGUAGES C ASM)

# Build type configuration
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

# Compiler flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror -Wno-unused-const-variable -fno-builtin")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS} -O3 -DRELEASE_BUILD")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS} -g -DDEBUG_BUILD")

# Log level configuration
set(KLOG_LEVEL "AUTO" CACHE STRING "Klog level (AUTO, NONE, PANIC, ERROR, WARNING, INFO, DEBUG)")
set_property(CACHE KLOG_LEVEL PROPERTY STRINGS AUTO NONE PANIC ERROR WARNING INFO DEBUG)

if(KLOG_LEVEL STREQUAL "AUTO")
# See klog.h
elseif(KLOG_LEVEL STREQUAL "NONE")
    add_definitions(-DKLOG_COMPILE_LEVEL=0)
elseif(KLOG_LEVEL STREQUAL "PANIC")
    add_definitions(-DKLOG_COMPILE_LEVEL=1)
elseif(KLOG_LEVEL STREQUAL "ERROR")
    add_definitions(-DKLOG_COMPILE_LEVEL=2)
elseif(KLOG_LEVEL STREQUAL "WARNING")
    add_definitions(-DKLOG_COMPILE_LEVEL=3)
elseif(KLOG_LEVEL STREQUAL "INFO")
    add_definitions(-DKLOG_COMPILE_LEVEL=4)
elseif(KLOG_LEVEL STREQUAL "DEBUG")
    add_definitions(-DKLOG_COMPILE_LEVEL=5)
else()
    message(FATAL_ERROR "Invalid KLOG_LEVEL: ${KLOG_LEVEL}. Must be one of: AUTO, NONE, PANIC, ERROR, WARNING, INFO, DEBUG")
endif()

# Busy-wait debug configuration
set(ENABLE_BUSY_WAIT_DEBUG OFF CACHE BOOL "Enable busy-wait debug")
if(ENABLE_BUSY_WAIT_DEBUG)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DENABLE_BUSY_WAIT_DEBUG")
endif()

# Architecture configuration
if (NOT ARCH)
    set(ARCH "aarch64")
endif()

if(NOT ARCH STREQUAL "aarch64")
    message(FATAL_ERROR "ARCH must be set to aarch64 with -DARCH=aarch64")
endif()

# Toolchain configuration
if(NOT CMAKE_CROSS_COMPILER_PATH)
    set(CMAKE_CROSS_COMPILER_PATH /Users/tongkun/Playground/arm-gnu-toolchain-14.2.rel1-darwin-arm64-aarch64-none-elf/bin)
endif()

set(CMAKE_C_COMPILER ${CMAKE_CROSS_COMPILER_PATH}/aarch64-none-elf-gcc)
set(CMAKE_ASM_COMPILER ${CMAKE_CROSS_COMPILER_PATH}/aarch64-none-elf-gcc)
set(OBJCOPY ${CMAKE_CROSS_COMPILER_PATH}/aarch64-none-elf-objcopy)
set(NM ${CMAKE_CROSS_COMPILER_PATH}/aarch64-none-elf-nm)

# Architecture-specific flags
if(MMU)
    add_definitions(-DMMU)
else()
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mstrict-align -mgeneral-regs-only")
endif()
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv8-a")

# Create build directories
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/include)
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/src)

# User application library sources
set(ULIB_SOURCES
    src/ulibs/syscall.c
    src/ulibs/printf.c
    src/ulibs/name.c
    src/ulibs/string.c
    src/ulibs/random.c
    src/ulibs/clock.c
    src/ulibs/io.c
)

set(MARKLIN_SOURCES
    src/uapps/marklin/controller/marklin.c
    src/uapps/marklin/topology/topology.c
    src/uapps/marklin/topology/track_data.c
    src/uapps/marklin/msgqueue/msgqueue.c
    # src/uapps/marklin/train/train.c
    # src/uapps/marklin/train/model.c
    src/uapps/marklin/train/model_defaults.c
    # src/uapps/marklin/train/calibration.c
    # src/uapps/marklin/train/offline.c
    # src/uapps/marklin/train/offline_velocity.c
    # src/uapps/marklin/train/offline_acceleration.c
    # src/uapps/marklin/train/offline_stop_distance.c
    src/uapps/marklin/train2/train.c
    src/uapps/marklin/train2/model.c
    src/uapps/marklin/command/command.c
    src/uapps/marklin/conductor/conductor.c
    src/uapps/marklin/conductor/sensor.c
    src/uapps/marklin/conductor/switch.c
    src/uapps/marklin/conductor/path.c
    src/uapps/marklin/conductor/block.c
    src/uapps/marklin/conductor/block_definitions.c
    src/uapps/marklin/tui/tui.c
)

# User application sources
set(UAPP_SOURCES
    src/uapps/init.c
    src/uapps/name_server.c
    src/uapps/clock_server.c
    src/uapps/rps/server.c
    src/uapps/rps/client.c
    src/uapps/idle_task.c
    src/uapps/io_server.c
    src/uapps/io_test.c
    ${MARKLIN_SOURCES}
    ${ULIB_SOURCES}
)

# Create user applications
add_executable(uapp ${UAPP_SOURCES})
target_include_directories(uapp PRIVATE
    src/uapps/include
    include/uapi
    ${CMAKE_BINARY_DIR}/include
)
set_target_properties(uapp PROPERTIES
    LINK_FLAGS "-nostdlib -nostartfiles -T ${CMAKE_SOURCE_DIR}/src/uapps/uapp.ld"
)

# Common source files
set(KERNEL_SOURCES
    src/arch/boot.S
    src/arch/rpi.c
    src/arch/mmu.S
    src/arch/exception.c
    src/timer/time.c
    src/timer/timer.c
    src/init.c
    src/printf.c
    src/exception.c
    src/klog.c
    src/panic.c
    src/symbol.c
    src/string.c
    src/uart.c
    src/boot_test.c
    src/task.c
    src/sched.c
    src/entry.S
    src/entry.c
    src/syscall.c
    src/context.c
    src/interrupt.c
    src/arch/gic.c
    src/event.c
    src/idle.c
)

# Copy user application to kernel image
add_custom_command(
    OUTPUT ${CMAKE_BINARY_DIR}/user_app.o
    COMMAND ${OBJCOPY} -O binary ${CMAKE_BINARY_DIR}/uapp ${CMAKE_BINARY_DIR}/uapp.bin
    COMMAND ${OBJCOPY} -I binary -O elf64-littleaarch64 -B aarch64 --rename-section .data=.user_task ${CMAKE_BINARY_DIR}/uapp.bin ${CMAKE_BINARY_DIR}/user_app.o
    DEPENDS uapp
    COMMENT "Creating user application object file"
)

# Stage 1: Initial kernel build without symbols
add_executable(stage1 ${KERNEL_SOURCES})
target_include_directories(stage1 PRIVATE
    include
    ${CMAKE_BINARY_DIR}/include
)
target_compile_definitions(stage1 PRIVATE __KERNEL__)
set_target_properties(stage1 PROPERTIES
    LINK_FLAGS "-nostdlib -nostartfiles -T ${CMAKE_SOURCE_DIR}/src/arch/kernel.ld"
    ENABLE_EXPORTS ON
)

# Symbol table generation
add_custom_command(
    OUTPUT ${CMAKE_BINARY_DIR}/src/symbols.c
    COMMAND ${CMAKE_COMMAND}
        -DNM_TOOL=${NM}
        -DELF_FILE=${CMAKE_BINARY_DIR}/stage1
        -DUAPP_ELF_FILE=${CMAKE_BINARY_DIR}/uapp
        -DSYMBOLS_FILE=${CMAKE_BINARY_DIR}/src/symbols.c
        -P ${CMAKE_SOURCE_DIR}/cmake/GenerateSymbols.cmake
    DEPENDS stage1 uapp
    COMMENT "Generating symbol table with kernel and user app symbols"
)

# Final kernel build with symbols and user application
add_executable(kernel.elf ${KERNEL_SOURCES} ${CMAKE_BINARY_DIR}/src/symbols.c ${CMAKE_BINARY_DIR}/user_app.o)
target_include_directories(kernel.elf PRIVATE
    include
    ${CMAKE_BINARY_DIR}/include
)
target_compile_definitions(kernel.elf PRIVATE __KERNEL__)
set_target_properties(kernel.elf PROPERTIES
    LINK_FLAGS "-nostdlib -nostartfiles -T ${CMAKE_SOURCE_DIR}/src/arch/kernel.ld"
)

# Create final kernel.img with embedded user application
add_custom_command(
    OUTPUT kernel.img
    COMMAND ${OBJCOPY} -O binary
    ${CMAKE_BINARY_DIR}/kernel.elf ${CMAKE_BINARY_DIR}/kernel.img
    DEPENDS kernel.elf
    COMMENT "Creating kernel.img from kernel.elf"
)

add_custom_target(kernel_img ALL DEPENDS kernel.img)
