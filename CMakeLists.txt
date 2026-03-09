cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME eskf_stm32)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C CXX ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# -----------------------------------------------------------------------------
# [ESKF] User Configuration
# -----------------------------------------------------------------------------

# 1. 宏定义配置 (CMSIS-DSP)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    ARM_MATH_CM4        # 目标内核: Cortex-M4
)

# 2. 头文件路径 (确保能找到 arm_math.h)
# CubeMX 通常生成在 Drivers/CMSIS/DSP/Include 或类似的路径
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Drivers/CMSIS/DSP/Include
    Drivers/CMSIS/Include
    UserCode/algorithm/eskf  # 包含算法头文件
    UserCode/app             # 包含应用头文件
    UserCode/ILoc            # 包含定位系统头文件
)

# 3. 链接 CMSIS-DSP 静态库

# 1. 矩阵运算相关
file(GLOB DSP_MAT_SRC "Drivers/CMSIS/DSP/Source/MatrixFunctions/*.c")
# 排除 MatrixFunctions.c (如果存在)
list(FILTER DSP_MAT_SRC EXCLUDE REGEX "MatrixFunctions.c$")

# 2. 基础数学运算 (可选)
file(GLOB DSP_BASIC_SRC "Drivers/CMSIS/DSP/Source/BasicMathFunctions/*.c")
# 排除 BasicMathFunctions.c
list(FILTER DSP_BASIC_SRC EXCLUDE REGEX "BasicMathFunctions.c$")

# 3. 统计功能 (可选)
file(GLOB DSP_STAT_SRC "Drivers/CMSIS/DSP/Source/StatisticsFunctions/*.c")
# 排除 StatisticsFunctions.c
list(FILTER DSP_STAT_SRC EXCLUDE REGEX "StatisticsFunctions.c$")
# CommonTables 通常也被其他文件引用，如果重复定义也需排除，但在旧版本源码中它是独立的

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${DSP_MAT_SRC}
    ${DSP_BASIC_SRC}
    ${DSP_STAT_SRC}
    Drivers/CMSIS/DSP/Source/CommonTables/CommonTables.c # 必须包含查表数据
    Drivers/CMSIS/DSP/Source/QuaternionMathFunctions/QuaternionMathFunctions.c # 必须包含四元数运算
    UserCode/algorithm/eskf/eskf_core.c
    UserCode/algorithm/eskf/eskf_mat.c
    UserCode/algorithm/eskf/time_sync.c
    UserCode/algorithm/eskf/eskf.c
    UserCode/app/app.cpp
    UserCode/ILoc/ILoc.cpp
    UserCode/ILoc/LocESKF.cpp
)

# -----------------------------------------------------------------------------

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
