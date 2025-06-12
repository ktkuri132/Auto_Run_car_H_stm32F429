#参考CubeMX生成的CMakeLists.txt文件进行修改和添加注释,若有疑问,请联系作者邮箱3457532883@qq.com
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.22)
# 设置目标处理器架构,芯片型号,和目标板简称
set(CMAKE_SYSTEM_PROCESSOR cortex-m4)
set(Target_Board STM32F4xx)
set(Board_Name stm32f4)
# 指定交叉编译工具链arm-none-eabi-gcc,添加到环境变量中,或者在此处直接指定绝对路径
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER  arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# 项目名称
project(AutoRunCarH C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

# 添加关于硬件浮点的支持的相关编译选项
if("${Target_Board}" STREQUAL "STM32F1xx")
    message(STATUS "${Target_Board}")
    add_compile_options(-mfloat-abi=soft)
    add_link_options(-mfloat-abi=soft)
elseif ("${Target_Board}" STREQUAL "STM32F4xx")
    message(STATUS "${Target_Board}")
    add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
    add_link_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
elseif("${Target_Board}" STREQUAL "STM32H7xx")
    message(STATUS "${Target_Board}")
    add_compile_options(-mfloat-abi=hard -mfpu=fpv5-d16)
    add_link_options(-mfloat-abi=hard -mfpu=fpv5-d16)
elseif("${Target_Board}" STREQUAL "")
    message(FATAL_ERROR "Unknown target board")
endif ()

# 指定芯片型号的相关编译选项
if("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "cortex-m4" OR 
"${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "cortex-m7" OR 
"${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "cortex-m3" OR 
"${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "cortex-m0")
    add_compile_options(-mcpu=${CMAKE_SYSTEM_PROCESSOR} -mthumb)
    add_link_options(-mcpu=${CMAKE_SYSTEM_PROCESSOR} -mthumb -mthumb-interwork)
elseif("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "cortex-a7" OR
"${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "cortex-a8" OR )
    add_compile_options(-mcpu=${CMAKE_SYSTEM_PROCESSOR})
    add_link_options(-mcpu=${CMAKE_SYSTEM_PROCESSOR})
endif()
add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)
add_link_options(-specs=nosys.specs -lc -lm)

# 此处忽略C++17中的操作绝对地址(寄存器)的警告
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-register")

# 进行汇编文件的预编译
add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>)
add_compile_options(-w)
# 提供编译选项,根据不同的编译类型进行不同的优化,常见的编译类型有Debug,Release,RelWithDebInfo,MinSizeRel
if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Maximum optimization for speed, debug info included")
    add_compile_options(-Ofast -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
else ()
    message(STATUS "Minimal optimization, debug info included")
    add_compile_options(-Og -g)
endif ()


# 添加链接脚本
file(GLOB_RECURSE LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/*.ld)

# 添加链接选项,生成map文件
add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options(-T ${LINKER_SCRIPT})


file(
    GLOB_RECURSE SOURCE_FILES
    ${CMAKE_SOURCE_DIR}/src/*.c*
)

# 生成可执行文件
add_executable(${PROJECT_NAME}.elf main.cpp ${SOURCE_FILES} ${LINKER_SCRIPT})


include_directories(${CMAKE_SOURCE_DIR})
target_include_directories(${PROJECT_NAME}.elf PUBLIC
        include
        include/bsp
        include/control
        include/core
        include/hw
        include/mw
        include/rtos
        include/spl
        include/spl/inc
)

add_definitions(
        -DSTM32F429_439xx
        -DUSE_STDPERIPH_DRIVER
)


# 定义其他类型的可执行文件
set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)
# 生成hex和bin文件
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")