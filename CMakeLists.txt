cmake_minimum_required(VERSION 3.20)

project(polsl_iwsk
    VERSION 0.1.0
    DESCRIPTION "RS-232 and MODBUS serial emulation project"
    LANGUAGES C
)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_EXPORT_COMPILE_COMMANDS)
    if(UNIX)
        add_custom_target(editor_compile_commands ALL
            COMMAND ${CMAKE_COMMAND} -E rm -f
                ${CMAKE_SOURCE_DIR}/compile_commands.json
            COMMAND ${CMAKE_COMMAND} -E create_symlink
                ${CMAKE_BINARY_DIR}/compile_commands.json
                ${CMAKE_SOURCE_DIR}/compile_commands.json
        )
    else()
        add_custom_target(editor_compile_commands ALL
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${CMAKE_BINARY_DIR}/compile_commands.json
                ${CMAKE_SOURCE_DIR}/compile_commands.json
        )
    endif()
endif()

add_library(serial_hal STATIC
    src/hal/serial_hal_linux.c
)

target_include_directories(serial_hal
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if(MSVC)
    target_compile_options(serial_hal PRIVATE /W4)
else()
    target_compile_options(serial_hal PRIVATE -Wall -Wextra -Wpedantic)
endif()
