cmake_minimum_required(VERSION 3.10)
project(rm_auto_aim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## 强制启用警告
add_definitions(-Wall -Werror)

add_compile_options(-g -O0)

## 导出编译命令以便使用 clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# libxr
set(XROBOT_MODULES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Modules)
add_subdirectory(libxr)

add_executable(${PROJECT_NAME} User/main.cpp)

target_link_libraries(
    ${PROJECT_NAME}
    PUBLIC xr
)

target_include_directories(
    ${PROJECT_NAME}
    PUBLIC $<TARGET_PROPERTY:xr,INTERFACE_INCLUDE_DIRECTORIES>
)

target_include_directories(
    ${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/User
)
