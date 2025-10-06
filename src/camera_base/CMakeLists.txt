cmake_minimum_required(VERSION 3.8)
project(camera_base LANGUAGES CXX)

# 头文件库（不会被编译）
add_library(camera_base INTERFACE)
add_library(camera_base::camera_base ALIAS camera_base)

# 头文件搜索路径（传给使用者）
target_include_directories(camera_base
  INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# 传递依赖（如果 xr 是一个 CMake target，链接它即可；
# 它的 INTERFACE_INCLUDE_DIRECTORIES 会自动跟着传递）
target_link_libraries(camera_base
  INTERFACE
    xr
)

# 要求使用者至少用到这个 C++ 标准（按需调整）
target_compile_features(camera_base INTERFACE cxx_std_17)
