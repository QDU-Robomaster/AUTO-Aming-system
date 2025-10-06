# src/rm_serial_driver/CMakeLists.txt

# 添加源代码文件
add_library(rm_serial_driver STATIC
  ${CMAKE_CURRENT_LIST_DIR}/src/rm_serial_driver.cpp
)

# 添加包含目录
target_include_directories(rm_serial_driver
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src
  PUBLIC $<TARGET_PROPERTY:xr,INTERFACE_INCLUDE_DIRECTORIES>
  PUBLIC $<TARGET_PROPERTY:armor_tracker,INTERFACE_INCLUDE_DIRECTORIES>
)

target_link_libraries(
    rm_serial_driver
    PUBLIC xr
)
