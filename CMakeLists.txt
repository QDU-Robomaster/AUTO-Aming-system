cmake_minimum_required(VERSION 3.10)
project(rm_auto_aim)

## 设置 C++14 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## 强制启用警告
add_definitions(-Wall -Werror)

## 导出编译命令以便使用 clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# libxr
add_subdirectory(libxr)

add_executable(${PROJECT_NAME}_node src/main.cpp)

target_link_libraries(
    ${PROJECT_NAME}_node
    PUBLIC xr
)

target_include_directories(
    ${PROJECT_NAME}_node
    PUBLIC $<TARGET_PROPERTY:xr,INTERFACE_INCLUDE_DIRECTORIES>
)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/hik_camera)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/uvc_camera)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/camera_base)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/rm_auto_aim/armor_detector)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/rm_auto_aim/armor_executor)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/rm_auto_aim/armor_tracker)
# add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/rm_gimbal_description)
# add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/rm_auto_aim/auto_aim_interfaces)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/rm_serial_driver)

target_link_libraries(${PROJECT_NAME}_node
  PUBLIC hik_camera
  PUBLIC uvc_camera
  PUBLIC armor_detector
  PUBLIC armor_executor
  PUBLIC armor_tracker
  PUBLIC rm_serial_driver
)

# 安装文件
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/ launch/
  DESTINATION share/${PROJECT_NAME}
)
