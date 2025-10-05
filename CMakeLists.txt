cmake_minimum_required(VERSION 3.10)
project(rm_auto_aim)

## 设置 C++14 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## 强制启用警告
add_definitions(-Wall -Werror)

## 导出编译命令以便使用 clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 查找 ROS 2 依赖
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(serial_driver REQUIRED)
find_package(class_loader REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)

# libxr
add_subdirectory(libxr)

add_executable(${PROJECT_NAME}_node src/main.cpp)

add_dependencies(${PROJECT_NAME}_node xr)

target_link_libraries(
    ${PROJECT_NAME}_node
    PUBLIC xr
)

target_include_directories(
    ${PROJECT_NAME}_node
    PUBLIC $<TARGET_PROPERTY:xr,INTERFACE_INCLUDE_DIRECTORIES>
)

# ros 依赖

target_link_libraries(${PROJECT_NAME}_node
  PUBLIC ${rclcpp_LIBRARIES}
  PUBLIC ${rcl_interfaces_LIBRARIES}
  PUBLIC ${class_loader_LIBRARIES}
  PUBLIC ${serial_driver_LIBRARIES}
  PUBLIC ${sensor_msgs_LIBRARIES}
  PUBLIC ${visualization_msgs_LIBRARIES}
  PUBLIC ${auto_aim_interfaces_LIBRARIES}
  PUBLIC ${std_srvs_LIBRARIES}
  PUBLIC ${rosidl_typesupport_cpp_LIBRARIES}
)

add_subdirectory(src/hik_camera)
add_subdirectory(src/rm_auto_aim/armor_detector)
add_subdirectory(src/rm_auto_aim/armor_executor)
add_subdirectory(src/rm_auto_aim/armor_tracker)
add_subdirectory(src/rm_gimbal_description)
add_subdirectory(src/rm_auto_aim/auto_aim_interfaces)
add_subdirectory(src/rm_serial_driver)

add_dependencies(${PROJECT_NAME}_node
  auto_aim_interfaces
)

target_link_libraries(${PROJECT_NAME}_node
  PUBLIC hik_camera
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
