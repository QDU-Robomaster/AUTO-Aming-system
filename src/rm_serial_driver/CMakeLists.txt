# src/rm_serial_driver/CMakeLists.txt

# 添加源代码文件
add_library(rm_serial_driver STATIC
  ${CMAKE_CURRENT_LIST_DIR}/src/rm_serial_driver.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/crc.cpp
)

# 链接必要的库
target_link_libraries(rm_serial_driver
  PUBLIC ${rclcpp_LIBRARIES}
  PUBLIC ${rcl_interfaces_LIBRARIES}
  PUBLIC ${class_loader_LIBRARIES}
  PUBLIC ${serial_driver_LIBRARIES}
  PUBLIC ${sensor_msgs_LIBRARIES}
  PUBLIC ${visualization_msgs_LIBRARIES}
  PUBLIC ${auto_aim_interfaces_LIBRARIES}
  PUBLIC ${std_srvs_LIBRARIES}
)

# 添加包含目录
target_include_directories(rm_serial_driver
  PUBLIC ${rclcpp_INCLUDE_DIRS}
  PUBLIC ${rcl_interfaces_INCLUDE_DIRS}
  PUBLIC ${class_loader_INCLUDE_DIRS}
  PUBLIC ${serial_driver_INCLUDE_DIRS}
  PUBLIC ${sensor_msgs_INCLUDE_DIRS}
  PUBLIC ${visualization_msgs_INCLUDE_DIRS}
  PUBLIC ${auto_aim_interfaces_INCLUDE_DIRS}
  PUBLIC ${std_srvs_INCLUDE_DIRS}
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src
)

add_dependencies(rm_serial_driver
  auto_aim_interfaces
)

file(
  GLOB IF_SOURCES "${CMAKE_BINARY_DIR}/src/rm_auto_aim/auto_aim_interfaces/rosidl_typesupport_cpp/auto_aim_interfaces/msg/*.cpp")

target_sources(
  rm_serial_driver
  PRIVATE ${IF_SOURCES}
)

target_include_directories(rm_serial_driver PUBLIC
  ${CMAKE_BINARY_DIR}/src/rm_auto_aim/auto_aim_interfaces/rosidl_generator_cpp
)
