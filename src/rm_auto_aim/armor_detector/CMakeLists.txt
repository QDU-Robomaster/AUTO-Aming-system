cmake_minimum_required(VERSION 3.10)
project(armor_detector)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} STATIC
  DIRECTORY src
)

target_include_directories(${PROJECT_NAME} 
  PUBLIC ${OpenCV_INCLUDE_DIRS} 
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/armor_detector/
)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rm_auto_aim::ArmorDetectorNode
  EXECUTABLE armor_detector_node
)

add_compile_definitions(MODEL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/model")

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest)
  ament_add_gtest(test_node_startup test/test_node_startup.cpp)
  target_link_libraries(test_node_startup ${PROJECT_NAME})

  ament_add_gtest(test_number_cls test/test_number_cls.cpp)
  target_link_libraries(test_number_cls ${PROJECT_NAME})

endif()

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
  model
)

target_include_directories(armor_detector PUBLIC
  ${CMAKE_BINARY_DIR}/src/rm_auto_aim/auto_aim_interfaces/rosidl_generator_cpp
)

file(
  GLOB IF_SOURCES "${CMAKE_BINARY_DIR}/src/rm_auto_aim/auto_aim_interfaces/rosidl_typesupport_cpp/auto_aim_interfaces/msg/*.cpp")

target_sources(
  armor_detector
  PRIVATE ${IF_SOURCES}
)

target_link_libraries(
    armor_detector
    xr
)

target_include_directories(armor_detector
  PUBLIC $<TARGET_PROPERTY:xr,INTERFACE_INCLUDE_DIRECTORIES>
)
