cmake_minimum_required(VERSION 3.8)
project(uvc_camera)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc highgui)

add_library(${PROJECT_NAME} STATIC ${CMAKE_CURRENT_SOURCE_DIR}/src/uvc_camera.cpp)

target_link_libraries(${PROJECT_NAME}
  xr
  ${OpenCV_LIBS}
)

target_include_directories(uvc_camera PUBLIC
  PUBLIC $<TARGET_PROPERTY:xr,INTERFACE_INCLUDE_DIRECTORIES>
  PUBLIC $<TARGET_PROPERTY:camera_base,INTERFACE_INCLUDE_DIRECTORIES>
  PUBLIC ${OpenCV_INCLUDE_DIRS}
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src
)
