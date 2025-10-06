cmake_minimum_required(VERSION 3.8)
project(hik_camera)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc highgui)

add_library(${PROJECT_NAME} STATIC ${CMAKE_CURRENT_SOURCE_DIR}/src/hik_camera_node.cpp)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(${PROJECT_NAME} PUBLIC hikSDK/lib/amd64)
  install(
    DIRECTORY hikSDK/lib/amd64/
    DESTINATION lib
  )
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(${PROJECT_NAME} PUBLIC hikSDK/lib/arm64)
  install(
    DIRECTORY hikSDK/lib/arm64/
    DESTINATION lib
  )
else()
  message(FATAL_ERROR "Unsupport host system architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

target_link_libraries(${PROJECT_NAME}

)

target_link_libraries(
    hik_camera
    xr
    ${OpenCV_LIBS}
    # HikSDK
    FormatConversion
    MediaProcess
    MvCameraControl
    MVRender
    MvUsb3vTL
)

target_include_directories(hik_camera PUBLIC
  PUBLIC $<TARGET_PROPERTY:xr,INTERFACE_INCLUDE_DIRECTORIES>
  PUBLIC $<TARGET_PROPERTY:camera_base,INTERFACE_INCLUDE_DIRECTORIES>
  PUBLIC ${OpenCV_INCLUDE_DIRS}
  PUBLIC hikSDK/include src/
)
