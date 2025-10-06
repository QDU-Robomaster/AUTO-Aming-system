project(armor_detector)

## Use C++14
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)


#######################
## Find dependencies ##
#######################
find_package(OpenCV REQUIRED)

###########
## Build ##
###########

# ament_auto_add_library(${PROJECT_NAME} STATIC
#   DIRECTORY src
# )
add_library(${PROJECT_NAME} STATIC 
  src/detector.cpp
  src/detector_node.cpp
  src/number_classifier.cpp
  src/pnp_solver.cpp
)

target_include_directories(${PROJECT_NAME} 
  PUBLIC ${OpenCV_INCLUDE_DIRS} 
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/
  PUBLIC $<TARGET_PROPERTY:xr,INTERFACE_INCLUDE_DIRECTORIES>
  PUBLIC $<TARGET_PROPERTY:camera_base,INTERFACE_INCLUDE_DIRECTORIES>
)

target_link_libraries(${PROJECT_NAME} 
  PUBLIC ${OpenCV_LIBS}
  PUBLIC xr
)

add_compile_definitions(MODEL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/model")
