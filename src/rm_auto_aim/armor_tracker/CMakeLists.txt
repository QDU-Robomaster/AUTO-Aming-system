project(armor_tracker)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

###########
## Build ##
###########

# ament_auto_add_library(${PROJECT_NAME} STATIC
#   DIRECTORY src
# )

add_library(armor_tracker STATIC 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/tracker_node.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/tracker.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/extended_kalman_filter.cpp
)

target_link_libraries(
  armor_tracker
  armor_executor 
  armor_detector
  xr
)

target_include_directories(
  armor_tracker
  PUBLIC $<TARGET_PROPERTY:armor_executor,INTERFACE_INCLUDE_DIRECTORIES>
  PUBLIC $<TARGET_PROPERTY:armor_detector,INTERFACE_INCLUDE_DIRECTORIES>
  PUBLIC $<TARGET_PROPERTY:xr,INTERFACE_INCLUDE_DIRECTORIES>
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)
