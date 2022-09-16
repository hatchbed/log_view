cmake_minimum_required(VERSION 3.5)
project(log_view)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)

set(CURSES_NEED_NCURSES TRUE)
find_package(Curses REQUIRED)
find_library(CURSES_PANEL_LIBRARY NAMES panel)

###########
## Build ##
###########

include_directories(
  include
  ${CURSES_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/log_filter.cpp
  src/log_store.cpp
  src/log_view.cpp
  src/panel_interface.cpp
  src/utils.cpp
  src/panels/exclude_panel.cpp
  src/panels/filter_panel.cpp
  src/panels/help_panel.cpp
  src/panels/level_panel.cpp
  src/panels/log_panel.cpp
  src/panels/node_panel.cpp
  src/panels/search_panel.cpp
  src/panels/status_panel.cpp
)
target_link_libraries(${PROJECT_NAME} ${CURSES_LIBRARIES} ${CURSES_PANEL_LIBRARY})
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rcl_interfaces
)

add_executable(log_viewer src/main.cpp)
target_link_libraries(log_viewer ${PROJECT_NAME})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} log_viewer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp rcl_interfaces)
ament_package()