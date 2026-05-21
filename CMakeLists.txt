cmake_minimum_required(VERSION 3.0.2)
project(log_view)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosgraph_msgs
)

set(CURSES_NEED_NCURSES TRUE)
find_package(Curses REQUIRED)
find_library(CURSES_PANEL_LIBRARY NAMES panel)
find_package(yaml-cpp REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp rosgraph_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CURSES_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/log_filter.cpp
  src/log_store.cpp
  src/log_view.cpp
  src/log_writer.cpp
  src/panel_interface.cpp
  src/preferences.cpp
  src/utils.cpp
  src/panels/details_panel.cpp
  src/panels/exclude_panel.cpp
  src/panels/filter_panel.cpp
  src/panels/help_panel.cpp
  src/panels/level_panel.cpp
  src/panels/log_panel.cpp
  src/panels/node_panel.cpp
  src/panels/prefs_panel.cpp
  src/panels/search_panel.cpp
  src/panels/status_panel.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CURSES_LIBRARIES}
  ${CURSES_PANEL_LIBRARY}
  yaml-cpp
  $<$<AND:$<CXX_COMPILER_ID:GNU>,$<VERSION_LESS:$<CXX_COMPILER_VERSION>,9.0>>:stdc++fs>
)

add_executable(log_viewer src/main.cpp)
target_link_libraries(log_viewer
  ${PROJECT_NAME}
  rosgraph_msgs::rosgraph_msgs__rosidl_typesupport_cpp
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} log_viewer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
