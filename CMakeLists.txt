cmake_minimum_required(VERSION 3.0.2)
project(log_view)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

set(CURSES_NEED_NCURSES TRUE)
find_package(Curses REQUIRED)
find_library(CURSES_PANEL_LIBRARY NAMES panel)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp
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
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${CURSES_LIBRARIES} ${CURSES_PANEL_LIBRARY})

add_executable(log_viewer src/main.cpp)
target_link_libraries(log_viewer ${PROJECT_NAME})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} log_viewer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
