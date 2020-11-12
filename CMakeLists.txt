cmake_minimum_required(VERSION 3.0.2)
project(log_view)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

set(CURSES_NEED_NCURSES TRUE)
find_package(Curses REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES log_view
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
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
  src/log_store.cpp
  src/log_view.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${CURSES_LIBRARIES})

add_executable(log_viewer src/main.cpp)
target_link_libraries(log_viewer ${PROJECT_NAME})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} log_viewer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
