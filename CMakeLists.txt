cmake_minimum_required(VERSION 2.8.3)

project(rqt_embed_window)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  rqt_gui
  rqt_gui_cpp

)

catkin_python_setup()

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/rqt_embed_window
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
