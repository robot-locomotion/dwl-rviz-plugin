cmake_minimum_required(VERSION 2.8.3)
project(dwl_rviz_plugin)


find_package(catkin REQUIRED COMPONENTS
  dwl
  dwl_msgs
  roscpp
  rviz)

find_package(dwl REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

add_definitions(-DQT_NO_KEYWORDS)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rviz
  DEPENDS system_lib)


include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${dwl_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS}
                 ${Boost_LIBRARY_DIRS}
                 ${dwl_LIBRARY_DIRS})
                 
qt4_wrap_cpp(MOC_FILES
  include/dwl_rviz_plugin/PointVisual.h
  include/dwl_rviz_plugin/LineVisual.h
  include/dwl_rviz_plugin/ArrowVisual.h
  include/dwl_rviz_plugin/PolygonVisual.h
  include/dwl_rviz_plugin/WholeBodyStateDisplay.h
  include/dwl_rviz_plugin/WholeBodyTrajectoryDisplay.h
  include/dwl_rviz_plugin/ReducedTrajectoryDisplay.h
  include/dwl_rviz_plugin/RewardMapDisplay.h
  include/dwl_rviz_plugin/ObstacleMapDisplay.h
  include/dwl_rviz_plugin/FootstepRegionDisplay.h
  OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)

set(SOURCE_FILES
  src/PointVisual.cpp
  src/LineVisual.cpp
  src/ArrowVisual.cpp
  src/PolygonVisual.cpp
  src/WholeBodyStateDisplay.cpp
  src/WholeBodyTrajectoryDisplay.cpp
  src/ReducedTrajectoryDisplay.cpp
  src/RewardMapDisplay.cpp
  src/ObstacleMapDisplay.cpp
  src/FootstepRegionDisplay.cpp
  ${MOC_FILES})


add_library(${PROJECT_NAME}  ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME}  ${QT_LIBRARIES}
                                       ${Boost_LIBRARIES}
                                       ${dwl_LIBRARIES}
                                       ${catkin_LIBRARIES}
                                       -ldefault_plugin)
add_dependencies(${PROJECT_NAME}  ${dwl_msgs_EXPORTED_TARGETS}
                                  ${dwl_terrain_EXPORTED_TARGETS})