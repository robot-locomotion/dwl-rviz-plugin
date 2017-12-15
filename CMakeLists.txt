cmake_minimum_required(VERSION 2.8.3)
project(dwl_rviz_plugin)


find_package(catkin REQUIRED COMPONENTS
  dwl
  dwl_msgs
  roscpp
  rviz)

find_package(dwl REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})

  qt4_wrap_cpp(MOC_FILES
    include/dwl_rviz_plugin/PointVisual.h
    include/dwl_rviz_plugin/LineVisual.h
    include/dwl_rviz_plugin/ArrowVisual.h
    include/dwl_rviz_plugin/PolygonVisual.h
    include/dwl_rviz_plugin/WholeBodyStateDisplay.h
    include/dwl_rviz_plugin/WholeBodyTrajectoryDisplay.h
    include/dwl_rviz_plugin/ReducedTrajectoryDisplay.h
    include/dwl_rviz_plugin/TerrainMapDisplay.h
    include/dwl_rviz_plugin/ObstacleMapDisplay.h
    include/dwl_rviz_plugin/FootstepRegionDisplay.h
    OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)

else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)

  qt5_wrap_cpp(MOC_FILES
    include/dwl_rviz_plugin/PointVisual.h
    include/dwl_rviz_plugin/LineVisual.h
    include/dwl_rviz_plugin/ArrowVisual.h
    include/dwl_rviz_plugin/PolygonVisual.h
    include/dwl_rviz_plugin/WholeBodyStateDisplay.h
    include/dwl_rviz_plugin/WholeBodyTrajectoryDisplay.h
    include/dwl_rviz_plugin/ReducedTrajectoryDisplay.h
    include/dwl_rviz_plugin/TerrainMapDisplay.h
    include/dwl_rviz_plugin/ObstacleMapDisplay.h
    include/dwl_rviz_plugin/FootstepRegionDisplay.h
    OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)

endif()

add_definitions(-DQT_NO_KEYWORDS)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rviz
  )


# Setting flags for optimization
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -march=native -mno-avx -mtune=native -D EIGEN_NO_DEBUG")


include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${dwl_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS}
                 ${Boost_LIBRARY_DIRS}
                 ${dwl_LIBRARY_DIRS})
                 

set(SOURCE_FILES
  src/DisplayInterface.cpp
  src/PointVisual.cpp
  src/LineVisual.cpp
  src/ArrowVisual.cpp
  src/PolygonVisual.cpp
  src/WholeBodyStateDisplay.cpp
  src/WholeBodyTrajectoryDisplay.cpp
  src/ReducedTrajectoryDisplay.cpp
  src/TerrainMapDisplay.cpp
  src/ObstacleMapDisplay.cpp
  src/FootstepRegionDisplay.cpp
  ${MOC_FILES})


add_library(${PROJECT_NAME}  ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME}  ${QT_LIBRARIES}
                                       ${Boost_LIBRARIES}
                                       ${dwl_LIBRARIES}
                                       ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}  ${dwl_msgs_EXPORTED_TARGETS}
                                  ${dwl_terrain_EXPORTED_TARGETS})
