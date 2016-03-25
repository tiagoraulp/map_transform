cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/planning)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/visibility)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/utils)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "-std=c++0x -std=gnu11 ${CMAKE_CXX_FLAGS}")

#set(CMAKE_BUILD_TYPE Debug)

#add_definitions ("-Wall -Wconversion")

find_package(OpenCV)

rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(${OpenCV_INCLUDE_DIRS})

rosbuild_add_boost_directories()

rosbuild_add_executable(visibility src/visibility/vis.cpp src/visibility/vis_transf.cpp src/visibility/visC_transf.cpp src/visibility/unreachable.cpp src/utils/labelling.cpp src/utils/clustering.cpp src/visibility/CritPoints.cpp src/visibility/obtuseAngle.cpp src/utils/bugfollowing.cpp src/utils/chain.cpp src/visibility/obs_extremes.cpp src/utils/vector_utils.cpp src/utils/ray.cpp src/utils/color.cpp src/visibility/brute_force.cpp)

rosbuild_add_executable(visNC src/visibility/visNC.cpp src/visibility/vis_transf.cpp   src/visibility/visNC_transf.cpp src/visibility/unreachable.cpp src/utils/labelling.cpp src/utils/clustering.cpp src/visibility/CritPoints.cpp src/visibility/obtuseAngle.cpp src/utils/bugfollowing.cpp src/utils/chain.cpp src/visibility/obs_extremes.cpp src/utils/vector_utils.cpp src/utils/ray.cpp src/utils/color.cpp src/visibility/brute_force.cpp  src/visibility/morph.cpp)

rosbuild_add_executable(planner src/planning/planner.cpp)

rosbuild_add_executable(planSens src/planning/planSensing.cpp src/utils/ray.cpp)

rosbuild_add_executable(planSensOpt src/planning/planSensingOpt.cpp src/utils/ray.cpp src/utils/vector_utils.cpp)

rosbuild_add_executable(pddl-gen src/planning/pddl-generator.cpp src/utils/vector_utils.cpp)

target_link_libraries(visibility ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})

target_link_libraries(visNC ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})

rosbuild_link_boost(visibility thread signals system)

rosbuild_link_boost(visNC thread signals system)
