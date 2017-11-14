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
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/utils)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/visibility)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/types)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/planning)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/conversions)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/map)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "-std=c++1y ${CMAKE_CXX_FLAGS}")

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

SET(UTILS_SRCS
    src/utils/bugfollowing.cpp
    src/utils/chain.cpp
    src/utils/clustering.cpp
    src/utils/color.cpp
    src/utils/labelling.cpp
    src/utils/ray.cpp
    src/utils/vector_utils.cpp
    src/utils/combinatorics.cpp
)

SET(VISIBILITY_SRCS
    src/visibility/unreachable.cpp
    src/visibility/CritPoints.cpp
    src/visibility/obtuseAngle.cpp
    src/visibility/obs_extremes.cpp
    src/visibility/brute_force.cpp
)

SET(TYPES_SRCS
    src/types/pointi.cpp
)

SET(CONVERSIONS_SRCS
    src/conversions/points_conversions.cpp
)

SET(PLANNING_SRCS
    src/planning/Astar.cpp
    src/planning/PAstar.cpp
)

ADD_LIBRARY(utils_library SHARED ${UTILS_SRCS})
target_link_libraries(utils_library ${OpenCV_LIBRARIES})

ADD_LIBRARY(vis_library SHARED ${VISIBILITY_SRCS})
target_link_libraries(vis_library utils_library)

ADD_LIBRARY(types_library SHARED ${TYPES_SRCS})

ADD_LIBRARY(conversions_library SHARED ${CONVERSIONS_SRCS})
target_link_libraries(conversions_library types_library ${OpenCV_LIBRARIES})

ADD_LIBRARY(planning_library SHARED ${PLANNING_SRCS})
target_link_libraries(planning_library types_library utils_library)

SET(target visibility)
ROSBUILD_ADD_EXECUTABLE(${target}
    src/visibility/vis.cpp
    src/visibility/vis_transf.cpp
    src/visibility/visC_transf.cpp)
TARGET_LINK_LIBRARIES(${target} vis_library)

SET(target visNC)
ROSBUILD_ADD_EXECUTABLE(${target}
    src/visibility/visNC.cpp
    src/visibility/vis_transf.cpp
    src/visibility/visNC_transf.cpp
    src/visibility/morph.cpp)
TARGET_LINK_LIBRARIES(${target} vis_library)

SET(target planner)
rosbuild_add_executable(${target} src/planning/planner.cpp)
TARGET_LINK_LIBRARIES(${target} planning_library conversions_library)

SET(target multiPlannerSens)
rosbuild_add_executable(${target} src/planning/multi_planner_sensing.cpp)
TARGET_LINK_LIBRARIES(${target} planning_library conversions_library)

SET(target planSensOpt)
rosbuild_add_executable(${target} src/planning/planSensingOpt.cpp)
TARGET_LINK_LIBRARIES(${target} planning_library conversions_library)

SET(target pddl-gen)
rosbuild_add_executable(${target} src/planning/pddl-generator.cpp)
TARGET_LINK_LIBRARIES(${target} planning_library conversions_library)

SET(target pddl-gen-NC)
rosbuild_add_executable(${target} src/planning/pddl-generator_NC.cpp)
TARGET_LINK_LIBRARIES(${target} planning_library conversions_library)

SET(target map-gen)
rosbuild_add_executable(${target} src/map/map-generator.cpp)
TARGET_LINK_LIBRARIES(${target} utils_library conversions_library)

SET(target room-gen)
rosbuild_add_executable(${target} src/map/room-generator.cpp)

SET(target obs-gen)
rosbuild_add_executable(${target} src/map/obstacle-generator.cpp)
TARGET_LINK_LIBRARIES(${target} utils_library conversions_library)

SET(target map-planner)
rosbuild_add_executable(${target} src/map/map-planner.cpp)
