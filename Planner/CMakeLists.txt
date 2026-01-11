cmake_minimum_required(VERSION 3.5)
project(planner_node)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(LAPACK REQUIRED)

option(WITH_HSL "Compile solver with coinhsl" ON)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(HSL)
if(NOT ${HSL_FOUND})
  set(WITH_HSL OFF)
endif()

if(WITH_HSL)
  add_definitions(-DWITH_HSL)
endif()

find_package(
  catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  roscpp
  rospy
  visualization_msgs
  sandbox_msgs
  tf
  std_msgs
  nav_msgs
)

add_message_files(
  FILES
  Polygon2d.msg
  Polygon2dArray.msg
  Vec2d.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DRECORD_FOR_PLAYBACK")
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DEXPERIMENT")
SET(CMAKE_BUILD_TYPE "RELEASE")
add_compile_options(-O3 -march=native -mtune=native -funroll-loops -flto)

catkin_package(
 INCLUDE_DIRS .
 CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ./
  # /usr/include/eigen3/
)

add_executable(planner_node1
  common/math/aabox2d.cc
  common/math/box2d.cc
  common/math/line_segment2d.cc
  common/math/linear_interpolation.cc
  common/math/math_utils.cc
  common/math/polygon2d.cc
  common/math/trajectory1d.cc
  common/math/vec2d.cc
  common/math/quintic_polynomial_curve1d.cc
  common/math/angle.cc
  common/math/sin_table.cc
  common/math/integral.cc
  common/math/quintic_spiral_path.cc
  common/data/discretized_trajectory.cc
  common/util/point_cloud.cc
  common/util/color.cc
  common/util/file_utils.cc
  common/visualization_plot.cpp
  common/environment.cc
  common/obstacle.cc
  map/map_generator.cc
  planning/decider/dp_path_decider.cc
  planning/decider/dp_sv_graph.cc
  planning/trajectory_nlp/trajectory_nlp.cc
  planning/trajectory_nlp/trajectory_optimizer.cc

  common/my_env.cpp
  interface.cc

  planner_node.cpp
)

target_include_directories(planner_node1 PUBLIC
  ./
  # third_party/nlohmann/
  ${catkin_INCLUDE_DIRS}
)

find_library(CASADI_LIBRARY casadi PATHS /usr/local/lib)
target_link_libraries(planner_node1
  ${CASADI_LIBRARY}
  ${catkin_LIBRARIES}
)

# 多AGV轨迹生成器
add_executable(multi_agv_trajectory_generator
  common/math/aabox2d.cc
  common/math/box2d.cc
  common/math/line_segment2d.cc
  common/math/linear_interpolation.cc
  common/math/math_utils.cc
  common/math/polygon2d.cc
  common/math/trajectory1d.cc
  common/math/vec2d.cc
  common/math/quintic_polynomial_curve1d.cc
  common/math/angle.cc
  common/math/sin_table.cc
  common/math/integral.cc
  common/math/quintic_spiral_path.cc
  common/data/discretized_trajectory.cc
  common/util/point_cloud.cc
  common/util/color.cc
  common/util/file_utils.cc
  common/visualization_plot.cpp
  common/environment.cc
  common/obstacle.cc
  map/map_generator.cc
  planning/decider/dp_path_decider.cc
  planning/decider/dp_sv_graph.cc
  planning/trajectory_nlp/trajectory_nlp.cc
  planning/trajectory_nlp/trajectory_optimizer.cc

  common/my_env.cpp
  kkswarm/multi_agv_trajectory_generator.cpp
)

target_link_libraries(multi_agv_trajectory_generator
  ${catkin_LIBRARIES}
)

add_dependencies(multi_agv_trajectory_generator
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# kkswarm控制节点
add_executable(kkswarm_control_node
  common/math/aabox2d.cc
  common/math/box2d.cc
  common/math/line_segment2d.cc
  common/math/linear_interpolation.cc
  common/math/math_utils.cc
  common/math/polygon2d.cc
  common/math/trajectory1d.cc
  common/math/vec2d.cc
  common/math/quintic_polynomial_curve1d.cc
  common/math/angle.cc
  common/math/sin_table.cc
  common/math/integral.cc
  common/math/quintic_spiral_path.cc
  common/data/discretized_trajectory.cc
  common/util/point_cloud.cc
  common/util/color.cc
  common/util/file_utils.cc
  common/visualization_plot.cpp
  common/environment.cc
  common/obstacle.cc
  map/map_generator.cc
  planning/decider/dp_path_decider.cc
  planning/decider/dp_sv_graph.cc
  planning/trajectory_nlp/trajectory_nlp.cc
  planning/trajectory_nlp/trajectory_optimizer.cc

  common/my_env.cpp
  kkswarm/test_kkswarm_control_node.cpp
)

target_link_libraries(kkswarm_control_node
  ${catkin_LIBRARIES}
)

add_dependencies(kkswarm_control_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
