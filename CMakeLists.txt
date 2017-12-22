cmake_minimum_required(VERSION 2.8.3)
project(grad_traj_optimization)

add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/arc_utilities)
add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/sdf_tools)


project(grad_traj_optimization)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


set(NLopt_DIR /usr/local/lib/cmake/nlopt)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  sdf_tools
  NLopt
)
find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/third_party/sdf_tools/include)
include_directories(${PROJECT_SOURCE_DIR}/third_party/arc_utilities/include)
include_directories(${NLopt_INCLUDE_DIRS})

add_executable(rdp_example src/rdp_example.cpp)
target_link_libraries(rdp_example ${catkin_LIBRARIES})

add_executable(random src/example_random.cpp src/qp_generator.cpp )
target_link_libraries(random ${catkin_LIBRARIES} ${NLopt_LIBRARIES} ${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/lib/libnlopt.so.0.9.0)

add_executable(click src/example_click.cpp src/qp_generator.cpp )
target_link_libraries(click ${catkin_LIBRARIES} ${NLopt_LIBRARIES} ${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/lib/libnlopt.so.0.9.0)

add_executable(text_input src/example_text_input.cpp src/qp_generator.cpp )
target_link_libraries(text_input ${catkin_LIBRARIES} ${NLopt_LIBRARIES} ${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/lib/libnlopt.so.0.9.0)