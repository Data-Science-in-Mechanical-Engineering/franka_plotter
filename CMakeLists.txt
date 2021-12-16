cmake_minimum_required(VERSION 3.4)
cmake_policy(SET CMP0048 NEW)
project(franka_plotter)

# c++
set(CMAKE_CXX_STANDARD 11)

# eigen3
find_package(Eigen3 REQUIRED)

# libfranka
find_package(Franka)
if (NOT Franka_FOUND)
  find_package(catkin COMPONENTS franka_control REQUIRED)
endif()

add_executable            (franka_plotter
source/arc.cpp
source/bezier_cubic.cpp
source/bezier_quadratic.cpp
source/configuration.cpp
source/line.cpp
source/drawing.cpp
source/main.cpp
source/plotter.cpp
source/segment.cpp)
target_include_directories(franka_plotter PUBLIC ${EIGEN3_INCLUDE_DIRS})
target_link_libraries     (franka_plotter PUBLIC Franka::Franka)

configure_file(source/vectorize.sh vectorize.sh)
