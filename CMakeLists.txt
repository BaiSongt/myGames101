# Set the minimum required version of CMake
cmake_minimum_required(VERSION 3.12)

# Set the C++ standard version to 17
set(CMAKE_CXX_STANDARD 17)

# Define a project with the name "GAMES101"
project(GAMES101)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# create a target name for every homework
set(TARGET_NAME ${PROJECT_NAME})

# Set the installation prefix for the project
set(CMAKE_INSTALL_PREFIX ${CMAKE_HOME_DIRECTORY})

# 添加动态库依赖 Eigen3 and OpenCV
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# Add the Eigen3 include directory to the compiler flags
include_directories(${Eigen3_INCLUDE_DIRS})
# Add the OpenCV include directories to the compiler flags
include_directories(${OpenCV_INCLUDE_DIRS})

add_subdirectory(00)
add_subdirectory(01)
# add_subdirectory(02)
# add_subdirectory(03)
# add_subdirectory(04)
# add_subdirectory(05)
# add_subdirectory(06)
# add_subdirectory(07)

install(TARGETS 00 01 DESTINATION  output)
# install(DIRECTORY 03/models DESTINATION output)
# install(DIRECTORY 06/models DESTINATION output)
# install(DIRECTORY 07/models DESTINATION output)
