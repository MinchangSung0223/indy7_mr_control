cmake_minimum_required(VERSION 3.0)
FIND_PACKAGE(jsoncpp REQUIRED)
include_directories(${JSONCPP_INCLUDE_DIRS})

PROJECT(pybullet_cpp_indy7)

cmake_minimum_required(VERSION 3.12)

SUBDIRS(bullet3)
file(GLOB_RECURSE INDY7 src/Indy7.cpp src/Indy7.h )

file(GLOB_RECURSE MR lib/ModernRobotics.cpp lib/ModernRobotics.h)


add_executable(pybullet_cpp_indy7 src/main.cpp ${INDY7} ${MR} )

target_link_libraries(pybullet_cpp_indy7 PUBLIC pybullet_cpp)
target_link_libraries(pybullet_cpp_indy7 PUBLIC jsoncpp_lib)



