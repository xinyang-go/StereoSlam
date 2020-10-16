cmake_minimum_required(VERSION 3.17)
project(StereoSlam)

set(CMAKE_CXX_STANDARD 17)

add_compile_definitions(PROJECT_DIR="${CMAKE_SOURCE_DIR}")

# 如果有ccache则使用ccache加速编译
FIND_PROGRAM(CCACHE_FOUND ccache)
IF (CCACHE_FOUND)
    SET_PROPERTY(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    SET_PROPERTY(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
    MESSAGE("-- Use ccache for compiler")
ENDIF ()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(Threads REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(G2O REQUIRED)
find_package(Cholmod REQUIRED)
find_package(Boost REQUIRED system timer)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Sophus_INCLUDE_DIRS})
include_directories(${G2O_INCLUDE_DIRS})
include_directories(${CHOLMOD_INCLUDE_DIR})

add_subdirectory(StereoMV)
include_directories(StereoMV/include)

aux_source_directory(src src)
add_library(StereoSlam SHARED ${src})
target_link_libraries(StereoSlam Threads::Threads)
target_link_libraries(StereoSlam Boost::system Boost::timer)
target_link_libraries(StereoSlam ${OpenCV_LIBS})
target_link_libraries(StereoSlam ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${G2O_TYPES_SBA} ${CHOLMOD_LIBRARIES})

add_executable(app main.cpp)
target_link_libraries(app StereoSlam StereoMV)