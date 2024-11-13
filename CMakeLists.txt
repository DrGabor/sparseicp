cmake_minimum_required(VERSION 2.8)
project(sparseicp)

#--- CMake configuration
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

#--- Compiler configuration
# can be relwithdebinfo or debug
# set(CMAKE_BUILD_TYPE Debug)
# 设置 Debug 模式下的编译器标志
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")  # 禁用优化并启用调试信息
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

#--- OpenMP (cannot work in clang)
find_package(OpenMP QUIET)
if(OPENMP_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
endif()

#--- Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
message(STATUS "EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")
message(STATUS "EIGEN3_VERSION: ${EIGEN3_VERSION}")

#--- Nanoflann
find_package(NanoFlann REQUIRED)
include_directories(${NANOFLANN_INCLUDE_DIR})

# Boost
find_package(Boost REQUIRED COMPONENTS chrono system filesystem thread
             REQUIRED)

# gflags
# https://gflags.github.io/gflags/#cmake
find_package(gflags REQUIRED)
message(STATUS "GFLAGS_INCLUDE_DIRS: ${GFLAGS_INCLUDE_DIRS}")
message(STATUS "GFLAGS_LIBRARIES: ${GFLAGS_LIBRARIES}")

# glog
# https://stackoverflow.com/questions/65588745/could-not-find-glog-missing-glog-include-dir-glog-library/65588917#65588917
# build from source seems to be problematic, possibly due to GLOG is related to ROS2::GFLAGS. 
# Try to "sudo apt install libgoogle-glog-dev"
find_package (glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})
message(include(SelectLibraryConfigurations) "GLOG_INCLUDE_DIRS: ${GLOG_INCLUDE_DIRS}")
message(STATUS "GLOG_LIBRARIES: ${GLOG_LIBRARIES}")

# PCL
# https://pcl.readthedocs.io/projects/tutorials/en/master/
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#--- Build example
include_directories(.)
file(GLOB SOURCES "*.h")
file(GLOB HEADERS "*.cpp")
add_executable(sparseicp ${SOURCES} ${HEADERS})
target_link_libraries(sparseicp 
                      ${LIBRARIES} 
                      ${PCL_LIBRARIES}
                      ${GFLAGS_LIBRARIES}
                      /usr/lib/x86_64-linux-gnu/libglog.so)

if(APPLE OR UNIX)
    #--- Deploy data folder link
    execute_process(COMMAND ln -f -s ${CMAKE_SOURCE_DIR}/data WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
    #--- "Dummy" target to have data appear in QtCreator
    add_custom_target(data SOURCES ${CMAKE_SOURCE_DIR}/data)
else()
    file(COPY ${CMAKE_SOURCE_DIR}/data DESTINATION ${PROJECT_BINARY_DIR})
endif()
