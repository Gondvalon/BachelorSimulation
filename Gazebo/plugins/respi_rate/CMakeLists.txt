cmake_minimum_required(VERSION 2.8.3)
project(respi_rate)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

find_package(Boost REQUIRED COMPONENTS system)
find_package(gazebo REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
include(FindPkgConfig)
if (PKG_CONFIG_FOUND)
	pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(respi_rate SHARED respi_rate.cpp)
target_link_libraries(respi_rate ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
