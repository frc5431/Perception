cmake_minimum_required(VERSION 2.8.11)

project(perception)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake_modules")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -Wall")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin")
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREAD ON)

find_package(Boost 1.50.3 REQUIRED COMPONENTS date_time filesystem chrono thread atomic wave coroutine system iostreams serialization locale random)

if (Boost_FOUND)
include_directories(${Boost_INCLUDE_DIR})
endif()

find_package(OpenCV 3.2.0 REQUIRED)

IF (OpenCV_FOUND)
include_directories(${OpenCV_INCLUDE_DIR})
endif()

if (PKG_CONFIG_FOUND)
  PKG_CHECK_MODULES(OpenNI2 libopenni2)
endif()

find_package(OpenNI2 REQUIRED)
if (OpenNI2_FOUND)
include_directories(${OpenNI2_INCLUDE_DIR})
endif()

find_package(freenect2 REQUIRED)
if (freenect2_FOUND)
include_directories(${freenect2_INCLUDE_DIR})
endif()

find_package(ntcore REQUIRED)
if (ntcore_FOUND)
include_directories(${ntcore_INCLUDE_DIR})
endif()

file(GLOB_RECURSE SOURCE_FILES "src/*.cpp")

add_executable(perception ${SOURCE_FILES})

target_include_directories(perception PUBLIC "include/")
target_link_libraries(perception ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} ${freenect2_LIBRARY} ${OpenNI2_LIBRARIES} ${ntcore_LIBRARY})
