cmake_minimum_required(VERSION 3.13)
project(robot)

set(EXECNAME_MAIN robot)
set(CURL_STATICLIB true)
add_definitions(-DCURL_STATICLIB)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -lcurl -pthread")
#set(CMAKE_EXE_LINKER_FLAGS "-static -static-libgcc -static-libstdc++")

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}") ##Disaple optimozation for profiling


set(CURL_CA_PATH none)

### Qt  ###
#set(CMAKE_PREFIX_PATH C:/Qt//5.12.6/mingw73_32)
# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

#set(QTPATH C:/Qt/5.12.6/mingw73_32/lib/cmake)
#find_package(Qt5 REQUIRED COMPONENTS Core Widgets HINTS ${QTPATH})
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
### end Qt  ###

### CPR  ###
set(ENV{CMAKE_USE_OPENSSL} OFF)
set(ENV{BUILD_CPR_TESTS} OFF)
set(ENV{USE_SYSTEM_CURL} OFF) #OFF for windows

#uncomment next 2 for windws
add_subdirectory(external/cpr)
include_directories(${CPR_INCLUDE_DIRS})

### end CPR ###


include_directories(external)

file(GLOB robot_HEAD "include/*.h")
file(GLOB robot_SRC "src/*.cpp")


add_executable(${EXECNAME_MAIN} main.cpp ${robot_HEAD} ${robot_SRC} external/json/json.hpp)

include_directories(${CURL_INCLUDE_DIRS})

target_link_libraries(${EXECNAME_MAIN} ${CURL_LIBRARIES} -lcurl  ${CPR_LIBRARIES} Qt5::Core Qt5::Widgets)

