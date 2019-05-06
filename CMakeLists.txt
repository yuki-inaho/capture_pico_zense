cmake_minimum_required(VERSION 3.5.1)

# project set
set(PROJ_NAME capture_pico_zense)
project(${PROJ_NAME})
add_executable(
    ${PROJ_NAME} 
    src/main.cpp
    src/PicoZenseSensorClass.cpp
    src/PicoZenseSetter.cpp
    src/ParameterManager.cpp
    src/SensorManager.cpp
    )

set(CMAKE_CXX_FLAGS "-O3 -std=c++11")

include_directories(include)

find_package(PkgConfig REQUIRED)

pkg_check_modules(OpenCV REQUIRED opencv)
include_directories(${OpenCV_CFLAGS})
target_link_libraries(${PROJ_NAME} ${OpenCV_LDFLAGS})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJ_NAME} ${PCL_LIBRARIES})

pkg_check_modules(PicoZense libpicozense REQUIRED)
include_directories(${PicoZense_INCLUDE_DIRS})
target_link_libraries(${PROJ_NAME} ${PicoZense_LDFLAGS})
