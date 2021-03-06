# Copyright: 2012 iCub Facility, Istituto Italiano di Tecnologia
# Author: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 3.5)
project(motorControlBasic)
find_package(YARP)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

set(SDFAST "${CMAKE_CURRENT_SOURCE_DIR}/IKD/sdfast/")
set(CODES  "${CMAKE_CURRENT_SOURCE_DIR}/IKD/")
set(ID 	   "${CMAKE_CURRENT_SOURCE_DIR}/IKD/icub_id_cvxgen/")
set(IK 	   "${CMAKE_CURRENT_SOURCE_DIR}/IKD/IKCG_cvxgen/")
set(IMU	   "${CMAKE_CURRENT_SOURCE_DIR}/IMU/")

include_directories (${CODES}/include/)
include_directories (${SDFAST}/include/)
include_directories (${IMU}/include/)
include_directories (${ID}/)
include_directories (${IK}/)

file(GLOB MODEL_SOURCES "${SDFAST}/*.c")
file(GLOB ID_SOURCES 	"${ID}/*.c")
file(GLOB IK_SOURCES 	"${IK}/*.c")
file(GLOB IMU_SOURCES 	"${IMU}/src/*.c" "${IMU}/src/*.cpp")
file(GLOB EXTRA_SOURCES "${CODES}/src/*.cpp")
file(GLOB STACK_SOURCES "main.cpp" "wrapper.cpp")

SET(CMAKE_CXX_FLAGS "-O3")
SET(CMAKE_C_FLAGS "-O3")
SET(CMAKE_C_FLAGS "-D USE_USB_CONNECTION")

add_library (Model ${MODEL_SOURCES} ${ID_SOURCES} ${IK_SOURCES} ${IMU_SOURCES} ${EXTRA_SOURCES})
add_executable (main ${STACK_SOURCES})
TARGET_LINK_LIBRARIES(main ${YARP_LIBRARIES} Model pthread)
