# Copyright: 2012 iCub Facility, Istituto Italiano di Tecnologia
# Author: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# Modified by: Ilaria Lauzana, EPFL

cmake_minimum_required(VERSION 3.5)
project(motorControlBasic)
find_package(YARP)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

set(SDFAST "${CMAKE_CURRENT_SOURCE_DIR}/../../IKD/sdfast/")
set(CODES  "${CMAKE_CURRENT_SOURCE_DIR}/../../IKD/")
set(ID 	   "${CMAKE_CURRENT_SOURCE_DIR}/../../IKD/icub_id_cvxgen/")
set(IK 	   "${CMAKE_CURRENT_SOURCE_DIR}/../../IKD/IKCG_cvxgen/")
set(WRAPPER "${CMAKE_CURRENT_SOURCE_DIR}/../../wrapper/")

include_directories (${CODES}/include/)
include_directories (${SDFAST}/include/)
include_directories (${WRAPPER}/include/)
include_directories (${MULTIDS}/include/)
include_directories (${MATHLIB}/include/)
include_directories (${MOTION}/include/)
include_directories (${ID}/)
include_directories (${IK}/)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/lib/)

file(GLOB MODEL_SOURCES "${SDFAST}/*.c")
file(GLOB ID_SOURCES 	"${ID}/*.c")
file(GLOB IK_SOURCES 	"${IK}/*.c")
file(GLOB EXTRA_SOURCES "${CODES}/src/*.cpp")
file(GLOB WRAPPER_SOURCES "${WRAPPER}/src/*.cpp")
file(GLOB STACK_SOURCES "main.cpp")
file( GLOB LIB_SOURCES lib/*.cpp )
file( GLOB LIB_HEADERS lib/*.h )

#SET(CMAKE_CXX_FLAGS "-O3")
#SET(CMAKE_C_FLAGS "-O3")
set(CMAKE_CXX_FLAGS "-std=c++11")

add_library (Model ${MODEL_SOURCES} ${ID_SOURCES} ${IK_SOURCES} ${EXTRA_SOURCES})
add_library (Bifurcation ${LIB_SOURCES} ${LIB_HEADERS} )
add_executable (main ${STACK_SOURCES} ${WRAPPER_SOURCES})
TARGET_LINK_LIBRARIES(main ${YARP_LIBRARIES} Model Bifurcation)

add_executable (testbif test.cpp ${WRAPPER_SOURCES})
TARGET_LINK_LIBRARIES(testbif ${YARP_LIBRARIES} Model Bifurcation)
