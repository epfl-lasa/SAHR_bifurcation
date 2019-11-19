cmake_minimum_required (VERSION 2.6)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

set(SNOPT  "${CMAKE_CURRENT_SOURCE_DIR}/snopt/")
set(SDFAST "${CMAKE_CURRENT_SOURCE_DIR}/sdfast/")
set(CODES  "${CMAKE_CURRENT_SOURCE_DIR}/")
set(ID 	   "${CMAKE_CURRENT_SOURCE_DIR}/icub_id_cvxgen/")
set(IK 	   "${CMAKE_CURRENT_SOURCE_DIR}/IKCG_cvxgen/")

include_directories (${CODES}/include/)
include_directories (${SDFAST}/include/)
include_directories (${ID}/)
include_directories (${IK}/)
include_directories (${SNOPT}/include/)
link_directories    (${SNOPT}/lib/)

file(GLOB MODEL_SOURCES "${SDFAST}/*.c")
file(GLOB ID_SOURCES 	"${ID}/*.c")
file(GLOB IK_SOURCES 	"${IK}/*.c")
file(GLOB EXTRA_SOURCES "${CODES}/src/*.cpp")
file(GLOB STACK_SOURCES "main.cpp")

#SET(CMAKE_CXX_FLAGS "-O3")
#SET(CMAKE_C_FLAGS "-O3")

add_library (Model ${MODEL_SOURCES} ${ID_SOURCES} ${IK_SOURCES} ${EXTRA_SOURCES})
add_executable (main ${STACK_SOURCES})
TARGET_LINK_LIBRARIES(main Model snopt7_cpp snopt7 dl)
