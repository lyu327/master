cmake_minimum_required(VERSION 2.8)
project(tt)
set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_FLAGS "-std=c++11")

#for g2o
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules/")
SET( G2O_ROOT /usr/local/include/g2o )
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIR})
#for CSparse
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})
find_package(Cholmod REQUIRED)
INCLUDE_DIRECTORIES(${CHOLMOD_INCLUDE_DIR})
#Eigen
INCLUDE_DIRECTORIES("/usr/include/eigen3")
add_executable(tt main.cpp)
target_link_libraries(tt ${G2O_LIBS} ${CSPARSE_LIBRARY} ${CHOLMOD_LIBRARIES} g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension g2o_types_sba)
