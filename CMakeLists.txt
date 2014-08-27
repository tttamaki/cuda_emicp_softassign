cmake_minimum_required(VERSION 2.8)
set(CMAKE_VERBOSE_MAKEFILE 1) # to see all make process

project( cuda_emicp_softassign )

SET_PROPERTY(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
SET_PROPERTY(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache) 


find_package(PCL 1.7 REQUIRED)
include_directories ( ${PCL_INCLUDE_DIRS} )
link_directories ( ${PCL_LIBRARY_DIRS} )
add_definitions ( ${PCL_DEFINITIONS} )

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/" ${CMAKE_MODULE_PATH})
include(FindFlann)


FIND_PACKAGE(OpenMP REQUIRED)
if(OPENMP_FOUND)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()


find_package(CUDA 6.5 REQUIRED)
include_directories ( ${CUDA_TOOLKIT_ROOT_DIR}/samples/common/inc/ )

message("CUDA_VERSION: " ${CUDA_VERSION} )
message("CUDA_CUDA_LIBRARY: " ${CUDA_CUDA_LIBRARY} )
message("CUDA_CUDART_LIBRARY: " ${CUDA_CUDART_LIBRARY} )
message("CUDA_NVCC_FLAGS: " ${CUDA_NVCC_FLAGS} )
message("CUDA_INCLUDE_DIRS: " ${CUDA_INCLUDE_DIRS} )
message("CUDA_CUBLAS_LIBRARIES: " ${CUDA_CUBLAS_LIBRARIES} )
message("CUDA_TOOLKIT_ROOT_DIR: " ${CUDA_TOOLKIT_ROOT_DIR} )


find_library(ATLAS atlas)

include(FindLAPACK)
add_definitions ( ${LAPACK_LINKER_FLAGS} )





cuda_add_executable ( ${PROJECT_NAME} src/main.cpp src/findRTfromS.cpp src/icp.cpp src/emicp_cpu.cpp src/softassign.cu src/emicp.cu )
target_link_libraries ( ${PROJECT_NAME} ${PCL_LIBRARIES} ${LAPACK_LIBRARIES} )

CUDA_ADD_CUBLAS_TO_TARGET ( ${PROJECT_NAME} )
