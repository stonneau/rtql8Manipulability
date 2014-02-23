cmake_minimum_required(VERSION 2.6)
PROJECT(manipulability)
include_directories("${RTQL8_INCLUDE_DIR}/rtql8")
include_directories("${RTQL8_MANIPULABILITY_ROOT}/include")
include_directories("${RTQL8_MANIPULABILITY_ROOT}/src")
include_directories("${CMAKE_BINARY_DIR}/lib")

set(SOURCES 
	DecomposedSkeleton.h DecomposedSkeleton.cpp
		sampling/Sample.h sampling/Sample.cpp
		${RTQL8_MANIPULABILITY_ROOT}/src/utils/MatrixDefsInternal.h
		${RTQL8_MANIPULABILITY_ROOT}/src/utils/MatrixDefs.h
		${RTQL8_MANIPULABILITY_ROOT}/src/utils/MatrixDefs.cpp
		${RTQL8_INCLUDE_DIR}/rtql8/kinematics/Skeleton.h
		${RTQL8_MANIPULABILITY_ROOT}/include/manipulability/ManipulabilityPaths.h
)

add_library(manipulability ${SOURCES})

SET_TARGET_PROPERTIES(manipulability PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
SET_TARGET_PROPERTIES(manipulability PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

