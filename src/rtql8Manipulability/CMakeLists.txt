cmake_minimum_required(VERSION 2.6)

include_directories("${PROJECT_SOURCE_DIR}/include")
include_directories("${PROJECT_SOURCE_DIR}/src")


add_library(
        rtql8Manipulability 
		DecomposedSkeleton.h DecomposedSkeleton.cpp
		sampling/Sample.h sampling/Sample.cpp
		${PROJECT_SOURCE_DIR}/src/utils/MatrixDefsInternal.h
		${PROJECT_SOURCE_DIR}/src/utils/MatrixDefs.h
		${PROJECT_SOURCE_DIR}/src/utils/MatrixDefs.cpp
		# ${PROJECT_SOURCE_DIR}/include/kinematics/joint.h
)
