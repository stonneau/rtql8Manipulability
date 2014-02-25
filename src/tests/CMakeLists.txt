cmake_minimum_required(VERSION 2.6)
PROJECT(manipulabilityTests)
include_directories("${RTQL8_MANIPULABILITY_ROOT}/include")
include_directories("${RTQL8_MANIPULABILITY_ROOT}/src")
include_directories("${RTQL8_INCLUDE_DIR}/rtql8")

LINK_DIRECTORIES(“${RTQL8_LIBRARY_DIR}”)

add_executable(
        manipulabilityTests Main.cpp
        MyWindow.h MyWindow.cpp
)


TARGET_LINK_LIBRARIES(manipulabilityTests ${GLUT_glut_LIBRARY})
TARGET_LINK_LIBRARIES(manipulabilityTests 
						ticpp 
						utils
						yui
						kinematics
						collision
						simulation
						geometry
						dynamics
						lcpsolver
						integration
						renderer
						manipulability
						${GLUT_glut_LIBRARY}
						GL
						GLU
)

