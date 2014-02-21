cmake_minimum_required(VERSION 2.6)

include_directories("${PROJECT_SOURCE_DIR}/include")
include_directories("${PROJECT_SOURCE_DIR}/src")


add_executable(
        manipulabilityTests Main.cpp
        MyWindow.h MyWindow.cpp
		# ${PROJECT_SOURCE_DIR}/include/kinematics/joint.h
)

target_link_libraries(manipulabilityTests ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} )

TARGET_LINK_LIBRARIES(manipulabilityTests debug ${RTQL8_KINEMATIC_LIBRARY}/libticppd)
TARGET_LINK_LIBRARIES(manipulabilityTests debug ${RTQL8_KINEMATIC_LIBRARY}/utilsd)
TARGET_LINK_LIBRARIES(manipulabilityTests debug ${RTQL8_KINEMATIC_LIBRARY}/kinematicsd)
TARGET_LINK_LIBRARIES(manipulabilityTests debug ${RTQL8_KINEMATIC_LIBRARY}/collisiond)
TARGET_LINK_LIBRARIES(manipulabilityTests debug ${RTQL8_KINEMATIC_LIBRARY}/simulationd)
TARGET_LINK_LIBRARIES(manipulabilityTests debug ${RTQL8_KINEMATIC_LIBRARY}/dynamicsd)
TARGET_LINK_LIBRARIES(manipulabilityTests debug ${RTQL8_KINEMATIC_LIBRARY}/rendererd)
TARGET_LINK_LIBRARIES(manipulabilityTests debug ${RTQL8_KINEMATIC_LIBRARY}/lcpsolverd)
TARGET_LINK_LIBRARIES(manipulabilityTests debug ${RTQL8_KINEMATIC_LIBRARY}/integrationd)
TARGET_LINK_LIBRARIES(manipulabilityTests debug ${RTQL8_KINEMATIC_LIBRARY}/yuid)
TARGET_LINK_LIBRARIES(manipulabilityTests debug rtql8Manipulability)

TARGET_LINK_LIBRARIES(manipulabilityTests optimized ${RTQL8_KINEMATIC_LIBRARY}/libticpp)
TARGET_LINK_LIBRARIES(manipulabilityTests optimized ${RTQL8_KINEMATIC_LIBRARY}/utils)
TARGET_LINK_LIBRARIES(manipulabilityTests optimized ${RTQL8_KINEMATIC_LIBRARY}/kinematics)
TARGET_LINK_LIBRARIES(manipulabilityTests optimized ${RTQL8_KINEMATIC_LIBRARY}/collision)
TARGET_LINK_LIBRARIES(manipulabilityTests optimized ${RTQL8_KINEMATIC_LIBRARY}/simulation)
TARGET_LINK_LIBRARIES(manipulabilityTests optimized ${RTQL8_KINEMATIC_LIBRARY}/dynamics)
TARGET_LINK_LIBRARIES(manipulabilityTests optimized ${RTQL8_KINEMATIC_LIBRARY}/lcpsolver)
TARGET_LINK_LIBRARIES(manipulabilityTests optimized ${RTQL8_KINEMATIC_LIBRARY}/integration)
TARGET_LINK_LIBRARIES(manipulabilityTests optimized ${RTQL8_KINEMATIC_LIBRARY}/renderer)
TARGET_LINK_LIBRARIES(manipulabilityTests optimized ${RTQL8_KINEMATIC_LIBRARY}/yui)
TARGET_LINK_LIBRARIES(manipulabilityTests optimized rtql8Manipulability)
