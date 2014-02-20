cmake_minimum_required(VERSION 2.6)

include_directories("${PROJECT_SOURCE_DIR}/include")
include_directories("${PROJECT_SOURCE_DIR}/src")


add_executable(
        manipulabilityTests Main.cpp
		# ${PROJECT_SOURCE_DIR}/include/kinematics/joint.h
)


TARGET_LINK_LIBRARIES(manipulabilityTests debug ${RTQL8_KINEMATIC_LIBRARY}/libticppd)
TARGET_LINK_LIBRARIES(manipulabilityTests debug ${RTQL8_KINEMATIC_LIBRARY}/utilsd)
TARGET_LINK_LIBRARIES(manipulabilityTests debug ${RTQL8_KINEMATIC_LIBRARY}/kinematicsd)
TARGET_LINK_LIBRARIES(manipulabilityTests debug ${RTQL8_KINEMATIC_LIBRARY}/collisiond)
TARGET_LINK_LIBRARIES(manipulabilityTests debug ${RTQL8_KINEMATIC_LIBRARY}/simulationd)
TARGET_LINK_LIBRARIES(manipulabilityTests debug rtql8Manipulability)

TARGET_LINK_LIBRARIES(manipulabilityTests optimized ${RTQL8_KINEMATIC_LIBRARY}/libticpp)
TARGET_LINK_LIBRARIES(manipulabilityTests optimized ${RTQL8_KINEMATIC_LIBRARY}/utils)
TARGET_LINK_LIBRARIES(manipulabilityTests optimized ${RTQL8_KINEMATIC_LIBRARY}/kinematics)
TARGET_LINK_LIBRARIES(manipulabilityTests optimized ${RTQL8_KINEMATIC_LIBRARY}/collision)
TARGET_LINK_LIBRARIES(manipulabilityTests optimized ${RTQL8_KINEMATIC_LIBRARY}/simulation)
TARGET_LINK_LIBRARIES(manipulabilityTests optimized rtql8Manipulability)