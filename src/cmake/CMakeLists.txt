cmake_minimum_required(VERSION 2.6)
project(mgear_solvers)

set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/../../release/platforms")
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}")

set(
	SOURCE_FILES
    "../register.cpp"
    "../mgear_solvers.h"
    "../add10ScalarNode.cpp"
    "../curveCns.cpp"
    "../ikfk2Bone.cpp"
    "../interpolateMatrix.cpp"
    "../inverseRotOrder.cpp"
    "../linearInterpolate3DvectorNode.cpp"
    "../matrixConstraint.cpp"
    "../mulMatrix.cpp"
    "../percentageToU.cpp"
    "../rayCastPosition.cpp"
    "../rollSplineKine.cpp"
    "../slideCurve2.cpp"
    "../spinePointAt.cpp"
    "../springGravityNode.cpp"
    "../springNode.cpp"
    "../squashStretch2.cpp"
    "../trigonometryAngle.cpp"
    "../utils.cpp"
    "../uToPercentage.cpp"
    "../vertexPosition.cpp"
)

find_package(Maya REQUIRED)
find_package(OpenGL REQUIRED)

include_directories(${MAYA_INCLUDE_DIR} ${OPENGL_INCLUDE_DIR})
link_directories(${MAYA_LIBRARY_DIR})

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${MAYA_LIBRARIES} ${OPENGL_LIBRARIES})

MAYA_PLUGIN(${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} ${MAYA_TARGET_TYPE} DESTINATION 
    "${MAYA_VERSION}/${PLATFORM_PLUGIN_DIR}/x64/plug-ins")