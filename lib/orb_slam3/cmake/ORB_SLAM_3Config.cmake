get_filename_component(ORB_SLAM_3_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
include(CMakeFindDependencyMacro)

list(APPEND CMAKE_MODULE_PATH ${ORB_SLAM_3_CMAKE_DIR})

find_dependency(Pangolin)
find_dependency(G2O 1.0)
find_dependency(DBoW2 1.0)

if(NOT TARGET ORB_SLAM_3::ORB_SLAM_3)
    include("${ORB_SLAM_3_CMAKE_DIR}/ORB_SLAM_3Targets.cmake")
endif()

set(ORB_SLAM_3_LIBRARIES ORB_SLAM_3::ORB_SLAM_3)