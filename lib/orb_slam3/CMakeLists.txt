cmake_minimum_required(VERSION 3.10)
project(orb_slam_3 VERSION 1.0.0 LANGUAGES CXX)

find_package(Pangolin REQUIRED)
find_package(OpenCV 3)

add_library(
    orb_slam_3 SHARED
    src/orb_slam3/System.cc
    src/orb_slam3/Tracking.cc
    src/orb_slam3/LocalMapping.cc
    src/orb_slam3/LoopClosing.cc
    src/orb_slam3/ORBextractor.cc
    src/orb_slam3/ORBmatcher.cc
    src/orb_slam3/FrameDrawer.cc
    src/orb_slam3/Converter.cc
    src/orb_slam3/MapPoint.cc
    src/orb_slam3/KeyFrame.cc
    src/orb_slam3/Atlas.cc
    src/orb_slam3/Map.cc
    src/orb_slam3/MapDrawer.cc
    src/orb_slam3/Optimizer.cc
    src/orb_slam3/PnPsolver.cc
    src/orb_slam3/Frame.cc
    src/orb_slam3/KeyFrameDatabase.cc
    src/orb_slam3/Sim3Solver.cc
    src/orb_slam3/Initializer.cc
    src/orb_slam3/Viewer.cc
    src/orb_slam3/ImuTypes.cc
    src/orb_slam3/G2oTypes.cc
    src/CameraModels/Pinhole.cpp
    src/CameraModels/KannalaBrandt8.cpp
    src/orb_slam3/OptimizableTypes.cpp
    src/orb_slam3/MLPnPsolver.cpp
    src/orb_slam3/TwoViewReconstruction.cc
)

add_dependencies(orb_slam_3 G2O::G2O DBoW2::DBoW2)

add_definitions(-DCOMPILEDWITHC11)

add_library(ORB_SLAM_3::ORB_SLAM_3 ALIAS orb_slam_3)

target_compile_features(orb_slam_3 PRIVATE cxx_std_11)

target_compile_options(orb_slam_3 PUBLIC -O3)

target_link_libraries(
    orb_slam_3
        G2O::G2O
        DBoW2::DBoW2
        ${Pangolin_LIBRARIES}
        ${OpenCV_LIBS}
        -lboost_serialization
        -lcrypto
)

target_include_directories(
    orb_slam_3
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${Pangolin_INCLUDE_DIRS}
)

# Installation
include(GNUInstallDirs)
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/ORB_SLAM_3)

install(
    TARGETS orb_slam_3
    EXPORT orb_slam_3-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

set_target_properties(orb_slam_3 PROPERTIES EXPORT_NAME ORB_SLAM_3)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(
    EXPORT orb_slam_3-targets
    FILE
        ORB_SLAM_3Targets.cmake
    NAMESPACE
        ORB_SLAM_3::
    DESTINATION
        ${INSTALL_CONFIGDIR}
)

include(CMakePackageConfigHelpers)

# Install to Build Dir
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/ORB_SLAM_3ConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

install(
    FILES
        ${CMAKE_CURRENT_LIST_DIR}/cmake/ORB_SLAM_3Config.cmake
    DESTINATION
        ${CMAKE_CURRENT_BINARY_DIR}
)

# Install FROM the Build Dir to the Install Dir

install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/ORB_SLAM_3Config.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/ORB_SLAM_3ConfigVersion.cmake
    DESTINATION
        ${INSTALL_CONFIGDIR}
)

export(
    EXPORT orb_slam_3-targets
    FILE ${CMAKE_CURRENT_BINARY_DIR}/ORB_SLAM_3Targets.cmake
    NAMESPACE DBoW2::
)

export(PACKAGE ORB_SLAM_3)