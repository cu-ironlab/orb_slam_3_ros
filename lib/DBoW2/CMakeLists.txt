cmake_minimum_required(VERSION 3.10)
project(DBoW2 VERSION 1.0.0 LANGUAGES CXX)

find_package(OpenCV 3 REQUIRED)

add_library(
    dbow2
    src/DBoW2/BowVector.cpp
    src/DBoW2/FeatureVector.cpp
    src/DBoW2/FORB.cpp
    src/DBoW2/ScoringObject.cpp
    src/DUtils/Random.cpp
    src/DUtils/Timestamp.cpp
)

add_library(DBoW2::DBoW2 ALIAS dbow2)

target_compile_features(dbow2 PRIVATE cxx_std_17)

target_compile_options(dbow2 PRIVATE -Wall -O3)

target_link_libraries(
    dbow2
    ${OpenCV_LIBS}
)

target_include_directories(
    dbow2
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${OpenCV_INCLUDE_DIRS}
)

# Installation
include(GNUInstallDirs)
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/DBoW2)

install(
    TARGETS dbow2
    EXPORT dbow2-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

set_target_properties(dbow2 PROPERTIES EXPORT_NAME DBoW2)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(
    EXPORT dbow2-targets
    FILE
        DBoW2Targets.cmake
    NAMESPACE
        DBoW2::
    DESTINATION
        ${INSTALL_CONFIGDIR}
)

include(CMakePackageConfigHelpers)

# Install to Build Dir
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/DBoW2ConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

install(
    FILES
        ${CMAKE_CURRENT_LIST_DIR}/cmake/DBoW2Config.cmake
    DESTINATION
        ${CMAKE_CURRENT_BINARY_DIR}
)

# Install FROM the Build Dir to the Install Dir

install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/DBoW2Config.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/DBoW2ConfigVersion.cmake
    DESTINATION
        ${INSTALL_CONFIGDIR}
)

export(
    EXPORT dbow2-targets
    FILE ${CMAKE_CURRENT_BINARY_DIR}/DBoW2Targets.cmake
    NAMESPACE DBoW2::
)

export(PACKAGE DBoW2)