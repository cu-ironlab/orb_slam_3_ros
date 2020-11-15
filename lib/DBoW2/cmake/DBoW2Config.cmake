get_filename_component(DBOW2_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
include(CMakeFindDependencyMacro)

list(APPEND CMAKE_MODULE_PATH ${DBOW2_CMAKE_DIR})

find_dependency(OpenCV 3)

if(NOT TARGET DBoW2::DBoW2)
    include("${DBOW2_CMAKE_DIR}/DBoW2Targets.cmake")
endif()

set(DBOW2_LIBRARIES DBoW2::DBoW2)