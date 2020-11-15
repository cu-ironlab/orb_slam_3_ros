get_filename_component(G2O_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
include(CMakeFindDependencyMacro)

list(APPEND CMAKE_MODULE_PATH ${G2O_CMAKE_DIR})

find_dependency(Eigen3 3.1.0)

if(NOT TARGET G2O::G2O)
    include("${G2O_CMAKE_DIR}/G2OTargets.cmake")
endif()

set(G2O_LIBRARIES G2O::G2O)