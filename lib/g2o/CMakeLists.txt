cmake_minimum_required(VERSION 3.10)
project(g2o VERSION 1.0 LANGUAGES CXX)

configure_file(config.h.in ${CMAKE_CURRENT_SOURCE_DIR}/config.h)

find_package(Eigen3 3.1.0 REQUIRED)

add_definitions(-DUNIX)

# Include the subdirectories
add_library(
    g2o SHARED
        include/types/types_sba.h
        include/types/types_six_dof_expmap.h
        src/types/types_sba.cpp
        src/types/types_six_dof_expmap.cpp
        src/types/types_seven_dof_expmap.cpp
        include/types/types_seven_dof_expmap.h
        include/types/se3quat.h
        include/types/se3_ops.h
        include/types/se3_ops.hpp
    #core
        include/core/base_edge.h
        include/core/base_binary_edge.h
        src/core/hyper_graph_action.cpp
        include/core/base_binary_edge.hpp
        include/core/hyper_graph_action.h
        include/core/base_multi_edge.h
        src/core/hyper_graph.cpp
        include/core/base_multi_edge.hpp
        include/core/hyper_graph.h
        include/core/base_unary_edge.h
        include/core/linear_solver.h
        include/core/base_unary_edge.hpp
        src/core/marginal_covariance_cholesky.cpp
        include/core/base_vertex.h
        include/core/marginal_covariance_cholesky.h
        include/core/base_vertex.hpp
        src/core/matrix_structure.cpp
        src/core/batch_stats.cpp
        include/core/matrix_structure.h
        include/core/batch_stats.h
        include/core/openmp_mutex.h
        include/core/block_solver.h
        include/core/block_solver.hpp
        src/core/parameter.cpp
        include/core/parameter.h
        src/core/cache.cpp
        include/core/cache.h
        src/core/optimizable_graph.cpp
        include/core/optimizable_graph.h
        src/core/solver.cpp
        include/core/solver.h
        include/core/creators.h
        src/core/optimization_algorithm_factory.cpp
        src/core/estimate_propagator.cpp
        include/core/optimization_algorithm_factory.h
        include/core/estimate_propagator.h
        src/core/factory.cpp
        include/core/optimization_algorithm_property.h
        include/core/factory.h
        include/core/sparse_block_matrix.h
        src/core/sparse_optimizer.cpp
        include/core/sparse_block_matrix.hpp
        include/core/sparse_optimizer.h
        src/core/hyper_dijkstra.cpp
        include/core/hyper_dijkstra.h
        src/core/parameter_container.cpp
        include/core/parameter_container.h
        src/core/optimization_algorithm.cpp
        include/core/optimization_algorithm.h
        src/core/optimization_algorithm_with_hessian.cpp
        include/core/optimization_algorithm_with_hessian.h
        src/core/optimization_algorithm_levenberg.cpp
        include/core/optimization_algorithm_levenberg.h
        src/core/optimization_algorithm_gauss_newton.cpp
        include/core/optimization_algorithm_gauss_newton.h
        src/core/jacobian_workspace.cpp
        include/core/jacobian_workspace.h
        src/core/robust_kernel.cpp
        include/core/robust_kernel.h
        src/core/robust_kernel_factory.cpp
        include/core/robust_kernel_factory.h
        src/core/robust_kernel_impl.cpp
        include/core/robust_kernel_impl.h
    #stuff
        include/stuff/string_tools.h
        include/stuff/color_macros.h
        include/stuff/macros.h
        src/stuff/timeutil.cpp
        include/stuff/misc.h
        include/stuff/timeutil.h
        src/stuff/os_specific.c
        include/stuff/os_specific.h
        src/stuff/string_tools.cpp
        src/stuff/property.cpp
        include/stuff/property.h
)

add_library(G2O::G2O ALIAS g2o)

target_compile_features(g2o PRIVATE cxx_std_17)

target_compile_options(g2o PUBLIC -Wall -W -O3)

target_include_directories(
    g2o
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${EIGEN3_INCLUDE_DIR}
)

# Installation
include(GNUInstallDirs)
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/G2O)

install(
    TARGETS g2o
    EXPORT g2o-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

set_target_properties(g2o PROPERTIES EXPORT_NAME G2O)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(
    EXPORT g2o-targets
    FILE
        G2OTargets.cmake
    NAMESPACE
        G2O::
    DESTINATION
        ${INSTALL_CONFIGDIR}
)

include(CMakePackageConfigHelpers)

# Install to Build Dir
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/G2OConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

install(
    FILES
        ${CMAKE_CURRENT_LIST_DIR}/cmake/G2OConfig.cmake
    DESTINATION
        ${CMAKE_CURRENT_BINARY_DIR}
)

# Install FROM the Build Dir to the Install Dir

install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/G2OConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/G2OConfigVersion.cmake
    DESTINATION
    ${INSTALL_CONFIGDIR}
)

export(
    EXPORT g2o-targets
    FILE ${CMAKE_CURRENT_BINARY_DIR}/G2OTargets.cmake
    NAMESPACE G2O::
)

export(PACKAGE G2O)