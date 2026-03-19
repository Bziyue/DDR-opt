if(DEFINED DDR_OPT_BUNDLED_OSQP_INCLUDED)
  return()
endif()
set(DDR_OPT_BUNDLED_OSQP_INCLUDED TRUE)

include(CMakeParseArguments)
include(ExternalProject)

function(ddr_opt_use_bundled_osqp)
  set(options "")
  set(oneValueArgs REPO_DIR PREFIX_VAR)
  cmake_parse_arguments(DDR_OPT "${options}" "${oneValueArgs}" "" ${ARGN})

  if(NOT DDR_OPT_REPO_DIR)
    message(FATAL_ERROR "ddr_opt_use_bundled_osqp requires REPO_DIR")
  endif()

  if(NOT DDR_OPT_PREFIX_VAR)
    message(FATAL_ERROR "ddr_opt_use_bundled_osqp requires PREFIX_VAR")
  endif()

  set(_osqp_source_dir "${DDR_OPT_REPO_DIR}/third_party/osqp")
  set(_osqp_eigen_source_dir "${DDR_OPT_REPO_DIR}/third_party/osqp-eigen")

  if(NOT EXISTS "${_osqp_source_dir}/CMakeLists.txt")
    message(FATAL_ERROR "Bundled OSQP source is missing at ${_osqp_source_dir}")
  endif()

  if(NOT EXISTS "${_osqp_eigen_source_dir}/CMakeLists.txt")
    message(FATAL_ERROR "Bundled osqp-eigen source is missing at ${_osqp_eigen_source_dir}")
  endif()

  if(CMAKE_BUILD_TYPE)
    set(_ddr_opt_build_type "${CMAKE_BUILD_TYPE}")
  else()
    set(_ddr_opt_build_type "Release")
  endif()

  set(_ddr_opt_vendor_root "${CMAKE_CURRENT_BINARY_DIR}/ddr_opt_vendor")
  set(_ddr_opt_vendor_prefix "${_ddr_opt_vendor_root}/install")
  set(_ddr_opt_vendor_include_dir "${_ddr_opt_vendor_prefix}/include")
  set(_ddr_opt_vendor_lib_dir "${_ddr_opt_vendor_prefix}/lib")

  file(MAKE_DIRECTORY "${_ddr_opt_vendor_include_dir}")
  file(MAKE_DIRECTORY "${_ddr_opt_vendor_include_dir}/osqp")
  file(MAKE_DIRECTORY "${_ddr_opt_vendor_lib_dir}")

  set(_ddr_opt_osqp_target "${PROJECT_NAME}_osqp_external")
  set(_ddr_opt_osqp_eigen_target "${PROJECT_NAME}_osqp_eigen_external")
  set(_ddr_opt_osqp_library "${_ddr_opt_vendor_lib_dir}/${CMAKE_SHARED_LIBRARY_PREFIX}osqp${CMAKE_SHARED_LIBRARY_SUFFIX}")
  set(_ddr_opt_osqp_eigen_library "${_ddr_opt_vendor_lib_dir}/${CMAKE_SHARED_LIBRARY_PREFIX}OsqpEigen${CMAKE_SHARED_LIBRARY_SUFFIX}")

  ExternalProject_Add(${_ddr_opt_osqp_target}
    PREFIX "${_ddr_opt_vendor_root}/osqp"
    SOURCE_DIR "${_osqp_source_dir}"
    BINARY_DIR "${_ddr_opt_vendor_root}/osqp/build"
    INSTALL_DIR "${_ddr_opt_vendor_prefix}"
    UPDATE_COMMAND ""
    TEST_COMMAND ""
    CMAKE_ARGS
      -DCMAKE_BUILD_TYPE=${_ddr_opt_build_type}
      -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
      -DBUILD_SHARED_LIBS=ON
      -DUNITTESTS=OFF
      -DPRINTING=OFF
      -DCMAKE_POSITION_INDEPENDENT_CODE=ON
    BUILD_BYPRODUCTS
      "${_ddr_opt_osqp_library}"
  )

  ExternalProject_Add(${_ddr_opt_osqp_eigen_target}
    PREFIX "${_ddr_opt_vendor_root}/osqp_eigen"
    SOURCE_DIR "${_osqp_eigen_source_dir}"
    BINARY_DIR "${_ddr_opt_vendor_root}/osqp_eigen/build"
    INSTALL_DIR "${_ddr_opt_vendor_prefix}"
    UPDATE_COMMAND ""
    TEST_COMMAND ""
    DEPENDS "${_ddr_opt_osqp_target}"
    CMAKE_ARGS
      -DCMAKE_BUILD_TYPE=${_ddr_opt_build_type}
      -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
      -DCMAKE_PREFIX_PATH=<INSTALL_DIR>
      -DBUILD_TESTING=OFF
      -DCMAKE_POSITION_INDEPENDENT_CODE=ON
    BUILD_BYPRODUCTS
      "${_ddr_opt_osqp_eigen_library}"
  )

  if(NOT TARGET ddr_opt_vendor_osqp_${PROJECT_NAME})
    add_library(ddr_opt_vendor_osqp_${PROJECT_NAME} SHARED IMPORTED GLOBAL)
    set_target_properties(ddr_opt_vendor_osqp_${PROJECT_NAME} PROPERTIES
      IMPORTED_LOCATION "${_ddr_opt_osqp_library}"
      INTERFACE_INCLUDE_DIRECTORIES "${_ddr_opt_vendor_include_dir};${_ddr_opt_vendor_include_dir}/osqp"
    )
    add_dependencies(ddr_opt_vendor_osqp_${PROJECT_NAME} "${_ddr_opt_osqp_target}")
  endif()

  if(NOT TARGET OsqpEigen::OsqpEigen)
    add_library(OsqpEigen::OsqpEigen SHARED IMPORTED GLOBAL)
  endif()

  set_target_properties(OsqpEigen::OsqpEigen PROPERTIES
    IMPORTED_LOCATION "${_ddr_opt_osqp_eigen_library}"
    INTERFACE_INCLUDE_DIRECTORIES "${_ddr_opt_vendor_include_dir}"
    INTERFACE_LINK_LIBRARIES "ddr_opt_vendor_osqp_${PROJECT_NAME}"
  )
  add_dependencies(OsqpEigen::OsqpEigen "${_ddr_opt_osqp_eigen_target}")

  set(${DDR_OPT_PREFIX_VAR} "${_ddr_opt_vendor_prefix}" PARENT_SCOPE)
endfunction()
