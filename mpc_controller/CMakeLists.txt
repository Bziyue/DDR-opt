cmake_minimum_required(VERSION 3.0.2)
project(mpc_controller)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++17 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

get_filename_component(DDR_OPT_REPO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
include("${DDR_OPT_REPO_DIR}/cmake/UseBundledOsqp.cmake")

find_package(catkin REQUIRED COMPONENTS
  carstatemsgs
  nav_msgs
  roscpp
  std_msgs
  visualization_msgs
  back_end
)

find_package(OsqpEigen QUIET)
find_package(cmake_modules REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

if(OsqpEigen_FOUND)
  get_filename_component(DDR_OPT_OSQP_PREFIX "${OsqpEigen_DIR}/../.." ABSOLUTE)
else()
  ddr_opt_use_bundled_osqp(
    REPO_DIR "${DDR_OPT_REPO_DIR}"
    PREFIX_VAR DDR_OPT_OSQP_PREFIX
  )
endif()

include_directories(
  /usr/local/include/osqp
  "${DDR_OPT_OSQP_PREFIX}/include"
  "${DDR_OPT_OSQP_PREFIX}/include/osqp"
)

# add_message_files(FILES
#   Polynome.msg
# )

# generate_messages(DEPENDENCIES
# 	std_msgs
#   geometry_msgs
# )

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES mpc_controller
 CATKIN_DEPENDS carstatemsgs message_generation nav_msgs roscpp std_msgs visualization_msgs back_end 
 DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


add_executable(mpc
  src/mpc_node.cpp
  src/mpc.cpp
)

add_library(mpc_controller 
  src/mpc_node.cpp
  src/mpc.cpp 
)

if(EXISTS "${DDR_OPT_OSQP_PREFIX}/lib")
  set_target_properties(mpc PROPERTIES
    BUILD_RPATH "${DDR_OPT_OSQP_PREFIX}/lib"
    INSTALL_RPATH "${DDR_OPT_OSQP_PREFIX}/lib"
  )
  set_target_properties(mpc_controller PROPERTIES
    BUILD_RPATH "${DDR_OPT_OSQP_PREFIX}/lib"
    INSTALL_RPATH "${DDR_OPT_OSQP_PREFIX}/lib"
  )
endif()

target_link_libraries(mpc
  OsqpEigen::OsqpEigen
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

target_link_libraries(mpc_controller
  OsqpEigen::OsqpEigen
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)
