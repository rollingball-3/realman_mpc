cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(ocs2_mobile_manipulator)


set(dependencies
  rclcpp
  visualization_msgs
  ocs2_core
  ocs2_ddp
  ocs2_mpc
  ocs2_robotic_tools
  ocs2_robotic_assets
  ocs2_pinocchio_interface
  ocs2_sphere_approximation
  ocs2_self_collision
  Boost
  pinocchio
  hpp-fcl
  nvblox_msgs
  std_msgs
  geometry_msgs
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(ocs2_robotic_assets REQUIRED)
find_package(ocs2_self_collision REQUIRED)
find_package(ocs2_sphere_approximation REQUIRED)
find_package(nvblox_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  log_setup
  log
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)
pkg_check_modules(hpp-fcl REQUIRED hpp-fcl)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# 确保 console_bridge 目录存在
find_package(console_bridge QUIET)
if(console_bridge_FOUND)
  message(STATUS "Found console_bridge: ${console_bridge_DIR}")
else()
  message(WARNING "console_bridge not found, but might be available in the system paths")
endif()

###########
## Build ##
###########

set(FLAGS
  ${OCS2_CXX_FLAGS}
  ${pinocchio_CFLAGS_OTHER}
  -Wno-ignored-attributes
  -Wno-invalid-partial-specialization   # to silence warning with unsupported Eigen Tensor
  -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
  -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
)

# Resolve for the package path at compile time.
configure_file (
  "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h.in"
  "${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/package_path.h" @ONLY
)

# Add directories for all targets
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
  ${hpp-fcl_INCLUDE_DIRS}
)

# 添加 pinocchio 和 hpp-fcl 的链接路径
link_directories(${pinocchio_LIBRARY_DIRS})
link_directories(${hpp-fcl_LIBRARY_DIRS})

# mobile maniulator interface library
add_library(${PROJECT_NAME}
  src/FactoryFunctions.cpp
  src/MobileManipulatorPreComputation.cpp
  src/MobileManipulatorPinocchioMapping.cpp
  src/constraint/EndEffectorConstraint.cpp
  src/dynamics/WheelBasedMobileManipulatorDynamics.cpp
  src/dynamics/DefaultManipulatorDynamics.cpp
  src/dynamics/FloatingArmManipulatorDynamics.cpp
  src/dynamics/FullyActuatedFloatingArmManipulatorDynamics.cpp
  src/MobileManipulatorInterface.cpp
  src/EsdfClientInterface.cpp
  src/constraint/ObstacleAvoidanceConstraint.cpp
  src/constraint/EmptyConstraint.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

# 显式添加 pinocchio 和 hpp-fcl 库到目标
target_link_libraries(${PROJECT_NAME}
  ${pinocchio_LIBRARIES}
  ${hpp-fcl_LIBRARIES}
)

add_executable(test_esdf_client test/test_esdf_client.cpp)
ament_target_dependencies(test_esdf_client
    ${dependencies}
)
target_link_libraries(test_esdf_client
    ${PROJECT_NAME}
    ${pinocchio_LIBRARIES}
    ${hpp-fcl_LIBRARIES}
)

install(
  TARGETS test_esdf_client
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

target_compile_options(${PROJECT_NAME} PUBLIC ${FLAGS})
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

####################
## Clang tooling ###
####################

find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling")
  add_clang_tooling(
    TARGETS ${PROJECT_NAME}
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
  )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

# 确保不导出不存在的路径
get_target_property(current_include_dirs ${PROJECT_NAME} INTERFACE_INCLUDE_DIRECTORIES)
set(filtered_include_dirs "")
if(current_include_dirs)
  foreach(include_dir ${current_include_dirs})
    if(EXISTS "${include_dir}")
      list(APPEND filtered_include_dirs "${include_dir}")
    else()
      message(WARNING "Skipping non-existent include directory: ${include_dir}")
    endif()
  endforeach()
  set_target_properties(${PROJECT_NAME} PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${filtered_include_dirs}")
endif()

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

#############
## Testing ##
#############
find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()
find_package(ament_cmake_gtest)

# Helper macro for adding target applications
macro(add_ocs2_test APP_NAME APP_SRC)
  ament_add_gtest(${APP_NAME}
    ${APP_SRC}
  )
  target_include_directories(${APP_NAME} PRIVATE
    ${PROJECT_BINARY_DIR}/include
  )
  ament_target_dependencies(${APP_NAME}
    ${dependencies}
  )
  target_link_libraries(${APP_NAME}
    ${PROJECT_NAME}
    ${pinocchio_LIBRARIES}
    ${hpp-fcl_LIBRARIES}
  )
endmacro()

add_ocs2_test(SelfCollisionTest test/testSelfCollision.cpp)
add_ocs2_test(EndEffectorConstraintTest test/testEndEffectorConstraint.cpp)
add_ocs2_test(DummyMobileManipulatorTest test/testDummyMobileManipulator.cpp)

ament_export_dependencies(${dependencies})  
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()