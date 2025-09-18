cmake_minimum_required(VERSION 3.8)
project(laser_uav_managers)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(laser_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(laser_uav_lib REQUIRED)
find_package(laser_uav_planner REQUIRED)
find_package(laser_uav_controllers REQUIRED)

set(acados_include "$ENV{ACADOS_SOURCE_DIR}/include")
set(acados_lib "$ENV{ACADOS_SOURCE_DIR}/lib")
set(full_model_build ${laser_uav_controllers_DIR}/../../../include/scripts/quadrotor_ode/)

set(dependencies
Eigen3
rclcpp 
rclcpp_lifecycle 
rclcpp_components
sensor_msgs
nav_msgs 
std_srvs 
px4_msgs
laser_msgs
laser_uav_planner
laser_uav_controllers
)

find_library(laser_uav_lib laser_uav_lib)
find_library(laser_uav_planner laser_uav_planner)
find_library(laser_uav_controllers laser_uav_controllers)

link_directories(${full_model_build})

include_directories(
  include
  ${laser_uav_lib_INCLUDE_DIRS}
  ${full_model_build}
	${acados_include}
	${acados_include}/blasfeo/include/
	${acados_include}/hpipm/include/
	${acados_include}/acados/
	${acados_include}/qpOASES_e/
  ${EIGEN3_INCLUDE_DIR}
)

set(executable_name control_manager)

add_executable(${executable_name}
  src/control_manager_main.cpp
)

set(library_name ${PROJECT_NAME}_control_manager_core)

add_library(${library_name} SHARED
  src/control_manager_node.cpp
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

target_link_libraries(${executable_name}
  ${library_name}
  laser_uav_lib::filter
	${laser_uav_planner}
	${laser_uav_controllers}
  ${full_model_build}/libacados_ocp_solver_quadrotor_ode.so
	${acados_lib}/libacados.so
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

target_include_directories(${library_name} PRIVATE src/include)

rclcpp_components_register_nodes(${library_name} "laser_uav_managers::ControlManagerNode")

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
