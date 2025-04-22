cmake_minimum_required(VERSION 3.8)
project(laser_uav_managers)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
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
find_package(laser_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(laser_uav_controllers REQUIRED)
find_package(mpc++ CONFIG REQUIRED)

set(dependencies
Eigen3
rclcpp 
rclcpp_lifecycle 
rclcpp_components
std_srvs 
nav_msgs
std_msgs
laser_msgs
geometry_msgs
laser_uav_controllers
mpc++
)

find_library(laser_uav_controllers laser_uav_controllers)

include_directories(
  include
  # ${full_model_build}
	# ${acados_include}
	# ${acados_include}/blasfeo/include/
	# ${acados_include}/hpipm/include/
	# ${acados_include}/acados/
	# ${acados_include}/qpOASES_e/
  ${mpc++_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# link_directories(${full_model_build})

add_executable(control_manager
  src/control_manager_main.cpp
)

add_library(control_manager_core SHARED
  src/control_manager_node.cpp
)

ament_target_dependencies(control_manager
  ${dependencies}
)

target_link_libraries(control_manager
  mpc++
  control_manager_core
  ${laser_uav_controllers}
  # ${full_model_build}/libacados_ocp_solver_uav.so
	# ${acados_lib}/libacados.so
)

ament_target_dependencies(control_manager_core
  ${dependencies}
)

target_include_directories(control_manager_core PRIVATE src/include)

rclcpp_components_register_nodes(control_manager_core "laser_uav_managers::ControlManagerNode")

install(TARGETS control_manager_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS control_manager 
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
