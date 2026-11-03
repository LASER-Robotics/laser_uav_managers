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
find_package(mars_quadrotor_msgs REQUIRED)
find_package(super_planner REQUIRED)
find_package(pmm_planner REQUIRED)
find_package(Threads REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(laser_uav_controllers REQUIRED)
find_package(laser_uav_estimators REQUIRED)
find_package(laser_uav_lib REQUIRED)

set(acados_include "$ENV{ACADOS_SOURCE_DIR}/include")
set(acados_lib "$ENV{ACADOS_SOURCE_DIR}/lib")
set(full_model_build ${laser_uav_controllers_DIR}/../../../include/scripts/multirotor_ode/)

set(dependencies
    Eigen3
    rclcpp
    rclcpp_lifecycle
    rclcpp_components
    sensor_msgs
    nav_msgs
    std_srvs
    tf2
    tf2_ros
    laser_msgs
    laser_uav_controllers
    laser_uav_estimators
    laser_uav_lib
)

find_library(laser_uav_lib laser_uav_lib)
find_library(laser_uav_controllers laser_uav_controllers)
find_library(laser_uav_estimators laser_uav_estimators)

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

set(control_executable_name control_manager)

add_executable(${control_executable_name} src/control_manager_main.cpp)

set(library_name ${PROJECT_NAME}_control_manager_core)

add_library(${library_name} SHARED src/control_manager_node.cpp)

ament_target_dependencies(${control_executable_name} ${dependencies})

target_link_libraries(
  ${control_executable_name}
  ${library_name}
)

target_link_libraries(
  ${library_name}
  ${laser_uav_controllers}
  ${full_model_build}/libacados_ocp_solver_multirotor_ode.so
  ${acados_lib}/libacados.so
)

ament_target_dependencies(${library_name} ${dependencies})

target_include_directories(${library_name} PRIVATE src/include)

rclcpp_components_register_nodes(${library_name} "laser_uav_managers::ControlManagerNode")

set(planning_library_name ${PROJECT_NAME}_planning_manager_core)
set(planning_dependencies
  Eigen3 rclcpp rclcpp_lifecycle rclcpp_components geometry_msgs nav_msgs std_srvs
  laser_msgs pmm_planner mars_quadrotor_msgs super_planner)
add_library(${planning_library_name} SHARED src/planning_manager_node.cpp)
target_compile_definitions(${planning_library_name} PRIVATE USE_ROS2 FMT_HEADER_ONLY)
ament_target_dependencies(${planning_library_name} ${planning_dependencies})
target_link_libraries(${planning_library_name} Threads::Threads)
rclcpp_components_register_nodes(${planning_library_name} "laser_uav_managers::PlanningManagerNode")
add_executable(planning_manager src/planning_manager_main.cpp)
ament_target_dependencies(planning_manager ${planning_dependencies})
target_link_libraries(planning_manager ${planning_library_name})

set(estimation_library_name "estimation_manager_node")
set(estimation_executable_name "estimation_manager_main")

add_library(${estimation_library_name} SHARED src/estimation_manager_node.cpp)

ament_target_dependencies(${estimation_library_name} ${dependencies})

target_link_libraries(${estimation_library_name} ${laser_uav_estimators})

add_executable(${estimation_executable_name} src/estimation_manager_main.cpp)

ament_target_dependencies(${estimation_executable_name} ${dependencies})

target_link_libraries(${estimation_executable_name} ${estimation_library_name})

rclcpp_components_register_nodes(${estimation_library_name} "laser_uav_managers::EstimationManager")

install(
  TARGETS ${library_name} ${estimation_library_name} ${planning_library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS ${control_executable_name} ${estimation_executable_name} planning_manager
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/)

install(DIRECTORY launch params plotjuggler DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/refactor_plotjuggler_config.sh DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_planning_launch test/test_planning_launch.py
    APPEND_ENV PYTHONDONTWRITEBYTECODE=1)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_reference_horizons test/test_reference_horizons.cpp)
  target_compile_definitions(test_reference_horizons PRIVATE
    TEST_MANAGER_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/params/control_manager.yaml"
    TEST_UAV_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/params/laser_uavs/x500.yaml"
    TEST_PLANNER_CONFIG_PATH="${pmm_planner_DIR}/../params/x500.yaml"
    TEST_CONTROLLER_CONFIG_PATH="${laser_uav_controllers_DIR}/../params/nmpc_controller/x500.yaml")
  ament_target_dependencies(test_reference_horizons ${dependencies} ${planning_dependencies})
  target_link_libraries(test_reference_horizons ${library_name} ${planning_library_name})

  ament_add_gtest(test_estimation_path test/test_estimation_path.cpp)
  target_compile_definitions(test_estimation_path PRIVATE
    TEST_ESTIMATION_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/params/estimation_manager.yaml"
    TEST_UAV_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/params/laser_uavs/x500.yaml")
  ament_target_dependencies(test_estimation_path ${dependencies})
  target_link_libraries(test_estimation_path ${estimation_library_name})

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies} ${planning_dependencies})
ament_package()
