cmake_minimum_required(VERSION 3.16)
project(path_follow_mpc VERSION 0.2.0 LANGUAGES CXX)

option(MPC_BUILD_ROS2 "Build the ROS 2 Humble adapter" ON)

add_library(mpc_core src/controller.cpp)
target_compile_features(mpc_core PUBLIC cxx_std_17)
target_include_directories(mpc_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(mpc_core PRIVATE -Wall -Wextra -Wpedantic)

if(MPC_BUILD_ROS2)
  enable_language(C)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rcl_interfaces REQUIRED)
  find_package(builtin_interfaces REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(std_srvs REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)
  find_package(ackermann_msgs REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  # 消息只在 ROS 层生成，关闭 MPC_BUILD_ROS2 后核心仍可在普通 CMake 工程使用。
  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/FourWheelSteeringCommand.idl"
    DEPENDENCIES std_msgs)
  rosidl_get_typesupport_target(mpc_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)
  add_executable(mpc_node src/mpc_node.cpp)
  target_compile_features(mpc_node PRIVATE cxx_std_17)
  target_compile_options(mpc_node PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(mpc_node mpc_core "${mpc_typesupport}")
  ament_target_dependencies(mpc_node rclcpp rcl_interfaces builtin_interfaces
    nav_msgs geometry_msgs std_msgs std_srvs ackermann_msgs
    tf2 tf2_ros tf2_geometry_msgs)
  install(TARGETS mpc_node DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
endif()

include(CTest)
if(BUILD_TESTING)
  add_executable(test_mpc test/test_controller.cpp)
  target_link_libraries(test_mpc mpc_core)
  target_compile_options(test_mpc PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME mpc_core_tests COMMAND test_mpc)
  if(MPC_BUILD_ROS2)
    add_executable(test_ros_adapter test/test_ros_adapter.cpp)
    target_compile_features(test_ros_adapter PRIVATE cxx_std_17)
    target_link_libraries(test_ros_adapter mpc_core "${mpc_typesupport}")
    target_compile_options(test_ros_adapter PRIVATE -Wall -Wextra -Wpedantic)
    ament_target_dependencies(test_ros_adapter rclcpp rcl_interfaces builtin_interfaces
      nav_msgs geometry_msgs std_msgs std_srvs ackermann_msgs tf2 tf2_ros tf2_geometry_msgs)
    add_test(NAME mpc_ros_adapter_tests COMMAND test_ros_adapter)
    set_tests_properties(mpc_ros_adapter_tests PROPERTIES
      TIMEOUT 60
      ENVIRONMENT
        "ROS_DOMAIN_ID=173;ROS_LOCALHOST_ONLY=1;LD_LIBRARY_PATH=${CMAKE_CURRENT_BINARY_DIR}:$ENV{LD_LIBRARY_PATH}")
  endif()
endif()

install(TARGETS mpc_core EXPORT mpc_coreTargets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(EXPORT mpc_coreTargets NAMESPACE path_follow_mpc::
  DESTINATION share/${PROJECT_NAME}/cmake)
if(MPC_BUILD_ROS2)
  ament_export_targets(mpc_coreTargets HAS_LIBRARY_TARGET)
  ament_export_include_directories(include)
  ament_export_dependencies(rosidl_default_runtime)
  ament_package()
else()
  include(CMakePackageConfigHelpers)
  configure_package_config_file(cmake/path_follow_mpcConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/path_follow_mpcConfig.cmake
    INSTALL_DESTINATION share/${PROJECT_NAME}/cmake)
  write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/path_follow_mpcConfigVersion.cmake
    VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/path_follow_mpcConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/path_follow_mpcConfigVersion.cmake
    DESTINATION share/${PROJECT_NAME}/cmake)
endif()
