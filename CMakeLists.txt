cmake_minimum_required(VERSION 3.5)
project(adaptive_component)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unknown-pragmas)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

include_directories(include)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

set(COMPONENT_AMENT_DEPENDENCIES
  rclcpp
  rclcpp_components)

## Simple composition
add_library(adaptive_component SHARED
src/adaptive_component.cpp)
target_compile_definitions(adaptive_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(adaptive_component ${COMPONENT_AMENT_DEPENDENCIES})
rclcpp_components_register_nodes(adaptive_component "composition::AdaptiveComponent")
set(node_plugins "${node_plugins}composition::AdaptiveComponent;$<TARGET_FILE:adaptive_component>\n")

# install components
install(TARGETS
  adaptive_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
