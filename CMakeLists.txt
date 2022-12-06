cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

# clangd support 
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
execute_process(COMMAND "rm" "-f" "${CMAKE_SOURCE_DIR}/compile_commands.json")
execute_process(COMMAND "ln" "-s" "-r" "${CMAKE_BINARY_DIR}/compile_commands.json" "${CMAKE_SOURCE_DIR}/")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tutorial_interfaces REQUIRED) 

add_executable(node_pubs src/node_pubs.cpp)
ament_target_dependencies(node_pubs rclcpp tutorial_interfaces)
add_executable(node_subs src/node_subs.cpp)
ament_target_dependencies(node_subs rclcpp tutorial_interfaces)

install(TARGETS
  node_pubs
  node_subs
  DESTINATION lib/${PROJECT_NAME})

ament_package()