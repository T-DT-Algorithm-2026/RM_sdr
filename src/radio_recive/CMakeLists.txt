cmake_minimum_required(VERSION 3.8)
project(radio_recive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cppzmq REQUIRED)
find_package(radio_interface REQUIRED)

# include directories
include_directories(include)

# create shared library component
add_library(radio_recive_component SHARED src/radio.cpp src/crc.cpp)
target_include_directories(radio_recive_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(
  radio_recive_component
  PUBLIC
    cppzmq
    rclcpp::rclcpp
    rclcpp_components::component
    ${std_msgs_TARGETS}
    ${radio_interface_TARGETS}
)

# register component
rclcpp_components_register_node(radio_recive_component PLUGIN "radio::Receiver" EXECUTABLE radio_recive_node)

install(TARGETS
  radio_recive_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(radio_recive_component)
ament_export_dependencies(rclcpp rclcpp_components std_msgs radio_interface)

ament_package()
