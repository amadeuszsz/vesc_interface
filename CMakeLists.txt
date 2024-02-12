cmake_minimum_required(VERSION 3.14)
project(vesc_interface)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

set(VESC_INTERFACE_LIB_SRC
  src/vesc_interface.cpp
)

set(VESC_INTERFACE_LIB_HEADERS
  include/vesc_interface/vesc_interface.hpp
  include/vesc_interface/visibility_control.hpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${VESC_INTERFACE_LIB_SRC}
  ${VESC_INTERFACE_LIB_HEADERS}
)

set(VESC_INTERFACE_NODE_SRC
  src/vesc_interface_node.cpp
)

set(VESC_INTERFACE_NODE_HEADERS
  include/vesc_interface/vesc_interface_node.hpp
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  ${VESC_INTERFACE_NODE_SRC}
  ${VESC_INTERFACE_NODE_HEADERS}
)
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "vesc_interface::VescInterfaceNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
