cmake_minimum_required(VERSION 3.8)
project(rm_auto_aim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  add_definitions("-DRM_DEBUG_MODE")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(rm_cam REQUIRED)
find_package(rm_util REQUIRED)
find_package(rm_filters REQUIRED)
find_package(rm_trajectory REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(OpenCV REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)


include_directories(include)

# build shared lib
add_library(${PROJECT_NAME} SHARED
  "src/detectors/armor_detector_svm.cpp"
  "src/detectors/auto_aim_algo.cpp"
  "src/auto_aim_node.cpp"
)

set(dependencies
  OpenCV
  rm_interfaces
  rm_cam
  rm_util
  rm_filters
  rm_trajectory
  rclcpp
  rclcpp_components
  Eigen3
  std_msgs
  tf2
  tf2_ros
  geometry_msgs
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies} eigen3_cmake_module)

add_executable(auto_aim_node "src/auto_aim_main.cpp")
target_link_libraries(auto_aim_node ${PROJECT_NAME})

rclcpp_components_register_nodes(${PROJECT_NAME} "rm_auto_aim::AutoAimNode")

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS auto_aim_node
  DESTINATION lib/${PROJECT_NAME}
)

# install launch resource config directory
install(
    DIRECTORY launch resource config
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
