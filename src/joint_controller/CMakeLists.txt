cmake_minimum_required(VERSION 3.8)
project(joint_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

# Set CMAKE_PREFIX_PATH if needed
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

# Add the user library
add_subdirectory(src/IKSolver)

# Add the executable
add_executable(joint_controller src/joint_controller_node.cpp)
ament_target_dependencies(joint_controller
  rclcpp
  std_msgs
)
target_link_libraries(joint_controller
  ik_solver
)

# Install the executable
install(TARGETS joint_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install urdf files
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
