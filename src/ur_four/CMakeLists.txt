cmake_minimum_required(VERSION 3.8)
project(ur_four)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)



add_executable(multi_thread_psort_orca scripts/multi_thread_psort_orca.cpp)
ament_target_dependencies(multi_thread_psort_orca
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  geometry_msgs
  tf2
  tf2_geometry_msgs
  nav_msgs
  control_msgs
)

add_executable(dec_rrt_baseline scripts/dec_rrt_baseline.cpp)
ament_target_dependencies(dec_rrt_baseline
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  geometry_msgs
  tf2
  tf2_geometry_msgs
  nav_msgs
  control_msgs
)

add_executable(baseline scripts/baseline.cpp)
ament_target_dependencies(baseline
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  geometry_msgs
  tf2
  tf2_geometry_msgs
  nav_msgs
  control_msgs
)

add_executable(centralized_baseline scripts/centralized_baseline.cpp)
ament_target_dependencies(centralized_baseline
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  geometry_msgs
  tf2
  tf2_geometry_msgs
  nav_msgs
  control_msgs
)

# target_link_libraries(multi_thread_psort_orca gz-transport14::core gz-msgs10::core)

# Link to matching gz targets 
target_link_libraries(multi_thread_psort_orca
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13
)

target_link_libraries(dec_rrt_baseline
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13
)

target_link_libraries(baseline
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13
)

target_link_libraries(centralized_baseline
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13
)


install(TARGETS
  multi_thread_psort_orca
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  dec_rrt_baseline
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  baseline
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  centralized_baseline
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY config launch meshes onrobot_sg scripts urdf worlds
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/gazebo_publisher.py
  scripts/wait_for_joint_states.py
  DESTINATION lib/${PROJECT_NAME}
)



# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
