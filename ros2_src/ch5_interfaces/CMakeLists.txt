cmake_minimum_required(VERSION 3.8)
project(ch5_interfaces)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
"srv/DriveInSquare.srv"
"srv/Takeoff.srv"
"srv/Land.srv"
"action/DriveSquareAction.action"  # Add the action file to the list of generated interfaces
"action/DriveCircle.action"
#DEPENDENCIES # This is where we would add dependencies if we had any besides the standard ROS 2 interfaces
)
if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
set(ament_cmake_copyright_FOUND TRUE)
set(ament_cmake_cpplint_FOUND TRUE)
ament_lint_auto_find_test_dependencies()
endif()
ament_package()