cmake_minimum_required(VERSION 3.8)
project(tb3_autonomy)
#Adds compiler warnings (-Wall, -Wextra, -Wpedantic) for stricter code checking.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED) # Core ROS 2 C++ library.
find_package(rclcpp_action REQUIRED) # ROS 2 actions (for navigation goals).
find_package(nav2_msgs REQUIRED) #Messages for the Nav2 stack.
find_package(behaviortree_cpp_v3 REQUIRED) # Behavior Trees framework.
find_package(yaml-cpp REQUIRED) #YAML file parsing.
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED) #Coordinate transformations.


#Copies the bt_xml and launch directories to the package's install path.
#This ensures that the behavior tree XML file (tree.xml) and launch files are available when the package is installed.
# Install directories
install(DIRECTORY
bt_xml launch config
DESTINATION share/${PROJECT_NAME}
)

#Defines navigation_behaviors.cpp as a source file for behaviors 
# Install C++ behaviors
set(BEHAVIOR_SOURCES
src/navigation_behaviors.cpp 
src/battery_behavior.cpp 
)
#Creates a list of dependencies required for the main executable.
set(TARGET_DEPENDS
    rclcpp
    behaviortree_cpp_v3
    yaml-cpp
    rclcpp_action
    nav2_msgs
    tf2
    tf2_geometry_msgs
)

#Includes the include/ directory (for header files).
#Compiles autonomy_node.cpp with navigation_behaviors.cpp.
#Links against the required ROS 2 and external libraries.
include_directories(include)
add_executable(autonomy_node src/autonomy_node.cpp ${BEHAVIOR_SOURCES})
ament_target_dependencies(autonomy_node ${TARGET_DEPENDS})
target_link_libraries(autonomy_node ${YAML_CPP_LIBRARIES})

#Installs autonomy_node in the ROS 2 package directory.
install(TARGETS
autonomy_node
DESTINATION lib/${PROJECT_NAME})

#Declares this as a ROS 2 package.
ament_package()
