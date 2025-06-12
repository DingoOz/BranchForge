cmake_minimum_required(VERSION 3.20)
project(MyBehaviorTree VERSION 1.0.0 LANGUAGES CXX)

# Set C++20 standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)

# Add executable
add_executable(MyBehaviorTree_node main.cpp)

# Include directories
target_include_directories(MyBehaviorTree_node
    PRIVATE include
)

# Link libraries
ament_target_dependencies(MyBehaviorTree_node
    rclcpp
    behaviortree_cpp
)

# Install
install(TARGETS MyBehaviorTree_node
    DESTINATION lib/branchforge_generated
)

install(DIRECTORY launch
    DESTINATION share/branchforge_generated
)

# Testing
if(ON AND BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)
    
    ament_lint_auto_find_test_dependencies()
    
    ament_add_gtest(MyBehaviorTree_test test/test_behavior_tree.cpp)
    target_include_directories(MyBehaviorTree_test PRIVATE include)
    ament_target_dependencies(MyBehaviorTree_test rclcpp behaviortree_cpp)
endif()

ament_package()
