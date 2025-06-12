cmake_minimum_required(VERSION 3.20)
project(branchforge VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find Qt6 packages
find_package(Qt6 REQUIRED COMPONENTS Core Widgets Concurrent Gui Qml Quick Xml)

qt6_standard_project_setup()

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Generate QRC file first
qt6_add_resources(QML_RESOURCES resources.qrc)

set(SOURCES
    src/main.cpp
    src/core/Application.cpp
    src/ui/MainWindow.cpp
    src/ros2/ROS2Interface.cpp
    src/ros2/ROS2TopicBrowser.cpp
    src/project/ProjectManager.cpp
    src/project/BehaviorTreeXML.cpp
    src/project/CodeGenerator.cpp
    src/project/BTSerializer.cpp
    src/monitoring/BTExecutionMonitor.cpp
    src/recording/DataRecorder.cpp
    src/nodes/AdvancedNodeSystem.cpp
    src/visualization/SensorDataPipeline.cpp
    ${QML_RESOURCES}
)

set(HEADERS
    include/core/Application.h
    include/ui/MainWindow.h
    include/ros2/ROS2Interface.h
    include/ros2/ROS2TopicBrowser.h
    include/project/ProjectManager.h
    include/project/BehaviorTreeXML.h
    include/project/CodeGenerator.h
    include/project/BTSerializer.h
    include/monitoring/BTExecutionMonitor.h
    include/recording/DataRecorder.h
    include/nodes/AdvancedNodeSystem.h
    include/visualization/SensorDataPipeline.h
)

# Process Qt6 MOC for headers
qt6_add_executable(branchforge_enhanced ${SOURCES} ${HEADERS})

# QML resources are now included in SOURCES above

# QML resources are automatically linked

target_link_libraries(branchforge_enhanced PRIVATE
    Qt6::Core
    Qt6::Widgets
    Qt6::Concurrent
    Qt6::Gui
    Qt6::Qml
    Qt6::Quick
    Qt6::Xml
)

target_compile_options(branchforge_enhanced PRIVATE
    -Wall -Wextra -Wpedantic
    -Wno-unused-parameter
)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_definitions(branchforge_enhanced PRIVATE DEBUG_BUILD)
endif()

# Testing configuration
option(BUILD_TESTING "Build tests" ON)

if(BUILD_TESTING)
    enable_testing()
    
    # Use system GTest
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(GTEST REQUIRED gtest>=1.14.0)
    pkg_check_modules(GTEST_MAIN REQUIRED gtest_main>=1.14.0)
    
    # Create imported targets for consistency
    if(NOT TARGET gtest)
        add_library(gtest SHARED IMPORTED)
        find_library(GTEST_LIB NAMES gtest PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu)
        set_target_properties(gtest PROPERTIES IMPORTED_LOCATION ${GTEST_LIB})
        target_include_directories(gtest INTERFACE ${GTEST_INCLUDE_DIRS})
    endif()
    
    if(NOT TARGET gtest_main)
        add_library(gtest_main SHARED IMPORTED)
        find_library(GTEST_MAIN_LIB NAMES gtest_main PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu)
        set_target_properties(gtest_main PROPERTIES IMPORTED_LOCATION ${GTEST_MAIN_LIB})
        target_include_directories(gtest_main INTERFACE ${GTEST_INCLUDE_DIRS})
    endif()
    
    # Test library with shared code
    add_library(branchforge_test_lib
        src/core/Application.cpp
        src/ui/MainWindow.cpp
        src/ros2/ROS2Interface.cpp
        src/ros2/ROS2TopicBrowser.cpp
        src/project/ProjectManager.cpp
        src/project/BehaviorTreeXML.cpp
        src/project/CodeGenerator.cpp
        src/project/BTSerializer.cpp
        src/monitoring/BTExecutionMonitor.cpp
        src/recording/DataRecorder.cpp
        src/nodes/AdvancedNodeSystem.cpp
        src/visualization/SensorDataPipeline.cpp
    )
    
    target_include_directories(branchforge_test_lib
        PUBLIC include
    )
    
    target_link_libraries(branchforge_test_lib
        Qt6::Core
        Qt6::Widgets
        Qt6::Concurrent
        Qt6::Gui
        Qt6::Qml
        Qt6::Quick
        Qt6::Xml
    )
    
    # Add test subdirectory
    add_subdirectory(tests)
endif()

install(TARGETS branchforge_enhanced
    DESTINATION bin
)