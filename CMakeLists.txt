cmake_minimum_required(VERSION 3.20)
project(branchforge VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find Qt6 packages
find_package(Qt6 REQUIRED COMPONENTS Core Widgets Concurrent Gui Qml Quick)

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
)

target_compile_options(branchforge_enhanced PRIVATE
    -Wall -Wextra -Wpedantic
    -Wno-unused-parameter
)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_definitions(branchforge_enhanced PRIVATE DEBUG_BUILD)
endif()

install(TARGETS branchforge_enhanced
    DESTINATION bin
)