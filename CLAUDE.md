# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

BranchForge is an open-source, comprehensive development platform for designing, visualizing, testing, and debugging Behaviour Trees (BTs) specifically tailored for ROS2 robotics applications. Built on modern C++20/Qt6 architecture with Ubuntu-first design.

## Technology Stack

- **Language**: Modern C++20 throughout the entire codebase
- **UI Framework**: Qt6.4+ with QML and Qt Quick for hardware-accelerated interfaces
- **Target Platform**: Ubuntu 22.04 LTS and newer (primary focus)
- **Build System**: CMake 3.20+ with C++20 module support
- **ROS2 Integration**: Native rclcpp integration with Humble, Iron, Jazzy, and Rolling
- **Graphics**: Qt6.4+ Quick with OpenGL/Vulkan backend for visualization
- **Extensions**: Dual Python/C++ hot-reload system for community development

## Essential Commands

### Build System
```bash
# Basic build (from project root)
mkdir build && cd build
cmake ..
make -j$(nproc)

# Clean rebuild
rm -rf build && mkdir build && cd build
cmake .. && make -j$(nproc)

# Run the application
./branchforge_enhanced
```

### Testing
```bash
# Build with tests enabled (default)
cmake -DBUILD_TESTING=ON ..
make -j$(nproc)

# Run all tests
ctest

# Run specific test
./test_behavior_tree_xml

# Run tests with verbose output
ctest --verbose
```

### Development Dependencies
```bash
# Install Qt6 dependencies on Ubuntu
sudo apt install -y qt6-base-dev qt6-declarative-dev qt6-quick3d-dev

# Install build tools
sudo apt install -y cmake build-essential pkg-config

# Install testing framework
sudo apt install -y libgtest-dev
```

## Architecture Components

### Core Application Structure
- **`src/core/Application.cpp`**: Main application class with conditional QML/Qt6 support
- **`src/ui/MainWindow.cpp`**: Primary UI controller bridging C++ and QML
- **`qml/main.qml`**: Main QML interface with SplitView layout for panels
- **`qml/components/`**: Modular QML components for each UI panel

### Key Design Patterns

#### Conditional Compilation
The codebase uses extensive conditional compilation to support different Qt6 configurations:
```cpp
#ifdef QT6_QML_AVAILABLE
    // QML-based UI code
#else
    // Fallback widget-based UI
#endif
```

#### QML-C++ Integration
- C++ classes are registered with QML using `qmlRegisterType` and `qmlRegisterSingletonType`
- Singletons are used for cross-component communication (ROS2Interface, ProjectManager)
- Properties and signals enable bidirectional communication between QML and C++

#### Resource Management
- QML files are embedded using Qt's resource system (`resources.qrc`)
- Conditional resource compilation based on Qt6 component availability
- Graceful fallback when QML components are not available

### UI Architecture
- **Three-panel layout**: Node Library (left), Node Editor (center), Properties (right)
- **Panel visibility**: Controlled via boolean properties with View menu integration
- **Component isolation**: Each panel is a self-contained QML component
- **State persistence**: UI layouts and panel visibility saved/restored

### Project Structure
```
include/
├── core/           # Application framework
├── ui/             # User interface components
├── project/        # Project management and serialization
├── nodes/          # Behavior tree node system
├── monitoring/     # Runtime monitoring and debugging
├── recording/      # Data recording and playback
├── ros2/           # ROS2 integration layer
└── visualization/  # 3D visualization and sensor data

qml/
├── main.qml        # Main application window
└── components/     # Reusable UI components
```

## Development Phases

The project is structured into 4 main phases:

1. **Phase 1 (Months 1-3)**: Foundation - Basic BT Visual Editor, Qt6.4+ UI Framework, ROS2 Integration Layer, C++20 Code Generation
2. **Phase 2 (Months 4-6)**: Core Features - Integrated 3D Visualization Engine, Real-time BT Monitoring, Data Recording/Playback, Advanced Node System
3. **Phase 3 (Months 7-9)**: Advanced Integration & Testing - Automated Testing Framework, Extension System, Performance Analytics, Simulation Integration
4. **Phase 4 (Months 10-12)**: Intelligence & Optimization - AI-Powered BT Analysis, Advanced Collaboration Features, Production Deployment Tools, Community Platform

## Key Design Principles

- **No External Dependencies**: Built-in 3D visualization eliminates RViz2 complexity
- **Hot-Reload Development**: Both Python and C++ extensions support live reloading
- **Professional Performance**: Enterprise-grade reliability with modern C++20 architecture
- **Community-Driven**: Open source with comprehensive extension API
- **All-in-One Solution**: Design, visualize, test, and optimize in a single application

## important-instruction-reminders
Do what has been asked; nothing more, nothing less.
NEVER create files unless they're absolutely necessary for achieving your goal.
ALWAYS prefer editing an existing file to creating a new one.
NEVER proactively create documentation files (*.md) or README files. Only create documentation files if explicitly requested by the User.