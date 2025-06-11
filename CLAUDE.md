# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

BranchForge is an open-source, comprehensive development platform for designing, visualizing, testing, and debugging Behaviour Trees (BTs) specifically tailored for ROS2 robotics applications. Built on modern C++20/Qt6 architecture with Ubuntu-first design.

## Technology Stack

- **Language**: Modern C++20 throughout the entire codebase
- **UI Framework**: Qt6 with QML and Qt Quick 3D for hardware-accelerated interfaces
- **Target Platform**: Ubuntu 22.04 LTS and newer (primary focus)
- **Build System**: CMake 3.20+ with C++20 module support
- **ROS2 Integration**: Native rclcpp integration with Humble, Iron, Jazzy, and Rolling
- **Graphics**: Qt6 Quick 3D with OpenGL/Vulkan backend for 3D visualization
- **Extensions**: Dual Python/C++ hot-reload system for community development

## Architecture Components

### Frontend (C++20/Qt6 Application)
- Qt6 Quick 3D for hardware-accelerated 3D rendering
- Modular docking system with persistent workspace layouts
- Custom BT editor with drag-and-drop node creation
- Integrated 3D visualization engine (replacing RViz2 dependency)
- Real-time performance monitoring and debugging interfaces

### Backend Services
- Modern C++20 abstraction layer with concepts and ranges
- High-performance ROS2 communication using rclcpp with coroutines
- Native URDF/SDF parser for robot model loading
- Optimized sensor data pipeline for point clouds, images, and maps
- SQLite database with C++20 wrappers for project and session management

### Extension System
- Dynamic C++ plugin loading with automatic compilation
- Embedded Python interpreter with module hot-swapping
- Unified extension API supporting both languages
- File watcher system for automatic reload during development
- Template generators for rapid extension scaffolding

## Development Standards

### Code Quality
- **Modern C++20**: Use concepts, ranges, coroutines, and modules where appropriate
- **Static Analysis**: Clang-tidy and cppcheck integration in CI pipeline
- **Code Coverage**: Minimum 80% coverage for core functionality
- **Documentation**: Doxygen for C++ API, Sphinx for user documentation
- **Security**: Regular security audits for extension system and data handling

### Testing Strategy
- **Unit Tests**: C++20 code with Google Test framework
- **Integration Tests**: ROS2 integration with automated robot simulations
- **UI Tests**: Qt6 GUI testing with automated interaction simulation
- **Performance Tests**: Benchmarking with large BT projects and datasets
- **Extension Tests**: Validation of Python and C++ extension loading/reloading

### Platform Support
- **Ubuntu Versions**: 22.04 LTS, 24.04 LTS, and rolling releases
- **ROS2 Distributions**: Humble, Iron, Jazzy, and Rolling
- **Hardware Configurations**: Various GPU configurations for 3D rendering
- **Robot Platforms**: Testing with TurtleBot, Spot, and industrial manipulators

## Development Phases

The project is structured into 4 main phases:

1. **Phase 1 (Months 1-3)**: Foundation - Basic BT Visual Editor, Qt6 UI Framework, ROS2 Integration Layer, C++20 Code Generation
2. **Phase 2 (Months 4-6)**: Core Features - Integrated 3D Visualization Engine, Real-time BT Monitoring, Data Recording/Playback, Advanced Node System
3. **Phase 3 (Months 7-9)**: Advanced Integration & Testing - Automated Testing Framework, Extension System, Performance Analytics, Simulation Integration
4. **Phase 4 (Months 10-12)**: Intelligence & Optimization - AI-Powered BT Analysis, Advanced Collaboration Features, Production Deployment Tools, Community Platform

## Key Design Principles

- **No External Dependencies**: Built-in 3D visualization eliminates RViz2 complexity
- **Hot-Reload Development**: Both Python and C++ extensions support live reloading
- **Professional Performance**: Enterprise-grade reliability with modern C++20 architecture
- **Community-Driven**: Open source with comprehensive extension API
- **All-in-One Solution**: Design, visualize, test, and optimize in a single application