# BranchForge Software Development Plan

## Project Overview

**BranchForge** is an open-source, comprehensive development platform for designing, visualizing, testing, and debugging Behaviour Trees (BTs) specifically tailored for ROS2 robotics applications. Built on modern C++20/Qt6 architecture with Ubuntu-first design, BranchForge integrates visual BT design, 3D robot visualization, automated testing, and AI-powered optimization into a single, cohesive development environment.

## Technical Architecture

### Core Technology Stack
- **Language**: Modern C++20 throughout the entire codebase
- **UI Framework**: Qt6 with QML and Qt Quick 3D for hardware-accelerated interfaces
- **Target Platform**: Ubuntu 22.04 LTS and newer (primary focus)
- **Build System**: CMake 3.20+ with C++20 module support
- **ROS2 Integration**: Native rclcpp integration with Humble, Iron, Jazzy, and Rolling
- **Graphics**: Qt6 Quick 3D with OpenGL/Vulkan backend for 3D visualization
- **Extensions**: Dual Python/C++ hot-reload system for community development

### Architecture Components

#### Frontend (C++20/Qt6 Application)
- Qt6 Quick 3D for hardware-accelerated 3D rendering
- Modular docking system with persistent workspace layouts
- Custom BT editor with drag-and-drop node creation
- Integrated 3D visualization engine (replacing RViz2 dependency)
- Real-time performance monitoring and debugging interfaces

#### Backend Services
- Modern C++20 abstraction layer with concepts and ranges
- High-performance ROS2 communication using rclcpp with coroutines
- Native URDF/SDF parser for robot model loading
- Optimized sensor data pipeline for point clouds, images, and maps
- SQLite database with C++20 wrappers for project and session management

#### Extension System
- Dynamic C++ plugin loading with automatic compilation
- Embedded Python interpreter with module hot-swapping
- Unified extension API supporting both languages
- File watcher system for automatic reload during development
- Template generators for rapid extension scaffolding

## Development Phases

### Phase 1: Foundation (Months 1-3)
**Core Platform Development**

#### Deliverables:
- **Basic BT Visual Editor**
  - Drag-and-drop node creation and connection
  - Standard BT node types (Sequence, Selector, Parallel, Decorator, Action, Condition)
  - Node parameter configuration with type validation
  - Tree structure validation and error detection
  - XML import/export for BT definitions

- **Qt6 Modular UI Framework**
  - Advanced docking system with draggable panels
  - Persistent workspace layouts with save/restore
  - Core UI modules: Node Library, Properties Inspector, Project Explorer
  - Theme system with dark/light mode support

- **ROS2 Integration Layer**
  - Basic ROS2 node discovery and topic browsing
  - Message type introspection and validation
  - Topic subscription and publishing capabilities
  - Integration with BehaviorTree.CPP library

- **C++20 Code Generation**
  - Template-based code generation for modern C++20
  - CMakeLists.txt and package.xml generation
  - Basic project structure creation

#### Technical Milestones:
- [ ] Qt6 application launches with modular interface
- [ ] Basic BT can be created, edited, and saved
- [ ] ROS2 topics can be browsed and subscribed
- [ ] Generated C++20 code compiles and runs
- [ ] Core UI modules functional with docking

### Phase 2: Core Features (Months 4-6)
**Visualization and Debugging**

#### Deliverables:
- **Integrated 3D Visualization Engine**
  - URDF/SDF robot model loading and display
  - Real-time sensor data visualization (LiDAR, cameras, IMU)
  - Occupancy grid and costmap rendering
  - Interactive 3D scene with camera controls
  - Multi-robot support for coordinated behaviors

- **Real-time BT Monitoring**
  - Live BT execution state visualization
  - Node execution timing and performance metrics
  - Blackboard variable monitoring and inspection
  - Execution timeline with scrubbing and replay

- **Data Recording and Playback**
  - Native MCAP/ROS bag file support
  - Synchronized recording of BT execution and sensor data
  - Time-scrubbing interface for debugging sessions
  - Session annotation and bookmarking system

- **Advanced Node System**
  - ROS2 action client/server node templates
  - Service call node implementations
  - Topic publisher/subscriber node types
  - Custom node creation wizard

#### Technical Milestones:
- [ ] 3D robot model displays correctly with joint state updates
- [ ] Live sensor data (point clouds, images) renders smoothly
- [ ] BT execution state visible in real-time
- [ ] MCAP files can be recorded and played back
- [ ] ROS2 action nodes functional in BT execution

### Phase 3: Advanced Integration & Testing (Months 7-9)
**Automated Testing and Optimization**

#### Deliverables:
- **Automated Testing Framework**
  - Goal-based test scenario definition
  - Simulation test harness with Gazebo integration
  - Real-world testing with safety monitoring
  - Automated success/failure detection
  - Regression testing suite for BT modifications

- **Extension System Implementation**
  - Dynamic C++ plugin loading with hot-reload
  - Embedded Python interpreter with module reloading
  - Extension template generators and documentation
  - File watcher system for development workflow
  - Extension packaging and distribution system

- **Performance Analytics**
  - BT execution profiling and bottleneck detection
  - Resource usage monitoring and optimization hints
  - Statistical analysis of BT performance over time
  - Comparative analysis between different BT strategies

- **Simulation Integration**
  - Native Gazebo world launching and management
  - Automated test scenario generation
  - Hardware-in-the-loop testing support
  - Mock node generation for testing without hardware

#### Technical Milestones:
- [ ] Automated test can run and determine pass/fail
- [ ] C++ extension can be compiled and hot-reloaded
- [ ] Python extension can be modified and reloaded instantly
- [ ] Performance profiler identifies BT bottlenecks
- [ ] Gazebo simulation integrates with BT testing

### Phase 4: Intelligence & Optimization (Months 10-12)
**AI-Powered Features and Polish**

#### Deliverables:
- **AI-Powered BT Analysis**
  - Machine learning models for BT optimization suggestions
  - Pattern recognition from successful BT implementations
  - Automatic bottleneck detection and resolution recommendations
  - Failure mode analysis with suggested improvements
  - Performance prediction for proposed BT modifications

- **Advanced Collaboration Features**
  - Team extension sharing and private repositories
  - Collaborative debugging sessions
  - BT review system with approval workflows
  - Knowledge base of successful BT patterns
  - Cloud synchronization for layouts and preferences

- **Production Deployment Tools**
  - Containerized BT deployment generation
  - CI/CD pipeline integration
  - Production monitoring and alerting
  - Performance benchmarking across robot fleets
  - Automatic documentation generation

- **Community Platform**
  - Extension marketplace with rating system
  - Template library for common robotics patterns
  - Tutorial system with interactive guides
  - Community forum integration
  - Contribution recognition system

#### Technical Milestones:
- [ ] AI system provides meaningful BT improvement suggestions
- [ ] Team collaboration features functional
- [ ] Extension marketplace allows browsing and installation
- [ ] Production deployment generates working containers
- [ ] Documentation system generates comprehensive guides

## Quality Assurance & Testing

### Automated Testing Strategy
- **Unit Tests**: C++20 code with Google Test framework
- **Integration Tests**: ROS2 integration with automated robot simulations
- **UI Tests**: Qt6 GUI testing with automated interaction simulation
- **Performance Tests**: Benchmarking with large BT projects and datasets
- **Extension Tests**: Validation of Python and C++ extension loading/reloading

### Code Quality Standards
- **Modern C++20**: Use concepts, ranges, coroutines, and modules where appropriate
- **Static Analysis**: Clang-tidy and cppcheck integration in CI pipeline
- **Code Coverage**: Minimum 80% coverage for core functionality
- **Documentation**: Doxygen for C++ API, Sphinx for user documentation
- **Security**: Regular security audits for extension system and data handling

### Platform Testing
- **Ubuntu Versions**: 22.04 LTS, 24.04 LTS, and rolling releases
- **ROS2 Distributions**: Humble, Iron, Jazzy, and Rolling
- **Hardware Configurations**: Various GPU configurations for 3D rendering
- **Robot Platforms**: Testing with TurtleBot, Spot, and industrial manipulators

## Risk Mitigation

### Technical Risks
- **ROS2 Evolution**: Modular architecture allows easy adaptation to new ROS2 versions
- **Qt6 Dependencies**: Minimal external dependencies, most functionality built on Qt6 core
- **Performance Bottlenecks**: Continuous profiling and optimization from early development
- **Extension Security**: Sandboxed execution environment for community extensions

### Development Risks
- **Scope Creep**: Well-defined phase deliverables with clear acceptance criteria
- **Community Adoption**: Early alpha releases to gather feedback and build community
- **Resource Management**: Open source development model with clear contribution guidelines
- **Competition**: Focus on unique value proposition of integrated BT development

## Success Criteria

### Phase 1 Success Metrics
- Basic BT editor functional for simple tree creation
- ROS2 integration working with standard message types
- Generated C++20 code compiles and executes correctly
- UI framework demonstrates professional quality interface

### Phase 2 Success Metrics
- 3D visualization renders robot and sensor data smoothly (60+ FPS)
- Real-time BT debugging provides actionable insights
- Data recording/playback works reliably with large datasets
- Advanced nodes integrate seamlessly with ROS2 ecosystem

### Phase 3 Success Metrics
- Automated testing framework can validate BT correctness
- Extension system supports community development workflows
- Performance analytics identify real optimization opportunities
- Simulation integration enables comprehensive testing

### Phase 4 Success Metrics
- AI suggestions demonstrably improve BT performance
- Community platform shows active engagement and contributions
- Production deployment tools support real-world robotics projects
- Documentation and tutorials enable rapid user onboarding

## Resource Requirements

### Development Team
- **Lead Architect**: C++20/Qt6 expertise, robotics background
- **ROS2 Integration Specialist**: Deep ROS2 knowledge, behavior trees experience
- **3D Graphics Developer**: Qt6 Quick 3D, OpenGL/Vulkan experience
- **AI/ML Engineer**: Machine learning for robotics applications
- **Community Manager**: Open source project management, documentation

### Infrastructure
- **Development Environment**: Ubuntu-based CI/CD with multiple ROS2 versions
- **Testing Hardware**: Various robot platforms for integration testing
- **Cloud Infrastructure**: Extension marketplace hosting and community platform
- **Documentation Platform**: Comprehensive guides, tutorials, and API references

### Timeline Summary
- **Months 1-3**: Core platform and basic functionality
- **Months 4-6**: Visualization, debugging, and data management
- **Months 7-9**: Testing framework and extension system
- **Months 10-12**: AI features and community platform
- **Ongoing**: Community support, maintenance, and enhancement


## Additional ideas during development
- Have a dockable panel that renders the map topic and the robot's position in the map
- Allow an option to overlay the lidar scan topic on the map 
- Highlight which node is currently being used by the robot (in simulation or in real life)
- have a smooth averaged highlight fucntion that shows the recent history of which nodes and branches have been used by the robot (15 second maxiumum)
- have a window that scrolls through the last nodes used or other relevant info from the ROS2 robot's state
- have dockable panel that displays how confused the robot is, as measured by how close to retry limits it is.

- allwo for a any tracked variable reported on a topic to be plotted over time in a chart
- allow for any panel ot be docked in any configuration
- ender urdf models in the map view
- render 3d point clouds and maps

## URDF Model Rendering Implementation Strategy

### Current Visualization System State
The visualization system has a solid architecture foundation but lacks Qt6 Quick3D integration. The `Visualization3DEngine` interface exists in `include/visualization/SensorDataPipeline.h` but isn't implemented with actual 3D rendering capabilities.

### URDF Rendering Implementation Approach

#### 1. **Add Required Dependencies**
Update CMakeLists.txt to include Qt6 Quick3D support:
```cmake
find_package(Qt6 REQUIRED COMPONENTS Quick3D Quick3DAssetImport)
```

#### 2. **URDF Parser Integration**
Extend the existing `SensorDataPipeline` with URDF capabilities:
- Parse URDF XML for robot geometry and joints using existing ROS2 integration
- Convert mesh references (.dae, .stl, .obj) to Qt6 Quick3D compatible formats
- Handle coordinate frame transformations between ROS2 (right-handed Z-up) and Qt6 (left-handed Y-up)
- Implement joint state updates from ROS2 `/joint_states` topic

#### 3. **3D Scene Architecture**
Transform the current 2D `LidarScanPanel.qml` into a full 3D viewer:
```qml
import QtQuick3D
View3D {
    // Robot model rendering with articulated joints
    Model { 
        source: "robot_meshes.mesh"
        materials: [ robotMaterial ]
    }
    // Point cloud rendering with GPU instancing
    Instancing { 
        instanceCount: pointCloudData.count
        // Custom point cloud shader for performance
    }
    // Map/occupancy grid as textured plane
    Model { 
        source: "#Rectangle"
        materials: [ occupancyGridMaterial ]
    }
    // Coordinate frame visualization
    Node { /* TF tree rendering */ }
}
```

#### 4. **Rendering Pipeline Components**
- **Robot Model**: Load URDF meshes as Qt6 Quick3D Models with proper PBR materials
- **LiDAR Points**: Custom instanced rendering for performance with millions of points using GPU compute shaders
- **Map Data**: Occupancy grid as textured plane/heightmap with dynamic updates
- **Coordinate Frames**: Interactive TF tree visualization with frame selection
- **Sensor Overlays**: Camera image projection, range sensor visualization
- **Joint State Animation**: Real-time joint position updates from ROS2 topics

#### 5. **Performance Optimization**
- **Hardware Acceleration**: Leverage Qt6 Quick3D's modern rendering pipeline with OpenGL/Vulkan backend
- **LOD System**: Level-of-detail for complex robot meshes based on camera distance
- **Frustum Culling**: Cull non-visible geometry for large datasets
- **Asynchronous Loading**: Non-blocking mesh and texture loading to prevent UI freezing
- **Point Cloud Optimization**: Spatial indexing and adaptive point size for smooth rendering
- **Memory Management**: Efficient buffer management for streaming sensor data

#### 6. **Integration Points**
- **Existing Architecture**: Build on the well-designed `SensorDataPipeline` and `Visualization3DEngine` interfaces
- **ROS2Bridge**: Extend `src/ros2/ROS2Interface.cpp` for URDF parameter server integration
- **UI Integration**: Enhance `qml/components/LidarScanPanel.qml` with 3D capabilities
- **Data Synchronization**: Use existing `SensorDataSynchronizer` for multi-modal data alignment

#### 7. **Development Priority**
1. **Phase 2.1**: Add Qt6 Quick3D dependencies and basic 3D scene setup
2. **Phase 2.2**: Implement URDF parser and basic robot model loading
3. **Phase 2.3**: Add point cloud rendering with performance optimization
4. **Phase 2.4**: Integrate occupancy grid and map rendering
5. **Phase 2.5**: Add interactive camera controls and coordinate frame visualization

The key advantage is leveraging Qt6 Quick3D's modern rendering pipeline while maintaining the existing well-designed data synchronization architecture from `SensorDataPipeline.h`.
- plot any actuator position on a chart
- mcap file format support
- team sharing
- Behaviour Trees are predominately defined in XML
