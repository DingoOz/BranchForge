# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

BranchForge is an open-source development platform for designing, visualizing, testing, and debugging Behaviour Trees (BTs) for ROS2 robotics applications. Built with C++20 and Qt6 QML on Ubuntu.

## Build Commands

```bash
# Build (from project root)
mkdir -p build && cd build && cmake .. && make -j$(nproc)

# Run the application
./build/branchforge_enhanced

# Build with tests
cmake -DBUILD_TESTING=ON .. && make -j$(nproc)

# Run all tests
ctest

# Run a specific test
./build/tests/unit/test_behavior_tree_xml

# Run tests with verbose output
ctest --verbose
```

## Dependencies

```bash
# Qt6 (required)
sudo apt install -y qt6-base-dev qt6-declarative-dev qt6-quick3d-dev

# Build tools
sudo apt install -y cmake build-essential pkg-config

# Testing (optional)
sudo apt install -y libgtest-dev
```

## Architecture

### Application Startup Flow
1. `src/main.cpp` → Creates `Application` instance
2. `src/core/Application.cpp` → Initializes Qt, registers QML types, loads QML engine
3. `qml/main.qml` → Main window with three-panel SplitView layout
4. QML components communicate with C++ singletons (ROS2Interface, ProjectManager, BTSerializer)

### QML-C++ Bridge
C++ classes are exposed to QML via `qmlRegisterType` and `qmlRegisterSingletonType` in `Application::setupQmlTypes()`:
- **Singletons**: ROS2Interface, ProjectManager, BTSerializer, ChartDataManager
- **Types**: MainWindow, CodeGenOptions

### Conditional Compilation
The codebase supports systems with/without QML:
```cpp
#ifdef QT6_QML_AVAILABLE  // QML-based UI
#ifdef QT6_QUICK_AVAILABLE
#ifdef QT6_XML_AVAILABLE
#ifdef QT6_CHARTS_AVAILABLE
#ifdef HAVE_ROS2  // ROS2 integration
```

### Key Components

| Component | Purpose |
|-----------|---------|
| `src/core/Application.cpp` | Application bootstrap, QML type registration |
| `src/ui/MainWindow.cpp` | C++ backend for main window (exposed to QML) |
| `src/project/BTSerializer.cpp` | Converts QML editor state to BehaviorTreeXML |
| `src/project/CodeGenerator.cpp` | Generates C++20 ROS2 code from behavior trees |
| `src/project/BehaviorTreeXML.cpp` | BT XML parsing and validation |
| `qml/components/NodeEditor.qml` | Visual node editor with zoom/pan/connections |
| `qml/components/NodeLibraryPanel.qml` | Draggable node palette |
| `qml/components/PropertiesPanel.qml` | Node property editor |

### Data Flow: Visual Editor → Code Generation
1. User creates nodes in `NodeEditor.qml` (stored in `dynamicNodes` array)
2. User connects nodes (stored in `connections` array)
3. `getEditorState()` exports to QVariantMap format
4. `BTSerializer.convertToBehaviorTreeXML()` creates BehaviorTreeXML structure
5. `CodeGenerator.generate()` produces C++20 ROS2 code

### Test Structure
```
tests/
├── unit/
│   ├── core/test_application.cpp
│   └── project/
│       ├── test_behavior_tree_xml.cpp
│       ├── test_bt_serializer.cpp
│       ├── test_code_generator.cpp
│       └── test_project_manager.cpp
├── integration/
│   └── test_visual_to_code_pipeline.cpp
└── data/          # Test XML fixtures
```

## Code Style Notes

- Namespaces: `BranchForge::Core`, `BranchForge::UI`, `BranchForge::Project`, `BranchForge::ROS2`
- Member variables: `m_` prefix (e.g., `m_codeGenOptions`)
- Qt logging: Use `Q_LOGGING_CATEGORY` and `qCInfo`/`qCWarning`/`qCCritical`
- QML files use Qt Quick 2.15 imports

## important-instruction-reminders
Do what has been asked; nothing more, nothing less.
NEVER create files unless they're absolutely necessary for achieving your goal.
ALWAYS prefer editing an existing file to creating a new one.
NEVER proactively create documentation files (*.md) or README files. Only create documentation files if explicitly requested by the User.
