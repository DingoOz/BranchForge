# BranchForge CI/CD Workflows

This directory contains GitHub Actions workflows for automated testing and quality assurance.

## Workflows

### 🧪 `test.yml` - Main Test Suite
**Trigger**: Push to `main`/`develop`, PRs to `main`/`develop`
- Builds BranchForge with full ROS2 Jazzy environment
- Runs all unit tests including BehaviorTreeXML tests
- Uploads test results as artifacts
- **Status**: ✅ Core test (test_behavior_tree_xml) passing with 15 tests

### 🏗️ `ci.yml` - Comprehensive CI
**Trigger**: Push to main branches and feature branches
- Multi-matrix builds (ROS2 Humble/Iron/Jazzy, Debug/Release)
- Static analysis with cppcheck and clang-tidy
- Code formatting checks
- Full dependency installation including BehaviorTree.CPP
- **Status**: 🔧 Comprehensive but may be resource-intensive

### ⚡ `pr-check.yml` - Fast PR Validation
**Trigger**: Pull request events
- Quick build validation without full ROS2 setup
- Code quality checks (cppcheck, clang-format)
- Project structure validation
- Dependency analysis
- **Status**: ✅ Lightweight and fast for quick feedback

## Current Test Coverage

### ✅ Working Tests
- **BehaviorTreeXML Tests**: 15 unit tests covering:
  - Node operations (add, remove, update)
  - Tree validation and metadata
  - XML import/export functionality
  - File I/O operations
  - Round-trip consistency

### 🔧 Tests in Development
- **CodeGenerator Tests**: C++20 code generation validation
- **BTSerializer Tests**: QML-to-C++ serialization
- **ProjectManager Tests**: Project file management
- **Integration Tests**: End-to-end workflow testing

## Test Execution

The workflows run tests using direct executable calls:
```bash
./build/branchforge/tests/unit/test_behavior_tree_xml
```

This approach bypasses `colcon test` issues and provides reliable test execution.

## Artifacts

Test workflows upload:
- Test result XML files
- Build logs and artifacts
- Static analysis reports

## Dependencies

### System Dependencies
- Ubuntu 24.04 LTS
- CMake 3.20+
- Qt6 (Base, Declarative, XML modules)
- Google Test framework
- pkg-config

### ROS2 Dependencies
- ROS2 Jazzy (primary target)
- rclcpp
- ament_cmake

### External Dependencies
- BehaviorTree.CPP v4.6+ (built from source)

## Usage

Workflows automatically trigger on:
- Push to protected branches (`main`, `develop`)
- Pull requests targeting protected branches
- Feature branch pushes (`feat/*`, `fix/*`)

## Quality Gates

All workflows include:
- ✅ Build success verification
- ✅ Core test execution (15 unit tests)
- ✅ Static analysis with cppcheck
- ✅ Basic code formatting checks
- ✅ Dependency validation

## Future Enhancements

- 🎯 Enable additional unit tests as they're fixed
- 🎯 Add integration test execution
- 🎯 Performance benchmarking
- 🎯 Code coverage reporting
- 🎯 Security scanning (CodeQL)
- 🎯 Release automation