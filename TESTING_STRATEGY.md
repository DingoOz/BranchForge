# BranchForge Testing Strategy

## Overview

This document outlines the comprehensive testing strategy for BranchForge to ensure quality, reliability, and prevent regressions as the codebase evolves.

## Testing Pyramid

### 1. Unit Tests (70% of tests)
- **Core Components**: BehaviorTreeXML, CodeGenerator, BTSerializer
- **Node System**: AdvancedNodeSystem, node creation/deletion
- **UI Components**: Individual QML components
- **Utilities**: String processing, validation, file I/O

### 2. Integration Tests (20% of tests)
- **End-to-end workflows**: Visual design → Code generation → Compilation
- **QML-C++ bridge**: BTSerializer integration
- **ROS2 integration**: Generated code compilation and execution
- **File system operations**: Project save/load, code export

### 3. System Tests (10% of tests)
- **Full application testing**: Complete user workflows
- **Performance testing**: Large behavior trees, memory usage
- **Cross-platform testing**: Ubuntu versions, Qt versions
- **Regression testing**: Ensure existing functionality still works

## Test Categories

### Core Library Tests
```
tests/
├── unit/
│   ├── core/
│   │   ├── test_application.cpp
│   │   └── test_logging.cpp
│   ├── project/
│   │   ├── test_behavior_tree_xml.cpp
│   │   ├── test_code_generator.cpp
│   │   ├── test_bt_serializer.cpp
│   │   └── test_project_manager.cpp
│   ├── nodes/
│   │   ├── test_advanced_node_system.cpp
│   │   └── test_node_validation.cpp
│   ├── ui/
│   │   └── test_main_window.cpp
│   └── utilities/
│       ├── test_string_utils.cpp
│       └── test_file_utils.cpp
```

### Integration Tests
```
tests/
├── integration/
│   ├── test_visual_to_code_pipeline.cpp
│   ├── test_qml_cpp_bridge.cpp
│   ├── test_generated_code_compilation.cpp
│   ├── test_ros2_integration.cpp
│   └── test_project_workflows.cpp
```

### QML Tests
```
tests/
├── qml/
│   ├── test_node_editor.qml
│   ├── test_node_library.qml
│   ├── test_properties_panel.qml
│   ├── test_code_gen_dialog.qml
│   └── test_main_window.qml
```

### System Tests
```
tests/
├── system/
│   ├── test_full_workflow.cpp
│   ├── test_performance.cpp
│   ├── test_memory_usage.cpp
│   └── test_large_projects.cpp
```

## Test Data and Fixtures

### Sample Behavior Trees
- Simple linear sequence
- Complex branching selector
- Deep nested structure
- Large-scale navigation tree
- Error/invalid trees for negative testing

### Generated Code Samples
- Reference implementations for validation
- Expected output templates
- Compilation test projects

## Testing Tools and Frameworks

### C++ Testing
- **Google Test (gtest)**: Primary testing framework
- **Google Mock (gmock)**: Mocking framework for dependencies
- **Qt Test**: Qt-specific testing utilities

### QML Testing
- **Qt Quick Test**: QML unit testing
- **TestCase**: QML component testing

### Code Coverage
- **gcov/lcov**: Code coverage analysis
- **Target**: 80%+ coverage for core components

### Performance Testing
- **Google Benchmark**: Performance regression detection
- **Memory profiling**: Valgrind integration

## Continuous Integration

### Automated Testing Pipeline
1. **Build Stage**: Compile all configurations
2. **Unit Test Stage**: Run all unit tests
3. **Integration Test Stage**: Run integration tests
4. **System Test Stage**: Full workflow tests
5. **Coverage Report**: Generate coverage reports
6. **Performance Benchmark**: Track performance metrics

### Test Environments
- **Ubuntu 22.04 LTS** with Qt6 6.2+
- **Ubuntu 24.04 LTS** with Qt6 6.4+
- **ROS2 Humble, Iron, Jazzy**

## Test Standards

### Test Naming Convention
- `Test_<ComponentName>_<Functionality>`
- Example: `Test_CodeGenerator_GenerateActionNode`

### Test Structure (AAA Pattern)
```cpp
TEST(ComponentName, TestCase) {
    // Arrange: Set up test data and conditions
    
    // Act: Execute the functionality being tested
    
    // Assert: Verify the results
}
```

### Mock Usage
- Mock external dependencies (file system, network)
- Use dependency injection for testability
- Avoid mocking value objects

## Quality Gates

### Pre-commit Checks
- All unit tests must pass
- Code coverage must not decrease
- Static analysis must pass (clang-tidy)

### Release Criteria
- 100% unit test pass rate
- 100% integration test pass rate
- 95%+ system test pass rate
- Performance benchmarks within acceptable range

## Regression Testing

### Automated Regression Suite
- Run full test suite on every PR
- Nightly builds with extended test suite
- Weekly performance regression tests

### Manual Testing Checklist
- [ ] Visual node creation and connection
- [ ] Code generation for all node types
- [ ] Generated code compilation
- [ ] UI responsiveness and interaction
- [ ] File save/load operations

## Test Data Management

### Version Control
- Store test data in `tests/data/`
- Version control expected outputs
- Maintain backward compatibility test cases

### Test Database
- Sample behavior trees of varying complexity
- Reference XML files
- Expected generated code outputs

## Metrics and Reporting

### Test Metrics
- **Test Coverage**: Line and branch coverage
- **Test Execution Time**: Track test performance
- **Flaky Test Detection**: Identify unstable tests
- **Regression Detection**: Track new failures

### Reporting
- Daily test reports
- Coverage trend analysis
- Performance regression alerts
- Quality dashboard

## Implementation Phases

### Phase 1: Foundation (Week 1-2)
- Set up Google Test framework
- Create basic unit tests for core components
- Establish CI pipeline

### Phase 2: Core Testing (Week 3-4)
- Comprehensive unit tests for all components
- Integration tests for major workflows
- QML component tests

### Phase 3: Advanced Testing (Week 5-6)
- System tests and performance benchmarks
- Regression test suite
- Coverage analysis and optimization

### Phase 4: Automation (Week 7-8)
- Full CI/CD integration
- Automated reporting
- Performance monitoring

## Benefits

1. **Quality Assurance**: Catch bugs early in development
2. **Regression Prevention**: Ensure changes don't break existing functionality
3. **Documentation**: Tests serve as living documentation
4. **Confidence**: Safe refactoring and feature addition
5. **Performance**: Track and prevent performance regressions
6. **Maintainability**: Easier debugging and troubleshooting

This comprehensive testing strategy ensures BranchForge maintains high quality and reliability as it evolves from Phase 1 through Phase 4 of development.