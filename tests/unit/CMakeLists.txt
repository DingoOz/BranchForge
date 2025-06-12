# Unit Tests

# Core component tests
add_branchforge_test(test_behavior_tree_xml ${CMAKE_CURRENT_SOURCE_DIR}/project/test_behavior_tree_xml.cpp)
# add_branchforge_test(test_code_generator ${CMAKE_CURRENT_SOURCE_DIR}/project/test_code_generator.cpp)
# add_branchforge_test(test_bt_serializer ${CMAKE_CURRENT_SOURCE_DIR}/project/test_bt_serializer.cpp)
# add_branchforge_test(test_project_manager ${CMAKE_CURRENT_SOURCE_DIR}/project/test_project_manager.cpp)

# Node system tests
# add_branchforge_test(test_advanced_node_system ${CMAKE_CURRENT_SOURCE_DIR}/nodes/test_advanced_node_system.cpp)

# Core application tests
# add_branchforge_test(test_application ${CMAKE_CURRENT_SOURCE_DIR}/core/test_application.cpp)

# UI tests (Qt-dependent) - Commented out for now
# add_branchforge_test(test_main_window ${CMAKE_CURRENT_SOURCE_DIR}/ui/test_main_window.cpp)

# Utility tests - Commented out for now
# add_branchforge_test(test_string_utils ${CMAKE_CURRENT_SOURCE_DIR}/utilities/test_string_utils.cpp)