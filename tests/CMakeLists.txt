# BranchForge Test Suite

# Helper function to create tests
function(add_branchforge_test test_name test_file)
    add_executable(${test_name} ${test_file})
    
    target_link_libraries(${test_name}
        branchforge_test_lib
        gtest
        gtest_main
    )
    
    target_include_directories(${test_name} PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/tests
    )
    
    # Add test to CTest
    add_test(NAME ${test_name} COMMAND ${test_name})
    
    # Set test properties
    set_tests_properties(${test_name} PROPERTIES
        TIMEOUT 60
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    )
endfunction()

# Unit Tests
add_subdirectory(unit)

# Integration Tests
add_subdirectory(integration)

# System Tests (optional, for longer-running tests)
if(BUILD_SYSTEM_TESTS)
    add_subdirectory(system)
endif()

# Test data
configure_file(data/sample_tree.xml data/sample_tree.xml COPYONLY)
configure_file(data/complex_tree.xml data/complex_tree.xml COPYONLY)
configure_file(data/invalid_tree.xml data/invalid_tree.xml COPYONLY)