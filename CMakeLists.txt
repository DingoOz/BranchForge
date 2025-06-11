cmake_minimum_required(VERSION 3.20)
project(branchforge VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find Qt6 packages - start simple
find_package(Qt6 REQUIRED COMPONENTS Core Widgets)

qt6_standard_project_setup()

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set(SOURCES
    src/main_enhanced_fixed.cpp
)

set(HEADERS
)

add_executable(branchforge_enhanced ${SOURCES} ${HEADERS})

target_link_libraries(branchforge_enhanced PRIVATE
    Qt6::Core
    Qt6::Widgets
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