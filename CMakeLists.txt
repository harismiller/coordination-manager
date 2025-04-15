cmake_minimum_required(VERSION 3.10)

# Project name and version
project(CoordinationManager VERSION 1.0)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add the executable
add_executable(coordination_manager
    main.cpp
    coordination_manager.cpp
    system_compiler.cpp
)

# Include directories
target_include_directories(coordination_manager PRIVATE ${CMAKE_SOURCE_DIR})

# Add libraries (if needed)
# Example: If you use the argparse library, you can link it here
# target_link_libraries(coordination_manager PRIVATE argparse)