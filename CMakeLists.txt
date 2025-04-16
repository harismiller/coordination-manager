cmake_minimum_required(VERSION 3.10)

# Project name and version
project(CoordinationManager VERSION 1.0)

# # Set GCC as the compiler
# set(CMAKE_C_COMPILER gcc)
# set(CMAKE_CXX_COMPILER g++)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add the executable
add_executable(coordination_manager
    main.cpp                  # Main file is now outside src/
    src/coordination_manager.cpp
    src/system_compiler.cpp
    src/agent_data.cpp
)

# Include directories
target_include_directories(coordination_manager PRIVATE include)