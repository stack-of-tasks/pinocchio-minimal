cmake_minimum_required(VERSION 3.10)

set(PROJECT_NAME pinocchio-minimal)
set(PROJECT_DESCRIPTION
    "Minimal example of a simple application depending on Pinocchio")
set(PROJECT_URL "http://github.com/stack-of-tasks/${PROJECT_NAME}")

project(${PROJECT_NAME})

# pinocchio dependency
find_package(pinocchio 2.7.0 REQUIRED)

# Create the main executable
add_executable(main main.cpp)

# Link main to its dependencies
target_link_libraries(main PUBLIC pinocchio::pinocchio)
