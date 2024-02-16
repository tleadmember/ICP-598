cmake_minimum_required(VERSION 3.1)
project(ICP-598)
set(CMAKE_CXX_STANDARD 17) # Use C++17

if(NOT CMAKE_BUILD_TYPE) # Set build type if not set
    set(CMAKE_BUILD_TYPE Debug)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra -Werror") # Set additional flags
set(CMAKE_CXX_FLAGS_DEBUG "-g -O3")

# find_path()

include_directories(include/)

add_library(matcher SHARED src/matcher.cpp) # libmatcher.so

# find_library()

add_executable(main src/main.cpp)

target_link_libraries(main matcher)