cmake_minimum_required(VERSION 3.20)

project(uhdeb)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# We will assume that we will need to include 
# and link both UHD and Boost on everything

# Add the configuration for UHD via its config.cmake
# It seems like CMake's search is very powerful,
# and looks up almost every environment variable
# See https://cmake.org/cmake/help/v3.22/command/find_package.html#search-procedure
# So this should work as long as any env var points to the UHD install dir
# find_package(UHD REQUIRED)
find_package(UHD)

if (NOT DEFINED ${UHD_INCLUDE_DIRS} OR NOT DEFINED ${UHD_LIBRARIES})
    # We try to use custom environment variable to resolve instead.
    # This tends to happen on windows for older UHD versions whose
    # CMake config files relied on pkg-config.
    if (DEFINED ENV{UHD_INSTALL_PREFIX})
        message("Manually setting UHD paths via UHD_INSTALL_PREFIX..")
        set(UHD_INCLUDE_DIRS $ENV{UHD_INSTALL_PREFIX}/include)
        find_library(UHD_LIBRARIES NAMES uhd HINTS $ENV{UHD_INSTALL_PREFIX}/lib)
    else()
        message(FATAL_ERROR "Couldn't find UHD. Try setting environment variable UHD_INSTALL_PREFIX instead.")
    endif()
endif()

# UHD include dirs
message("Including " ${UHD_INCLUDE_DIRS})
message("Including " ${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(
    ${UHD_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Boost include dirs
find_package(Boost)
if (NOT ${Boost_FOUND})
    message(FATAL_ERROR "Couldn't find boost. Try setting env var BOOSTROOT to the boost directory.")
endif()
message("Including " ${Boost_INCLUDE_DIRS})
include_directories(
    ${Boost_INCLUDE_DIRS}
)

# Link boost lib directories
if (NOT DEFINED Boost_LIBRARY_DIRS)
    message("Default Boost_LIBRARY_DIRS not found.")
    if (DEFINED CUSTOM_Boost_LIBRARY_DIRS)
        set(Boost_LIBRARY_DIRS ${CUSTOM_Boost_LIBRARY_DIRS})
        message("Custom: " ${Boost_LIBRARY_DIRS})
    else()
        message(FATAL_ERROR "If you installed boost libraries from pre-compiled binaries, please add -DCUSTOM_Boost_LIBRARY_DIRS=/path/to/the/libs")
    endif()
endif()

# Link boost via the directory
message("Linking boost libraries in " ${Boost_LIBRARY_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

# Link UHD via the direct path
message("Linking UHD at " ${UHD_LIBRARIES})
link_libraries(${UHD_LIBRARIES})

# Add all individual subdirectories
add_subdirectory(tx_file_while_rx)
add_subdirectory(tools)
add_subdirectory(encapsulation)


