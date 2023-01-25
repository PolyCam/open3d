cmake_minimum_required(VERSION 3.19)
project(las)

set(TARGET las)

# ==================================================================================================
# Sources and headers
# ==================================================================================================
file(GLOB SRCS src/*.cpp zip/*.cpp)

# ==================================================================================================
# Include and target definitions
# ==================================================================================================
add_library(${TARGET} STATIC ${SRCS})
target_include_directories(${TARGET}
PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/inc>
)
target_include_directories(${TARGET}
PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/zip>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/zip>
)

# ==================================================================================================
# Installation
# ==================================================================================================
install(TARGETS ${TARGET} EXPORT open3dTargets ARCHIVE DESTINATION lib)
