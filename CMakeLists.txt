cmake_minimum_required(VERSION 3.19)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED yes)
set(ENABLE_EXCEPTIONS yes)
set(ENABLE_RTTI yes)

project(open3d)

set(TARGET open3d)

# ==================================================================================================
# Sources and headers
# ==================================================================================================
file(GLOB_RECURSE SRCS ${PROJECT_SOURCE_DIR}/open3d *.cpp)

# ==================================================================================================
# Include and target definitions
# ==================================================================================================
add_library(${TARGET} STATIC ${SRCS})
target_include_directories(${TARGET} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}>
)
target_include_directories(${TARGET} PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/tomasakeninemoeller/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/3rdparty/tomasakeninemoeller/include>
)
target_include_directories(${TARGET} PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/poisson-recon>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/3rdparty/poisson-recon>
)

if(UNIX OR MINGW)
    target_compile_options(${TARGET} PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-fpermissive -w>) # Eigen generates deprecated warnings due to
        # use of std::result and this silences those.
elseif(MSVC)
    add_definitions(-D_USE_MATH_DEFINES -DNOMINMAX)
endif()

# ==================================================================================================
# Dependencies
# ==================================================================================================

find_package(tinygltf CONFIG REQUIRED)
target_link_libraries(${TARGET} PRIVATE tinygltf)

find_package(PNG REQUIRED)
include_directories(${PNG_INCLUDE_DIR})
target_link_libraries(${TARGET} PRIVATE ${PNG_LIBRARIES})

find_package(Eigen3 CONFIG REQUIRED)
target_link_libraries(${TARGET} PRIVATE Eigen3::Eigen)

find_package(fmt CONFIG REQUIRED)
target_link_libraries(${TARGET} PUBLIC fmt::fmt)

find_package(jsoncpp CONFIG REQUIRED)
target_link_libraries(${TARGET} PRIVATE JsonCpp::JsonCpp)

find_package(Qhull CONFIG REQUIRED)
target_link_libraries(${TARGET} PRIVATE Qhull::qhull_r Qhull::qhullcpp)

find_package(rply CONFIG REQUIRED)
target_link_libraries(${TARGET} PRIVATE rply::rply)

find_package(tinyobjloader CONFIG REQUIRED)
target_link_libraries(${TARGET} PRIVATE tinyobjloader::tinyobjloader)

find_package(lz4 CONFIG REQUIRED)
target_link_libraries(${TARGET} PRIVATE lz4::lz4)

# ==================================================================================================
# 3rd party
# ==================================================================================================

add_subdirectory(3rdparty/las)
target_link_libraries(${TARGET} PRIVATE las)

# ==================================================================================================
# Installation
# ==================================================================================================
install(TARGETS ${TARGET} EXPORT ${TARGET}Targets ARCHIVE DESTINATION lib)
install(DIRECTORY open3d/ DESTINATION ${CMAKE_INSTALL_PREFIX}/include/open3d FILES_MATCHING PATTERN "*.h")
install(EXPORT "${TARGET}Targets" DESTINATION "share/${TARGET}")
install(FILES cmake/${TARGET}Config.cmake DESTINATION "share/${TARGET}")
