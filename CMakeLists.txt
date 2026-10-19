# Copyright 2026, Yang Luo
# SPDX-License-Identifier: GPL-3.0-or-later
#
# CMakeLists.txt
# Build system for the rocos-mujoco MuJoCo simulation environment.

cmake_minimum_required(VERSION 3.16)
file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/VERSION" ROCOS_MUJOCO_VERSION LIMIT_COUNT 1)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS VERSION)
# CMake project(VERSION) only accepts the numeric part of a SemVer version.
string(REGEX MATCH "^[0-9]+\\.[0-9]+\\.[0-9]+" ROCOS_MUJOCO_VERSION_CORE "${ROCOS_MUJOCO_VERSION}")
project(rocos_mujoco VERSION ${ROCOS_MUJOCO_VERSION_CORE}
        DESCRIPTION "ROCOS MuJoCo Simulation Environment")
message(STATUS "ROCOS MuJoCo version: ${ROCOS_MUJOCO_VERSION}")

# ---- C++ standard --------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

option(ROCOS_MUJOCO_ENABLE_VISUALIZATION
       "Build the GLFW/OpenGL MuJoCo visualization window" ON)

# ==========================================================================
# Third-party libraries (仿照 RoboChargeX 的 third_party 结构)
# ==========================================================================

# ---- MuJoCo 3.4.0 --------------------------------------------------------
set(MUJOCO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/mujoco-3.4.0")
if(NOT EXISTS "${MUJOCO_DIR}/lib/libmujoco.so" AND
   NOT EXISTS "${MUJOCO_DIR}/lib/libmujoco.so.3.4.0")
    set(MUJOCO_DIR "$ENV{HOME}/.mujoco/mujoco-3.4.0")
endif()
message(STATUS "Third-party MuJoCo:  ${MUJOCO_DIR}")

if(EXISTS "${MUJOCO_DIR}/lib/libmujoco.so.3.4.0")
    # Prefer the real versioned library. On Windows-mounted WSL checkouts,
    # libmujoco.so can be a plain-text symlink placeholder.
    set(MUJOCO_LIB "${MUJOCO_DIR}/lib/libmujoco.so.3.4.0")
elseif(EXISTS "${MUJOCO_DIR}/lib/libmujoco.so")
    set(MUJOCO_LIB "${MUJOCO_DIR}/lib/libmujoco.so")
else()
    find_library(MUJOCO_LIB mujoco
                 PATHS "${MUJOCO_DIR}/lib"
                 NO_DEFAULT_PATH REQUIRED)
endif()

add_library(third_party::mujoco SHARED IMPORTED)
set_target_properties(third_party::mujoco PROPERTIES
    IMPORTED_LOCATION              "${MUJOCO_LIB}"
    INTERFACE_INCLUDE_DIRECTORIES  "${MUJOCO_DIR}/include")

# ---- GLFW / OpenGL (optional visualization) --------------------------------
if(ROCOS_MUJOCO_ENABLE_VISUALIZATION)
    find_package(glfw3 QUIET)
    if(NOT glfw3_FOUND)
        find_package(PkgConfig QUIET)
        if(PkgConfig_FOUND)
            pkg_check_modules(GLFW3 glfw3)
        endif()
    endif()
    if(glfw3_FOUND)
        message(STATUS "Third-party GLFW:   system (cmake)")
    elseif(GLFW3_FOUND)
        message(STATUS "Third-party GLFW:   system (pkg-config)")
    else()
        message(FATAL_ERROR "GLFW not found; use -DROCOS_MUJOCO_ENABLE_VISUALIZATION=OFF for headless servers")
    endif()

    find_package(OpenGL REQUIRED)
    find_package(PNG 1.6 REQUIRED)
    message(STATUS "Visualization:      enabled")
else()
    message(STATUS "Visualization:      disabled (headless)")
endif()

# ---- yaml-cpp -------------------------------------------------------------
find_package(yaml-cpp REQUIRED)
message(STATUS "Third-party yaml-cpp: system")

# ---- Threads --------------------------------------------------------------
find_package(Threads REQUIRED)

# ==========================================================================
# Build targets
# ==========================================================================

# ---- Shared library: librocos_mujoco.so -----------------------------------
add_library(rocos_mujoco SHARED
    src/mujoco_simulator.cpp
)
if(ROCOS_MUJOCO_ENABLE_VISUALIZATION)
    target_sources(rocos_mujoco PRIVATE src/hardware_mj.cpp src/simulation_menu.cpp)

    # Embed the logo so installed binaries do not need a runtime asset path.
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS rocos-mujoco.png)
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/rocos-mujoco.png" ROCOS_LOGO_HEX HEX)
    string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," ROCOS_LOGO_BYTES "${ROCOS_LOGO_HEX}")
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/generated")
    configure_file(src/logo_data.hpp.in generated/logo_data.hpp @ONLY)
    target_include_directories(rocos_mujoco PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/generated")
endif()

target_include_directories(rocos_mujoco
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(rocos_mujoco
    PUBLIC
        third_party::mujoco
        yaml-cpp
    PRIVATE
        Threads::Threads
        rt
        pthread
)
if(ROCOS_MUJOCO_ENABLE_VISUALIZATION)
    target_compile_definitions(rocos_mujoco
        PUBLIC ROCOS_MUJOCO_ENABLE_VISUALIZATION=1)
    target_link_libraries(rocos_mujoco PRIVATE OpenGL::GL PNG::PNG)
    # GLFW: link conditionally based on how it was found
    if(glfw3_FOUND)
        target_link_libraries(rocos_mujoco PRIVATE glfw)
    else()
        target_link_libraries(rocos_mujoco PRIVATE ${GLFW3_LIBRARIES})
        target_include_directories(rocos_mujoco PRIVATE ${GLFW3_INCLUDE_DIRS})
    endif()
endif()

# ---- Executable: rocos_mujoco_sim -----------------------------------------
add_executable(rocos_mujoco_sim
    src/main.cpp
)
target_link_libraries(rocos_mujoco_sim PRIVATE rocos_mujoco)

# ==========================================================================
# Install
# ==========================================================================
include(GNUInstallDirs)

install(TARGETS rocos_mujoco rocos_mujoco_sim
    EXPORT  rocos_mujoco-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
