# Copyright 2026, Yang Luo
# SPDX-License-Identifier: GPL-3.0-or-later
#
# CMakeLists.txt
# Build system for the rocos-mujoco MuJoCo simulation environment.

cmake_minimum_required(VERSION 3.16)
project(rocos_mujoco VERSION 1.0.0
        DESCRIPTION "ROCOS MuJoCo Simulation Environment")

# ---- C++ standard --------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# ==========================================================================
# Third-party libraries (仿照 RoboChargeX 的 third_party 结构)
# ==========================================================================

# ---- MuJoCo 3.4.0 --------------------------------------------------------
set(MUJOCO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/mujoco-3.4.0")
if(NOT EXISTS "${MUJOCO_DIR}/lib/libmujoco.so")
    set(MUJOCO_DIR "$ENV{HOME}/.mujoco/mujoco-3.4.0")
endif()
message(STATUS "Third-party MuJoCo:  ${MUJOCO_DIR}")

find_library(MUJOCO_LIB mujoco
             PATHS "${MUJOCO_DIR}/lib"
             NO_DEFAULT_PATH REQUIRED)

add_library(third_party::mujoco SHARED IMPORTED)
set_target_properties(third_party::mujoco PROPERTIES
    IMPORTED_LOCATION              "${MUJOCO_LIB}"
    INTERFACE_INCLUDE_DIRECTORIES  "${MUJOCO_DIR}/include")

# ---- GLFW -----------------------------------------------------------------
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
    message(FATAL_ERROR "GLFW not found")
endif()

# ---- yaml-cpp -------------------------------------------------------------
find_package(yaml-cpp REQUIRED)
message(STATUS "Third-party yaml-cpp: system")

# ---- OpenGL ---------------------------------------------------------------
find_package(OpenGL REQUIRED)

# ---- Threads --------------------------------------------------------------
find_package(Threads REQUIRED)

# ==========================================================================
# Build targets
# ==========================================================================

# ---- Shared library: librocos_mujoco.so -----------------------------------
add_library(rocos_mujoco SHARED
    src/mujoco_simulator.cpp
    src/hardware_mj.cpp
)

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
        OpenGL::GL
)
# GLFW: link conditionally based on how it was found
if(glfw3_FOUND)
    target_link_libraries(rocos_mujoco PRIVATE glfw)
else()
    target_link_libraries(rocos_mujoco PRIVATE ${GLFW3_LIBRARIES})
    target_include_directories(rocos_mujoco PRIVATE ${GLFW3_INCLUDE_DIRS})
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

# ---- Copy config & model files to build directory -------------------------
file(COPY config/ DESTINATION ${CMAKE_BINARY_DIR}/bin/config)
file(COPY model/  DESTINATION ${CMAKE_BINARY_DIR}/bin/model)
add_custom_command(TARGET rocos_mujoco_sim POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_SOURCE_DIR}/config"
        "${CMAKE_BINARY_DIR}/bin/config"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_SOURCE_DIR}/model"
        "${CMAKE_BINARY_DIR}/bin/model"
)
