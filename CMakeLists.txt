cmake_minimum_required(VERSION 3.16)
project(BeatAnalyzer 
    VERSION 1.0.0
    DESCRIPTION "Real-time beat analysis and OSC synchronization"
    LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)

# Options
option(BUILD_TESTS "Build unit tests" ON)
option(BUILD_SHARED_LIBS "Build shared libraries" ON)

# Find packages
find_package(PkgConfig REQUIRED)

pkg_check_modules(JACK jack)
pkg_check_modules(SNDFILE sndfile)
pkg_check_modules(SAMPLERATE REQUIRED samplerate)

# Try to find in system if pkg-config fails
if(NOT JACK_FOUND)
    find_package(JACK QUIET)
endif()

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

# BTrack library (submodule)
set(BTRACK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/BTrack)
# BTrack is upstream code and stays untouched in the submodule. Our changes
# live in patches/ and are applied to a copy in the build directory, so what
# differs from upstream is always one readable file away.
set(BTRACK_PATCHED_DIR ${CMAKE_BINARY_DIR}/btrack-patched)
file(GLOB BTRACK_PATCHES ${CMAKE_CURRENT_SOURCE_DIR}/patches/btrack-*.patch)
list(SORT BTRACK_PATCHES)
file(REMOVE_RECURSE ${BTRACK_PATCHED_DIR})
file(COPY ${BTRACK_DIR}/src/BTrack.cpp ${BTRACK_DIR}/src/BTrack.h
     DESTINATION ${BTRACK_PATCHED_DIR}/src)
foreach(patch_file ${BTRACK_PATCHES})
    execute_process(
        COMMAND patch -p1 --forward --input ${patch_file}
        WORKING_DIRECTORY ${BTRACK_PATCHED_DIR}
        RESULT_VARIABLE patch_result
        OUTPUT_VARIABLE patch_output
        ERROR_VARIABLE patch_output)
    if(NOT patch_result EQUAL 0)
        message(FATAL_ERROR "BTrack patch ${patch_file} does not apply:\n${patch_output}")
    endif()
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${patch_file})
    message(STATUS "BTrack patched: ${patch_file}")
endforeach()
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    ${BTRACK_DIR}/src/BTrack.cpp ${BTRACK_DIR}/src/BTrack.h)

set(BTRACK_SOURCES
    ${BTRACK_PATCHED_DIR}/src/BTrack.cpp
    ${BTRACK_DIR}/src/OnsetDetectionFunction.cpp
    ${BTRACK_DIR}/libs/kiss_fft130/kiss_fft.c
)
# The patched copy first, so "BTrack.h" is the patched header.
set(BTRACK_INCLUDE_DIRS
    ${BTRACK_PATCHED_DIR}/src
    ${BTRACK_DIR}/src
    ${BTRACK_DIR}/libs/kiss_fft130
)

# Source files
set(BEAT_ANALYZER_SOURCES
    src/main.cpp
    src/app/beat_analyzer_app.cpp
    src/app/beat_processing.cpp
    src/audio/jack_client.cpp
    src/audio/audio_buffer.cpp
    src/analysis/beat_detection.cpp
    src/analysis/beat_tracker.cpp
    src/analysis/beat_clock_follower.cpp
    src/analysis/vu_meter.cpp
    src/analysis/grid_calculator.cpp
    src/osc/osc_sender.cpp
    src/osc/osc_receiver.cpp
    src/osc/osc_messages.cpp
    src/osc/pioneer_receiver.cpp
    src/config/config_loader.cpp
    src/util/logging.cpp
    ${BTRACK_SOURCES}
)

set(BEAT_ANALYZER_HEADERS
    include/app/beat_analyzer_app.h
    include/audio/jack_client.h
    include/audio/audio_buffer.h
    include/audio/audio_types.h
    include/analysis/beat_detection.h
    include/analysis/beat_tracker.h
    include/analysis/beat_info.h
    include/analysis/vu_meter.h
    include/analysis/grid_calculator.h
    include/osc/osc_sender.h
    include/osc/osc_receiver.h
    include/osc/osc_messages.h
    include/osc/pioneer_receiver.h
    include/config/config_loader.h
    include/util/logging.h
)

# Main executable
add_executable(beat-analyzer ${BEAT_ANALYZER_SOURCES} ${BEAT_ANALYZER_HEADERS})

# BTrack compile definitions and includes
target_compile_definitions(beat-analyzer PRIVATE USE_KISS_FFT)
target_include_directories(beat-analyzer BEFORE PRIVATE
    ${BTRACK_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(beat-analyzer PRIVATE
    ${JACK_LIBRARIES}
    ${SAMPLERATE_LIBRARIES}
    pthread
    m
)

# liblo nicht mehr benötigt — OSC sender/receiver nutzen raw UDP sockets

# Optional sndfile support
if(SNDFILE_FOUND)
    target_link_libraries(beat-analyzer PRIVATE ${SNDFILE_LIBRARIES})
    target_compile_definitions(beat-analyzer PRIVATE HAS_SNDFILE=1)
endif()

# Include directories for dependencies
target_include_directories(beat-analyzer PRIVATE
    ${JACK_INCLUDE_DIRS}
    ${SNDFILE_INCLUDE_DIRS}
    ${SAMPLERATE_INCLUDE_DIRS}
)

# Compiler flags
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(beat-analyzer PRIVATE
        -Wall -Wextra -Wpedantic
        -O3
        -march=native
    )
endif()

# Tests
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# Installation
install(TARGETS beat-analyzer
    RUNTIME DESTINATION bin
)

# Print summary
message(STATUS "BeatAnalyzer Configuration:")
message(STATUS "  JACK support: ${JACK_FOUND}")
message(STATUS "  Build tests: ${BUILD_TESTS}")
