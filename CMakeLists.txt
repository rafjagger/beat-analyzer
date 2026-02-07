cmake_minimum_required(VERSION 3.16)
project(BeatAnalyzer 
    VERSION 1.0.0
    DESCRIPTION "Real-time beat analysis and OSC synchronization"
    LANGUAGES CXX)

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

# Try to find in system if pkg-config fails
if(NOT JACK_FOUND)
    find_package(JACK QUIET)
endif()

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

# Source files
set(BEAT_ANALYZER_SOURCES
    src/main.cpp
    src/audio/jack_client.cpp
    src/audio/audio_buffer.cpp
    src/analysis/beat_detection.cpp
    src/analysis/beat_tracker.cpp
    src/analysis/vu_meter.cpp
    src/osc/osc_sender.cpp
    src/osc/osc_receiver.cpp
    src/osc/osc_messages.cpp
    src/config/config_loader.cpp
    src/util/logging.cpp
)

set(BEAT_ANALYZER_HEADERS
    include/audio/jack_client.h
    include/audio/audio_buffer.h
    include/audio/audio_types.h
    include/analysis/beat_detection.h
    include/analysis/beat_tracker.h
    include/analysis/beat_info.h
    include/analysis/vu_meter.h
    include/osc/osc_sender.h
    include/osc/osc_receiver.h
    include/osc/osc_messages.h
    include/config/config_loader.h
    include/util/logging.h
)

# Main executable
add_executable(beat-analyzer ${BEAT_ANALYZER_SOURCES} ${BEAT_ANALYZER_HEADERS})

# Link libraries
target_link_libraries(beat-analyzer PRIVATE
    ${JACK_LIBRARIES}
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

install(FILES config/config.yaml
    DESTINATION etc/beat-analyzer
)

# Print summary
message(STATUS "BeatAnalyzer Configuration:")
message(STATUS "  JACK support: ${JACK_FOUND}")
message(STATUS "  Build tests: ${BUILD_TESTS}")
