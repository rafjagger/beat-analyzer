cmake_minimum_required(VERSION 3.16)

# Beat Detection Test
add_executable(beat_detection_test
    beat_detection_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/analysis/beat_detection.cpp
)

target_include_directories(beat_detection_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

target_link_libraries(beat_detection_test PRIVATE
    pthread
    m
)

# Add test
add_test(NAME BeatDetectionTests COMMAND beat_detection_test)
