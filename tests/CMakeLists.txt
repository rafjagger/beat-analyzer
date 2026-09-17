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

# BeatClockFollower: die Clock folgt BTracks Beats
add_executable(beat_clock_follower_test
    beat_clock_follower_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/analysis/beat_clock_follower.cpp
)
target_include_directories(beat_clock_follower_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
add_test(NAME BeatClockFollowerTests COMMAND beat_clock_follower_test)

# BTrack mit gemessener Beat-Periode (gepatcht, siehe patches/)
add_executable(btrack_period_test
    btrack_period_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/analysis/beat_clock_follower.cpp
    ${BTRACK_SOURCES}
)
target_include_directories(btrack_period_test BEFORE PRIVATE
    ${BTRACK_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${SAMPLERATE_INCLUDE_DIRS}
)
target_compile_definitions(btrack_period_test PRIVATE
    USE_KISS_FFT
    BEAT_ANALYZER_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/.."
)
target_link_libraries(btrack_period_test PRIVATE ${SAMPLERATE_LIBRARIES} m)
add_test(NAME BTrackPeriodTests COMMAND btrack_period_test)
