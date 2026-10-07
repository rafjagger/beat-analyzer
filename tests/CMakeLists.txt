cmake_minimum_required(VERSION 3.16)

# The tests are plain assert()s. build.sh builds Release, and Release sets
# -DNDEBUG, which turns every assert() into nothing: until 2026-09-30 each of
# these passed without checking anything. Keep them on for the tests.
add_compile_options(-UNDEBUG)

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

# The VU inputs' names, after REAPER's VU outputs 31-70
add_executable(vu_ports_test
    vu_ports_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/audio/vu_ports.cpp
)
target_include_directories(vu_ports_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
add_test(NAME VuPortsTests COMMAND vu_ports_test)


# The OSC words and ports from the .env's a3-osc block (the one truth)
add_executable(osc_words_test
    osc_words_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/config/osc_words.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/audio/vu_ports.cpp
)
target_include_directories(osc_words_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
add_test(NAME OscWordsTests COMMAND osc_words_test)

# The VU channels, split into bundles that fit the sender's 512-byte slots
add_executable(vu_bundle_test
    vu_bundle_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/osc/osc_sender.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/util/logging.cpp
)
target_include_directories(vu_bundle_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_link_libraries(vu_bundle_test PRIVATE pthread)
add_test(NAME VuBundleTests COMMAND vu_bundle_test)

# The unit leaves real-time to JACK's process thread
add_executable(unit_scheduling_test unit_scheduling_test.cpp)
target_compile_definitions(unit_scheduling_test PRIVATE
    BEAT_ANALYZER_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/.."
)
add_test(NAME UnitSchedulingTests COMMAND unit_scheduling_test)

# The VU peak per send is the raw highest sample since the last send
add_executable(vu_meter_test
    vu_meter_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/analysis/vu_meter.cpp
)
target_include_directories(vu_meter_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
add_test(NAME VuMeterTests COMMAND vu_meter_test)
