cmake_minimum_required(VERSION 3.20)
project(servo_sensei VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---------------------------------------------------------------------------
# Main application
# ---------------------------------------------------------------------------
add_executable(servo_app app/main.cpp)
target_include_directories(servo_app PRIVATE include)

add_executable(servo_controller app/servo_controller.cpp)
target_include_directories(servo_controller PRIVATE include)

add_executable(scan_servos app/scan_servos.cpp)
target_include_directories(scan_servos PRIVATE include)

add_executable(servo_cli app/servo_cli.cpp)
target_include_directories(servo_cli PRIVATE include)

# ---------------------------------------------------------------------------
# Unit tests  (no hardware required — run with ctest)
# ---------------------------------------------------------------------------
option(BUILD_TESTING "Build unit tests" ON)
if(BUILD_TESTING)

include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(servo_tests
    tests/test_commands.cpp
    tests/test_response.cpp
    tests/test_serial_driver.cpp
)
target_include_directories(servo_tests PRIVATE include tests)
target_link_libraries(servo_tests PRIVATE GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(servo_tests)

endif() # BUILD_TESTING

# ---------------------------------------------------------------------------
# Hardware tests  (require a physical servo bus — run manually with sudo)
# ---------------------------------------------------------------------------
if(BUILD_TESTING)

add_executable(test_sync_read tests/test_sync_read.cpp)
target_include_directories(test_sync_read PRIVATE include)

add_executable(test_sync_rw tests/test_sync_rw.cpp)
target_include_directories(test_sync_rw PRIVATE include)

endif() # BUILD_TESTING

# ---------------------------------------------------------------------------
# Install rules
# ---------------------------------------------------------------------------
include(GNUInstallDirs)

install(TARGETS scan_servos servo_cli servo_controller
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Install the env config as a sample; .sample suffix prevents overwriting
# user-edited files on upgrade. The recipe renames it to servo.env.
install(FILES conf/servo.env
    DESTINATION ${CMAKE_INSTALL_SYSCONFDIR}/servo_sensei
    RENAME servo.env.sample
)

# ---------------------------------------------------------------------------
# Documentation  (requires Doxygen — run with: cmake --build build --target docs)
# ---------------------------------------------------------------------------
find_package(Doxygen QUIET)
if(DOXYGEN_FOUND)
    add_custom_target(docs
        COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_SOURCE_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Generating HTML documentation with Doxygen"
        VERBATIM
    )
else()
    message(STATUS "Doxygen not found — 'docs' target unavailable")
endif()
