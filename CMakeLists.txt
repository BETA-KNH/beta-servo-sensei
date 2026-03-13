cmake_minimum_required(VERSION 3.20)
project(servo_sensei VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---------------------------------------------------------------------------
# Main application
# ---------------------------------------------------------------------------
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

install(TARGETS scan_servos servo_cli
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Install the env config as a sample; .sample suffix prevents overwriting
# user-edited files on upgrade. The recipe renames it to servo.env.
install(FILES conf/servo.env
    DESTINATION ${CMAKE_INSTALL_SYSCONFDIR}/servo_sensei
    RENAME servo.env.sample
)

# ---------------------------------------------------------------------------
# YARP motor control plugin  (optional — requires YARP to be installed)
#
# Builds only when YARP is found on the system.  All other targets compile
# independently — no YARP dependency leaks into the core driver.
#
# Install:
#   cmake -B build -DBUILD_SHARED_LIBS=ON
#   cmake --build build
#   sudo cmake --install build
#
# Run wrapped (exposes RPC / state / command YARP ports):
#   yarpdev --device deviceBundler \
#           --wrapper_device controlBoard_nws_yarp \
#           --attached_device st_servo \
#           --from conf/st_servo.ini
# ---------------------------------------------------------------------------
find_package(YARP QUIET COMPONENTS dev os)

if(NOT YARP_FOUND)
    message(STATUS "YARP not found — skipping st_servo plugin (set CMAKE_PREFIX_PATH to enable)")
else()

message(STATUS "YARP ${YARP_VERSION} found — building st_servo plugin")

# Declare the plugin to the YARP device factory.
yarp_prepare_plugin(st_servo
    CATEGORY  device
    TYPE      STServoYarpDriver
    INCLUDE   yarp/STServoYarpDriver.hpp
    DEFAULT   ON
)

if(NOT SKIP_st_servo)
    yarp_add_plugin(st_servo
        src/yarp/STServoYarpDriver.cpp
        include/yarp/STServoYarpDriver.hpp
    )

    target_include_directories(st_servo PRIVATE include)
    target_link_libraries(st_servo PRIVATE YARP::YARP_dev YARP::YARP_os)
    target_compile_features(st_servo PRIVATE cxx_std_17)
    set_target_properties(st_servo PROPERTIES OUTPUT_NAME "yarp_st_servo")

    # Write the plugin manifest at configure time (supports generator expressions).
    file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/plugins)
    file(GENERATE
        OUTPUT  ${CMAKE_BINARY_DIR}/plugins/st_servo.ini
        CONTENT "[plugin st_servo]\ntype device\nname st_servo\nlibrary yarp_st_servo\nfilename $<TARGET_FILE:st_servo>\n"
    )

    install(TARGETS st_servo LIBRARY DESTINATION lib/yarp)
    install(FILES ${CMAKE_BINARY_DIR}/plugins/st_servo.ini DESTINATION share/yarp/plugins)
    install(FILES conf/st_servo.ini
        DESTINATION ${CMAKE_INSTALL_SYSCONFDIR}/servo_sensei
        RENAME st_servo.ini.sample
    )

    # -----------------------------------------------------------------------
    # fake_servo_bus — hardware-free debug simulator (Linux only)
    #
    # Creates a kernel PTY pair and forks yarpdev as a subprocess.
    # Usage:  ./fake_servo_bus --ids "1 2 3" --name /robot/debug_servo
    # -----------------------------------------------------------------------
    if(UNIX AND NOT APPLE)
        add_executable(fake_servo_bus
            app/fake_servo_bus/main.cpp
            app/fake_servo_bus/common.cpp
            app/fake_servo_bus/config.cpp
            app/fake_servo_bus/sim_bus.cpp
            app/fake_servo_bus/ui.cpp
        )
        target_include_directories(fake_servo_bus PRIVATE
            tests
            app/fake_servo_bus
        )
        target_compile_features(fake_servo_bus PRIVATE cxx_std_17)

        # Optional ncurses TUI — live servo-state table in the terminal
        find_package(Curses)
        if(CURSES_FOUND)
            target_link_libraries(fake_servo_bus PRIVATE ${CURSES_LIBRARIES})
            target_include_directories(fake_servo_bus PRIVATE ${CURSES_INCLUDE_DIRS})
            target_compile_definitions(fake_servo_bus PRIVATE HAVE_CURSES=1)
            message(STATUS "fake_servo_bus: ncurses TUI enabled")
        else()
            message(STATUS "fake_servo_bus: ncurses not found — text-mode only")
        endif()
    endif()
endif() # NOT SKIP_st_servo

# -----------------------------------------------------------------------
# YARP driver unit tests  (hardware-free, PTY-based; Linux only)
# -----------------------------------------------------------------------
option(BUILD_YARP_TESTS "Build STServoYarpDriver unit tests" ON)

if(BUILD_TESTING AND BUILD_YARP_TESTS AND NOT SKIP_st_servo)
    add_executable(test_yarp_driver
        tests/test_yarp_driver.cpp
        src/yarp/STServoYarpDriver.cpp
    )
    target_include_directories(test_yarp_driver PRIVATE include tests)
    target_link_libraries(test_yarp_driver PRIVATE
        GTest::gtest
        YARP::YARP_dev
        YARP::YARP_os
    )
    target_compile_features(test_yarp_driver PRIVATE cxx_std_17)
    gtest_discover_tests(test_yarp_driver)

    if(UNIX AND NOT APPLE)
        add_executable(test_fake_servo_bus
            tests/test_fake_servo_bus.cpp
        )
        target_include_directories(test_fake_servo_bus PRIVATE include)
        target_link_libraries(test_fake_servo_bus PRIVATE
            GTest::gtest_main
        )
        target_compile_features(test_fake_servo_bus PRIVATE cxx_std_17)
        target_compile_definitions(test_fake_servo_bus PRIVATE
            FAKE_SERVO_BUS_PATH="$<TARGET_FILE:fake_servo_bus>"
        )
        gtest_discover_tests(test_fake_servo_bus)
    endif()
endif()

endif() # YARP_FOUND

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
